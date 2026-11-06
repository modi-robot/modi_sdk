# 摩擦参数辨识与关节写入

摩擦辨识由 `modi_system` 按机械臂管理。SDK 使用 `Robot()->Calibration()` 启动、
查询、取消和取得结果；当前关节参数的读取和异步写入在 `Robot()->Model()`。
臂名使用 `Model()->GetArmNames()` 的实际模型名称，每臂矩阵行顺序与该臂关节顺序一致。

```cpp
int FrictionIdentificationEnable(const std::string& arm_name);
int GetCalibrationStatus(const std::string& arm_name, CalibrationTaskStatus& status);
int GetFrictionIdentificationResult(const std::string& arm_name,
                                  std::vector<std::vector<double>>& identified_params,
                                  std::vector<double>* residuals_a = nullptr);
int CancelCalibration(const std::string& arm_name);
// Model():
int GetJointFrictionParams(const std::string& arm_name,
                          std::vector<std::vector<double>>& params);
int SetJointFrictionParams(const std::string& arm_name,
                          const std::vector<std::vector<double>>& params);
int SetCollisionResiduals(const std::string& arm_name,
                          const std::vector<double>& residuals_a);
```

状态只有 `kIdle=0`、`kRunning=1`、`kSucceeded=2`、`kCancelled=3`、`kFailed=4`。
启动和写入返回 0 表示受理，操作在服务端异步继续，SDK 断连不自动取消。
同一臂的辨识、停止处理和写入共用一个状态，不提供任务 ID 或历史查询。

`GetCalibrationStatus` 查询成功时更新输出；操作失败时输出 `kFailed`，返回该操作的
负错误码；查询本身失败时保持输出不变。正常取消的返回值是 0。
DDS 响应 `error_code` 表示查询错误，`operation_error_code` 表示被查询操作错误，
SDK 将后者转换成公开函数返回值。不要因为返回值非零就忽略已经更新的 `kFailed`。

## 辨识

目标臂需已上电、Robot=Running、Safety=Normal、位置控制，且规划器空闲、实测静止。
同一臂不允许重复操作；双臂可同时辨识。辨识期间，影响辨识臂的普通运动、力控和
模型修改返回 `kCalibrationBusy`。当前普通运动服务按全部臂提交，因此整条请求会被拒绝；
命令 topic 同样受仲裁。普通停止接口转为取消正在进行的辨识。

1. 保护模型配置、工具负载、安装姿态、世界重力、控制周期和 Kt，禁止配置修改。
   Kt 来自 HAL `GetJointTorqueConstant()`；CanEvo 返回初始化时从关节固件读取、校验的缓存，仿真返回配置值。
2. 在独立后台线程检查数据目录，生成完整标准轨迹并预检位置、速度与
   0.5 rad/s² 加速度界限；不自动裁剪轨迹。
3. 一次 `MoveJoint(first_point, 0.1, 0.5)` 定位。规划结束后所有关节误差
   ≤0.01 rad、速度 ≤0.01 rad/s 连续 0.5 s；10 s 内未稳定则失败。
4. 实时线程通过 HAL 每控制周期下发一行位置，下一周期分别调用现有
   `GetJointsPositions()`、`GetJointsVelocities()`、`GetJointsCurrents()`，
   将位置、速度、电流保存到预分配内存。最后一点多保留一个采样周期，
   N 个点对应 N 行反馈。
   HAL 读取失败、维度异常、非有限值、下发失败或采样不完整都会终止。
5. 在独立后台线程的拟合入口复制一次动力学输入，再拟合并保存，通过后发布 N×5 `[Fs,Fc,vs,p,B]`。
   同时保存配套的逐关节碰撞残差 `residuals_a`，保持结束位置，不自动返回或写入关节。

标准轨迹约需 446 s（5 ms 周期），所有关节同步在 ±π/6 内运动，五档速度、
每档两次往返。算法只辨识 Fc、B，输出 `[Fc,Fc,0.02,2,B]`，详细规则见 WBC
`docs/friction_identification_usage.md`。轨迹超出模型限制时直接失败。

实时线程只处理输出、反馈和状态切换；每臂后台线程负责生成、拟合与保存。
服务端直接调用 Interface 的同步生成、拟合接口，WBC 不创建后台线程或配置锁。
生成轨迹只读取周期和关节数量，不依赖 Kt 或动力学模型。拟合入口复制当前模型、
关节顺序、周期、Kt 和固定安装位姿，使用局部 q/v/a 与独立 Pinocchio Data，
不读取在线 RobotWrapper::state.q。安装位姿通过现有设置与 Reset 流程同步；
位姿未变化时不写入。普通关节状态更新不修改该位姿，Reset 不复制动力学模型，
参数写入也不准备动力学模型。此流程针对固定安装基座，不支持采样期间基座运动。

采集按控制周期及数组行号对应，不记录接收序号或时间戳，不判断重复读取同一份
PDO 缓存，也不根据实际采样时间校正数据。三个查询分别读取缓存，不保证来自同一帧。
CanEvo 内部原有快照一致性和 PDO 超时机制保留。定位稳定时间、定位及停止超时
仍使用控制流程计时，不进入拟合数组。

system 从受理任务起禁止模型配置修改，直至后台任务完成拟合、保存和退出。
任一臂仍有活动操作、未结束的后台任务或参数读取占用时，全局配置请求返回
`kCalibrationBusy`，不等待或自动重试。服务、topic 和排队请求在实际执行前检查。
直接调用 WBC 的调用方必须自行保证生成及拟合期间模型配置和安装位姿不变；
可以继续更新普通关节状态，同一臂的算法调用须串行。

## 取消和拖动

`CancelCalibration` 只受理请求。运动中的辨识撤销后续轨迹输出，按 8.0 rad/s²
减速停止，实测速度稳定后才进入 `kCancelled`；停止处理失败进入 `kFailed`。
停止超时或控制器复位失败同时触发现有安全故障流程，防止继续旧规划。
准备、拟合阶段可以立即逻辑取消，后台迟到结果不能发布。
后台仍在退出时，同一臂的新操作和全局配置修改继续返回忙，即使对外已为
`kCancelled` 或 `kFailed`；后台完成后才释放配置占用。

工具按钮有效按下或 SDK `Force()->EnableFreeDrive(arm_name)` 会先取消辨识，
减速结束后按现有流程进入拖动。按钮在停止期间释放则只取消，不进入拖动。
拖动接管不等待完整拟合。若旧后台尚未退出，拖动初始化沿用受保护的 WBC 重力，
不重新写入模型；日志注明沿用配置。下一次在后台已结束时开启拖动，再读取 HAL
重力进行更新。拖动开启或退出都不解除旧后台的配置占用。
SDK 拖动调用等待接管，允许最长 15 s 的服务响应时间。
退出拖动或拖动初始化失败均不恢复旧轨迹。单臂接管不取消另一臂。
参数写入阶段保持原有互斥规则，不允许拖动抢占阻塞中的 SDO。

## 参数写入

`Model()->GetJointFrictionParams(arm_name, params)` 读取 N×5 `[Fs,Fc,vs,p,B]`；
`Model()->GetJointCollisionResiduals(arm_name, residuals_a)` 单独读取逐关节 Ri（A）。
读取在非实时线程执行并沿用辨识读预约，失败清空输出。

设置也分为两个独立接口和服务：
- `SetJointFrictionParams(arm_name, params)`：只写入协议前五项，不修改碰撞残差。
- `SetCollisionResiduals(arm_name, residuals_a)`：只写入协议第六项，不修改摩擦参数。

两个接口返回 0 仅表示受理，均通过 `GetCalibrationStatus()` 查询写入及回读校验结果。
Ri 必须与该臂关节同序、等长且为 0～65.535 A 的有限值。需要同时保存时，先提交
摩擦参数，等待 `kSucceeded`，再提交碰撞残差并等待成功；失败或取消不继续下一步。
每个写入可能部分完成，不自动回滚；协议第七项始终不修改。

`GetFrictionIdentificationResult(arm_name, params, &residuals_a)` 返回同一次成功辨识的
五项参数及配套残差，调用方显式使用上述两个接口保存。新辨识受理时清除旧结果，
失败或取消不发布新结果。不会再按摩擦参数是否匹配辨识结果隐式写入残差。
写入流程不直接调用 WBC `SetFrictionParams`；服务端在下一次
实际上电前从 CanEvo HAL 读取五项参数及碰撞残差并应用到 WBC，同时应用机型配置的
`friction_compensation_Scale`（缺省每关节 `0.6`）。读取或设置失败会中止上电。

写入前完整校验：有限值、范围 [0,65.535]、Fs≥Fc≥0、vs>0、p>0、B≥0。
千倍缩放、整数取整只在 CanEvo SDK 中执行。所有关节写入后回读五项，按
量化后的 uint16 数值逐项比较；配套残差也须回读校验一致，全部通过才成功。

目标臂必须静止并处于位置控制；任一臂辨识期间不读写摩擦 SDO。
同一臂读取与写入串行，写入期间不启动新辨识。写入可能部分成功，不回滚；
取消等待当前 HAL 调用结束。HAL 在每关节写入后触发 Flash 保存并等待保存成功确认。

新辨识被受理会清除旧结果，失败或取消后没有该次有效结果。参数写入保留之前的
辨识结果，无论写入成功还是失败。被拒绝请求不覆盖当前状态、错误和结果。

## 错误与保存

| SystemError | 数值 | 原因 |
|---|---:|---|
| kCalibrationBusy | -104008 | 操作冲突或后台尚未退出 |
| kCalibrationResultUnavailable | -104009 | 没有有效辨识结果 |
| kCalibrationPositioningTimeout | -104010 | 初始定位后未稳定 |
| kCalibrationFeedbackInvalid | -104011 | HAL 反馈失败、维度或数值异常、采样不完整 |
| kCalibrationSamplingInterrupted | -104012 | 保留错误码；当前固定周期采集不检测 Tick 时间差 |
| kCalibrationStorageFailed | -104013 | 数据目录或保存失败 |
| kCalibrationParamVerifyFailed | -104014 | 参数回读与目标不符 |

其他错误复用 System/WBC/HAL/SDK 目录。摩擦参数读取沿用 vector 接口，空结果或
维度异常映射为 HAL 读取失败；写入保留具体 SDO 等下层错误返回。
首个操作错误优先，后续清理和诊断保存错误追加日志。

可选配置 `system.calibration_output_directory` 指定服务端保存根目录；默认使用
`/tmp/modi/calibration/<robot_id>/`。每臂按时间建立独立目录，日志输出路径。
默认目录属于临时数据，可能随系统重启或临时目录清理被删除，需要保留时请另行归档。

目录保存系统 YAML、启动配置（周期、限位、基座、重力和 Kt）、完整轨迹、
位置/速度/电流、辨识结果、写入目标/回读及操作诊断，不保存反馈序号和接收时间。
辨识残差保存为 `identified_residuals_a.csv`，配套写入时另存
`requested_residuals_a.csv` 和 `readback_residuals_a.csv`，每行按该臂关节顺序排列，单位 A。
`operation.txt` 记录内部阶段、首个错误和取消情况。完整动力学描述和每关节拟合质量
（RMSE、留出误差、重复性、预测变化及失败原因）写入服务端日志。
失败和取消保存已有采样；后台保存不阻塞拖动。服务重启恢复 Idle，磁盘记录保留，
不自动恢复操作。拟合与必须的数据保存全部完成后才发布成功。

SDK 示例：

```bash
single_arm_example_friction_identification <robot_id> <arm_name>
single_arm_example_friction_identification <robot_id> <arm_name> --write
dual_arm_example_friction_identification <robot_id>
dual_arm_example_friction_identification <robot_id> --write
```

示例要求机器人事先处于可运行状态，不自动上电；Ctrl-C 请求取消。
双臂先同时辨识，全部成功后才按臂写入。写入状态为 `kSucceeded` 表示服务端
已完成回读核对；示例随后读取并打印当前参数，调用方无需乘除 1000 或转换成 float。
