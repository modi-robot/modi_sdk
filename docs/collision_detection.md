# 分臂碰撞检测与恢复

```cpp
std::vector<bool> flags;
const int result = client->Robot()->State()->CollisionDetection(flags);
if (result == 0) {
  // flags 与 Model()->GetActuatorJointNames() 同序；true 保留到清除成功。
} else {
  // 查询失败或已锁存检测运行错误；输出为空，不能解释为没有碰撞。
}

// 按 GetArmNames() 返回的机械臂名称设置和读取。Off 不清除已有锁存。
const auto arm_name = client->Robot()->Model()->GetArmNames().at(0);
client->Robot()->Manager()->SetCollisionSensitivity(arm_name, modi_sdk::CollisionSensitivity::kMedium);
modi_sdk::CollisionSensitivity sensitivity;
if (client->Robot()->Manager()->GetCollisionSensitivity(arm_name, sensitivity) == 0) {
  // 当前生效值：Off=0、Low=1、Medium=2、High=3。
}
// 操作者统一清除当前已锁存的碰撞；不会自动上电或续跑旧轨迹。
client->Robot()->Manager()->ClearCollision();
```

逐关节碰撞状态提供 `StateApi::CollisionDetection(std::vector<bool>&)` 和无参
`ManagerApi::ClearCollision()`，不提供按臂清除重载或公开碰撞结构体。
查询服务与状态发布读取 system 实时缓存，不重复调用 WBC、不推进滤波或确认计数。

灵敏度通过 `/Robot/Manager/GetCollisionSensitivity` 读取当前运行值，通过
`/Robot/Manager/SetCollisionSensitivity` 应用到当前运行实例，SDK 本身不写配置文件。
Scope 与安装位姿共用 `config/robot_config.yaml` 的用户覆盖/包内默认配置路径，
将灵敏度分别保存为 `collision_sensitivity.left` 和 `collision_sensitivity.right`（0=关闭、1=低、2=中、3=高）。
旧版顶层标量作为两臂初值，首次保存时迁移成映射并保留另一臂值；每次保存及失败回退只处理指定臂。
Scope 启动时先加载该值并调用 SDK 应用，再允许自动上电；缺项默认 Medium（2），非法值报告错误。
前端保存时先应用，再与安装位姿共用写文件流程保存；写入失败回退运行值并报告错误。
Getter 失败返回错误码并将输出设为 -1，不能将其解释为 Off 或 Medium。
SDK 的第一个参数为 `arm_name`；显式传入空字符串时：设置应用到全部臂；查询仅在所有臂档位相同时返回该值，否则返回错误。

## 分臂行为

固定基座位置控制下，每控制周期在 UpdateState 后、发出新目标前检测一次。
仅碰撞臂立即停止、保持上电并锁存；其他臂继续运动、拖动及检测。
已就绪检测的反馈或算法错误也停止对应臂，并保留内部逐臂错误码；不会伪造关节碰撞位。
普通运动请求跳过碰撞臂、拖动臂，响应注明跳过对象；没有可执行臂时返回错误。
每臂规划队列及 WBC 后台暂停独立处理，碰撞前的排队请求和迟到规划不会在恢复后提交。

`SafetyState::kCollision=4` 是汇总状态，并非全机器人运动权限。
任一臂仍锁存时保持 Collision；`RobotState` 继续反映实际电源状态。
急停、硬件/通信故障和退出具有更高优先级，仍执行原有全局停止与下电流程。
`ClearFaults`、`Estop(false)`、Off、上下电和关闭界面弹窗均不清除碰撞。

## 拖动、上电与清除

碰撞不阻止上电。已锁存臂上电时保持停止位，验证 CSP/Running 或 CSP/QuickStop；
未锁存臂仍要求 CSP/Running。显式下电后清除碰撞不会自动上电。

反馈有效、静止且无硬件故障时，未清除碰撞的臂也可开启拖动。
system 保持停止位准备 CSP 当前位置目标，再随该目标解除驱动停止。
停止保持可能反馈为 CSV/Running（`0x2003`），也可能反馈 QuickStop；解除前依据已执行的停止、
已准备的 CSP 目标及静止反馈校验，不要求先返回 CSP 或 QuickStop，解除后再严格确认 CSP/Running；
等待期间持续发送当前位置保持 PDO，确认成功后才进入 CST 拖动。
恢复位置控制时允许固定目标下的短暂调节；需连续 50 ms 速度均不超过 `5°/s`（约 `0.0872665 rad/s`）
才完成恢复。目标不跟随该调节移动，软件锁存保留；持续调节超过 1 s 会重新停止。
软件锁存与触发关节继续保留。退出拖动后若仍锁存，恢复 CSP 保持并重新执行停止。
允许再次开启；失败重新停止对应臂，不恢复旧轨迹。

`ClearCollision()` 只处理当前已锁存的臂，不复位其他臂。
正在拖动的臂只清除软件锁存，继续拖动；其他臂验证静止，以 CSP 当前位置目标解除驱动停止，
等待 CSP/Running 确认后再清除软件锁存。已下电的臂确认 Ready/Disabled 即可，不自动使能。
逐臂执行，成功臂不回滚，失败臂继续锁存，接口返回首个错误。

恢复失败由后台日志记录 `prepare_csp_hold`、`wait_csp_running`、`enable_cst` 或
`exit_free_drive` 阶段及关节、实际模式、伺服状态和错误码。
日志另含 `reason`、`velocity_rad_s`、静止阈值、HAL 读取错误码及反馈长度，区分读取失败和未静止。
工具按钮的 `requested` 只表示接受请求，`Free-drive enabled ... CST feedback confirmed` 才表示拖动已使能。

拖动、其他力控、辨识及本臂后台独占期间暂停该臂检测；恢复位置控制时重置历史。
查询 false 仅表示没有已锁存的关节触发，并不代表检测参数已经就绪。
全零摩擦参数允许上电但跳过本臂检测，日志明确记录未就绪。Kt 来自初始化时的关节读值缓存。
不新增 PDO 序号或时间戳，反馈有效性沿用 SDK 原有超时检查。

## scope 界面

关节紧急报文区域独立显示各臂碰撞与检测异常，保留真实 EMCY 报文。
新碰撞自动弹出提示；同次锁存不反复弹出，另一臂新增碰撞或清除后再次碰撞会重新提示。
可直接关闭弹窗并通过卡片重开；关闭不改变锁存或控制模式。
弹窗只有一个“清除碰撞”按钮，通过 `POST /api/manager/clear_collision` 调用统一清除，
不携带臂名，与“清除故障”分别处理。Collision 不禁用上电和拖动，普通运动按臂判断权限。

SDK、system、scope、HAL 与生成的通信类型需一起重新构建。
模拟测试之后，仍需在实机低速场景验证分臂停止、保持上电和拖动恢复。
