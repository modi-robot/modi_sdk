# MODI SDK 接口文档

本文以表格形式说明 `modi_sdk` 的公开 C++ 接口。函数签名和默认值以
`include/modi_sdk/` 下的头文件为准。

## 1. 通用约定

| 项目 | 说明 |
|---|---|
| 命名空间 | `modi_sdk` |
| 主头文件 | `#include "modi_sdk/robot_client.hpp"` |
| 控制/配置返回值 | 通常 `0` 表示成功，负值表示参数、通信、超时或服务端拒绝等错误 |
| 状态查询失败 | 可能返回空向量、负值或 `valid=false` |
| 关节单位 | rad、rad/s、rad/s²、N·m |
| 位姿 | `[x,y,z,roll,pitch,yaw]`，单位 m/rad |
| 六维速度 | `[vx,vy,vz,wx,wy,wz]`，单位 m/s、rad/s |
| 六维力 | `[fx,fy,fz,tx,ty,tz]`，单位 N/N·m |

## 2. RobotClient

### 2.1 Config

| 字段 | 默认值 | 参数说明 |
|---|---:|---|
| `domain_id` | `0` | DDS Domain ID，必须与 `modi_system` 一致。 |
| `spin_frequency_hz` | `1000.0` | 实时话题线程频率，单位 Hz。 |
| `spin_cpu_id` | `3` | 实时线程绑定的逻辑 CPU 编号。 |
| `spin_priority` | `90` | 实时线程 `SCHED_FIFO` 优先级。 |
| `logger` | `nullptr` | 外部 spdlog 日志器；为空时 SDK 创建默认日志器。 |

### 2.2 客户端接口

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `RobotClient()` | 无 | 客户端对象 | 使用默认配置。 |
| `RobotClient(int domain_id)` | `domain_id`：DDS Domain ID | 客户端对象 | 使用指定 Domain ID。 |
| `RobotClient(const Config& config)` | `config`：完整客户端配置 | 客户端对象 | 使用完整配置。 |
| `RegisterSubscriber(topic_names, callback)` | `topic_names`：订阅话题列表；`callback`：实时回调 | `0` 成功，负值失败 | 必须在 `Start()` 前调用。 |
| `RemoveSubscriber(topic_name)` | `topic_name`：待移除话题 | `0` 成功，负值失败 | 移除订阅。 |
| `RegisterPublisher(topic_names, callback)` | `topic_names`：发布话题列表；`callback`：样本填充回调 | `0` 成功，负值失败 | 必须在 `Start()` 前调用。 |
| `RemovePublisher(topic_name)` | `topic_name`：待移除话题 | `0` 成功，负值失败 | 移除发布。 |
| `Start()` | 无 | `true` 成功，`false` 失败 | 启动 DDS 与实时线程。 |
| `Stop()` | 无 | 无 | 停止客户端，可重复调用。 |
| `Robot()` | 无 | `std::shared_ptr<RobotApi>` | 获取机器人功能门面。 |
| `Chassis()` | 无 | `std::shared_ptr<ChassisApi>` | 获取底盘功能门面。 |

`Parser::Value<MsgT>(topic_name)` 返回指定订阅话题的最新消息副本；名称无效、未注册
或类型不匹配时返回 `MsgT{}`。`Publisher::Mutable<MsgT>(topic_name, value)` 成功返回
`0`，话题无效或类型不匹配返回 `-1`。

### 2.3 实时回调辅助接口

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `Parser::Value<MsgT>(topic_name)` | `MsgT`：话题消息类型；`topic_name`：已注册订阅话题 | 最新 `MsgT` 副本；失败返回 `MsgT{}` | 在订阅回调内读取最新消息。 |
| `Publisher::Mutable<MsgT>(topic_name, value)` | `MsgT`：话题消息类型；`topic_name`：已注册发布话题；`value`：本周期样本 | `0` 成功，`-1` 失败 | 在发布回调内更新待发送消息。 |

## 3. RobotApi

| 获取方法 | 返回值 | 功能 |
|---|---|---|
| `Model()` | `std::shared_ptr<ModelApi>` | 模型、工具和安装位姿。 |
| `State()` | `std::shared_ptr<StateApi>` | 关节、末端和故障状态。 |
| `Motion()` | `std::shared_ptr<MotionApi>` | 轨迹、速度和伺服控制。 |
| `Force()` | `std::shared_ptr<ForceApi>` | 力控配置。 |
| `Algo()` | `std::shared_ptr<AlgoApi>` | 运动学、动力学和标定。 |
| `Manager()` | `std::shared_ptr<ManagerApi>` | 电源、安全和故障管理。 |
| `Io()` | `std::shared_ptr<IoApi>` | 数字 IO。 |
| `Eef()` | `std::shared_ptr<EefApi>` | 工具板 EG2 夹爪。 |

### 3.1 ModelApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `GetAllJointDOF()` | 无 | 全部关节自由度；失败为负值 | 包含非驱动关节。 |
| `GetActuatorJointDOF()` | 无 | 驱动关节自由度；失败为负值 | 运动目标长度应与此一致。 |
| `GetAllJointNames()` | 无 | 全部关节名；失败为空 | 顺序与系统配置一致。 |
| `GetActuatorJointNames()` | 无 | 驱动关节名；失败为空 | 与状态和目标数组顺序一致。 |
| `GetActuatorJointLimits(lower, upper)` | `lower`、`upper`：输出上下限 | `0` 成功，负值失败 | 单位 rad。 |
| `GetActuatorJointVelocityLimits(limits)` | 输出每关节最大速度幅值 | `0` 成功，失败清空输出 | rad/s，来自 WBC Model 的 URDF velocity。 |
| `GetActuatorJointCurrentLimits(limits)` | 输出每关节最大电流幅值 | `0` 成功，失败清空输出 | A，来自 WBC Model 的 URDF effort 原值（本项目约定）。 |
| `GetActuatorJointAccelerationLimits(limits)` | 输出每关节最大加速度幅值 | `0` 成功，失败清空输出 | rad/s²，来自 RobotConfig 注入的 WBC 配置。 |
| `GetEndEffectorVelocityLimits(translation, rotation)` | 输出各末端平移和旋转速度模长上限 | `0` 成功，失败清空两个输出 | m/s、rad/s，来自 RobotConfig 注入的 WBC 配置；单臂一项、双臂两项。 |
| `GetEndEffectorNames()` | 无 | 末端坐标系名称；失败为空 | 外层末端数组顺序依据。 |
| `GetRobotName()` | 无 | 顶层描述配置名称；失败为空 | 对应 `modi_config.yaml` 的 `system.robot_name`。 |
| `GetArmNames()` | 无 | 机械臂模型名称；失败为空 | 按 `arm.groups[]` 配置顺序返回。 |
| `AddToolModel(tool_name, ee_name, frame_xyzrpy, mass, com, inertia)` | 工具名、末端名、位姿、质量 kg、质心 m、惯量 | `0` 成功，负值失败 | 添加工具与负载模型。 |
| `RemoveToolModel(tool_name)` | `tool_name`：工具名 | `0` 成功，负值失败 | 删除工具模型。 |
| `GetTcpOffset(name, pose)` | `name`：工具/末端名；`pose`：输出位姿 | `0` 成功，负值失败 | 获取 TCP 偏移。 |
| `GetPayload(name, mass, com, inertia)` | `name`：工具名；其余为输出 | `0` 成功，负值失败 | 获取负载参数。 |
| `GetControlPeriod()` | 无 | 控制周期，单位 s；失败时非正值 | 获取系统控制周期。 |
| `LoadCollisionPairs(path)` | `path`：碰撞对配置文件 | `0` 成功，负值失败 | 加载自碰撞检测对。 |
| `RemoveAllCollisionPairs()` | 无 | `0` 成功，负值失败 | 清空碰撞检测对。 |
| `GetFbasePosition()` | 无 | 每条机械臂的基座位姿；失败为空 | 外层顺序与 `GetArmNames()` 一致。 |
| `SetInstallationPosition(poses)` | `poses`：每条机械臂一个六维位姿 | `0` 成功，负值失败 | 顺序与 `GetArmNames()` 一致。 |

### 3.2 StateApi

以上关节限制按 `GetActuatorJointNames()` 排列，末端限制按 `GetEndEffectorNames()`
排列，双臂依次拼接。任一臂数据不可用则整个查询失败，不返回部分结果。
新增接口只提供限制数据，不增加运动规划器的速度或加速度限速行为。

以下关节数组按机器人、臂组、关节的系统配置顺序排列。

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `GetControlMode()` | 无 | `std::vector<ControlMode>`；失败为空 | 每臂一个模式，与 `ModelApi::GetArmNames()` 同序；未知值为 `kUnknown`。 |
| `GetActuatorJointPositions()` | 无 | 关节位置数组，rad；失败为空 | 驱动关节位置。 |
| `GetActuatorJointVelocities()` | 无 | 关节速度数组，rad/s；失败为空 | 驱动关节速度。 |
| `GetActuatorJointAccelerations()` | 无 | 关节加速度数组，rad/s²；失败为空 | 驱动关节加速度。 |
| `GetActuatorJointTorquesData()` | 无 | 关节力矩数组，N·m；失败为空 | 驱动关节力矩。 |
| `GetJointsCurrents()` | 无 | 关节实际转矩电流数组，A；失败为空 | 数据来自驱动器实际电流反馈。 |
| `GetJointsVoltages()` | 无 | 关节母线电压数组，V；失败为空 | 数据来自驱动器母线电压反馈。 |
| `GetJointsTemperatures()` | 无 | 关节电机温度数组，℃；失败为空 | 数据来自驱动器电机温度反馈。 |
| `CheckActuatorJointLimits()` | 无 | 每关节限位标志；失败为空 | `true` 表示触发限位。 |
| `GetServoState()` | 无 | `ServoState` 数组；失败为空 | 每个数值含义见 5.3 节。 |
| `GetJointFaultCode()` | 无 | `FaultCode` 数组；失败为空 | 当前故障，每个故障码见 5.4 节。 |
| `GetJointEmcyCode()` | 无 | `FaultCode` 数组；无事件为 `kNone` | 读取后消费 EMCY 事件，故障码见 5.4 节。 |
| `GetJointWarningCode()` | 无 | `WarningCode` 数组；失败为空 | 每个警告码见 5.5 节。 |
| `GetEndEffectorPoses()` | 无 | 每末端一个六维位姿；失败为空 | 单位 m/rad。 |
| `GetEndEffectorTwists()` | 无 | 每末端一个六维速度；失败为空 | 单位 m/s、rad/s。 |
| `GetEndEffectorAccels()` | 无 | 每末端一个六维加速度；失败为空 | 线/角加速度。 |
| `GetEndEffectorWrenchesData()` | 无 | 每末端一个六维力；失败为空 | 单位 N/N·m。 |
| `GetComPose()` | 无 | 每机型的质心位姿；失败为空 | 外层顺序与配置一致。 |
| `GetFbasePosition()` | 无 | 每机型的浮动基位姿；失败为空 | 外层顺序与配置一致。 |
| `SelfCollisionsDetection()` | 无 | 碰撞名称对数组；无碰撞为空 | 每项为发生碰撞的两个对象名。 |

### 3.3 MotionApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `MoveJoint(target_joints, velocity, acceleration, blocked=false)` | 目标关节 rad；速度 rad/s；加速度 rad/s²；是否阻塞 | `0` 成功，负值失败 | 关节点到点运动。 |
| `MoveLine(target_poses, velocity, acceleration, blocked=false)` | 每末端目标位姿；速度；加速度；是否阻塞 | `0` 成功，负值失败 | 笛卡尔直线运动。 |
| `MovePose(target_poses, velocity, acceleration, blocked=false)` | 每末端目标位姿；速度；加速度；是否阻塞 | `0` 成功，负值失败 | 笛卡尔点到点运动。 |
| `MoveCircle(via1, via2, alpha, velocity, acceleration, blocked=false)` | 两组途经点；各末端圆弧角；速度；加速度；是否阻塞 | `0` 成功，负值失败 | 圆弧或整圆运动。 |
| `SpeedLine(target_twists, acceleration=0.2)` | 每末端六维速度；加速度 | `0` 成功，负值失败 | 末端速度控制。 |
| `SpeedJoint(joint_velocities, acceleration=0.5)` | 关节速度；加速度 | `0` 成功，负值失败 | 关节速度控制。 |
| `ServoLine(target_poses, period)` | 每末端目标位姿；周期 s | `0` 成功，负值失败 | 周期末端位姿控制。 |
| `ServoJoint(target_joints, period, target_velocity={}, target_acceleration={})` | 位置；周期 s；可选速度和加速度 | `0` 成功，负值失败 | 周期关节控制。 |
| `StopMotion(acceleration=8.0)` | 停止减速度 | `0` 成功，负值失败 | 停止当前运动。 |
| `GetPlannerStatus()` | 无 | `PlannerStatus` 整数值；失败 `-1` | 获取规划器状态。 |

### 3.4 ForceApi

二维参数外层顺序与 `GetEndEffectorNames()` 一致。

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `FcEnable(controller_type=0)` | `ControllerType` 对应整数 | `0` 成功，负值失败 | 启用力控。 |
| `FcDisable()` | 无 | `0` 成功，负值失败 | 禁用力控。 |
| `EnableFreeDrive(arm_name)` | `arm_name` 必须来自 `ModelApi::GetArmNames()` | `0` 成功，负值失败 | 为指定机械臂启用拖动示教；顶层聚合型号不可作为臂名。 |
| `DisableFreeDrive(arm_name)` | `arm_name` 必须来自 `ModelApi::GetArmNames()` | `0` 成功，负值失败 | 关闭拖动示教，并以退出瞬间实测位置切回 CSP 保持。 |
| `IsFreeDriveEnabled(arm_name)` | `arm_name` 必须来自 `ModelApi::GetArmNames()` | 完整进入 CST 且实时力矩持续有效时为 `true`；查询失败也为 `false` | 查询指定机械臂拖动示教状态。 |
| `GetBaseGravity(arm_name)` | `arm_name` 必须来自 `ModelApi::GetArmNames()` | `{gx, gy, gz}`，失败为空 | 从 HAL 获取指定机械臂 URDF `base_link` 中的测量重力，单位 `m/s²`；已应用 `gravity_sensor_position` 的 sensor→base 旋转。拖动启用时后端再结合 `T_world_base` 转为 world 重力传给 WBC。DDS 服务为 `/Robot/Force/GetBaseGravity`。 |
| `SetTaskFrame(types, frame_offsets)` | 坐标系类型；每末端六维偏移 | `0` 成功，负值失败 | 设置任务坐标系。 |
| `SetRefPoint(ref_points)` | 每末端参考点 | `0` 成功，负值失败 | 设置力控参考点。 |
| `SetGoalWrench(wrenches)` | 每末端六维目标力 | `0` 成功，负值失败 | 设置目标力。 |
| `SetWrenchThreshold(wrenches)` | 每末端六维阈值 | `0` 成功，负值失败 | 设置力阈值。 |
| `SetWrenchOffset(wrenches)` | 每末端六维偏移 | `0` 成功，负值失败 | 设置力传感器偏移。 |
| `SetCartesianSelectVector(selects)` | 每末端六自由度选择标志 | `0` 成功，负值失败 | 选择笛卡尔力控自由度。 |
| `SetJointSelectVector(selects)` | 每关节选择标志 | `0` 成功，负值失败 | 选择关节力控自由度。 |
| `SetCartesianFcParams(damps, stiffnesses)` | 每末端阻尼、刚度 | `0` 成功，负值失败 | 设置笛卡尔力控参数。 |
| `SetJointFcParams(damps, stiffnesses)` | 每关节阻尼、刚度 | `0` 成功，负值失败 | 设置关节力控参数。 |
| `SetNullSpaceParams(damps, stiffnesses)` | 零空间阻尼、刚度 | `0` 成功，负值失败 | 设置零空间参数。 |
| `GetGoalWrenches(wrenches)` | `wrenches`：输出参数 | `0` 成功，负值失败 | 获取目标六维力。 |
| `GetRefPointWrenches(wrenches)` | `wrenches`：输出参数 | `0` 成功，负值失败 | 获取参考点六维力。 |

拖动示教按机械臂独立启停。聚合运动或传统力控命令会跳过已经进入拖动示教的臂，
其余臂仍可运行；所有目标臂均处于拖动示教时返回模式冲突。
`ManagerApi::PowerOff()`、`BrakeLock()`、急停、故障和进程退出都会先关闭所有臂的
拖动示教，停止 CST 力矩输出，再继续抱闸和断使能。

### 3.5 AlgoApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `IkPosSolver(ref_q, target_poses, result_q, pos_err=0.001, ori_err=0.0873, iterations=100)` | 参考关节、目标位姿、输出关节、误差阈值、迭代次数 | `0` 成功，负值失败 | 位置级逆运动学。 |
| `IkSolver(ref_q, ref_dq, target_poses, result_q, result_dq, result_ddq, ...)` | 参考状态、目标位姿、输出位置/速度/加速度及求解参数 | `0` 成功，负值失败 | 完整逆运动学。 |
| `FkPosSolver(joint_positions, result_poses)` | 关节位置；输出末端位姿 | `0` 成功，负值失败 | 位置级正运动学。 |
| `FkSolver(q, dq, ddq, poses, twists, accelerations)` | 关节状态；输出末端位姿、速度、加速度 | `0` 成功，负值失败 | 完整正运动学。 |
| `IdSolver(q, dq, ddq, torques)` | 关节状态；输出力矩 | `0` 成功，负值失败 | 逆动力学。 |
| `SetFrameTaskWeight(frame_name, task_type, position_weight, orientation_weight)` | 坐标系名、任务类型、位置/姿态权重 | `0` 成功，负值失败 | 设置坐标系任务权重。 |
| `GetFrameTaskWeight(frame_name, task_type, position_weight, orientation_weight)` | 坐标系名；其余为输出 | `0` 成功，负值失败 | 获取坐标系任务权重。 |
| `SetPostureJointWeight(joint_name, weight)` | 关节名、权重 | `0` 成功，负值失败 | 设置姿态关节权重。 |
| `GetPostureJointWeight(joint_name)` | 关节名 | 权重；失败值由服务定义 | 获取姿态关节权重。 |
| `SetKineticEnergyTaskWeight(weight)` | 权重 | `0` 成功，负值失败 | 设置动能任务权重。 |
| `GetKineticEnergyTaskWeight()` | 无 | 权重；失败值由服务定义 | 获取动能任务权重。 |
| `SetComConstraintSquareSize(side_length)` | 正方形边长，单位 m | `0` 成功，负值失败 | 设置质心约束范围。 |
| `GetComConstraintSquareSize()` | 无 | 边长；失败值由服务定义 | 获取质心约束范围。 |
| `MaskDof(joint_name)` | 关节名 | `0` 成功，负值失败 | 屏蔽自由度。 |
| `UnmaskDof(joint_name)` | 关节名 | `0` 成功，负值失败 | 解除屏蔽。 |
| `CalibrationPayload(sensor_in_flange, poses, sensor_datas, mass, com, inertia, offsets)` | 传感器安装位姿、采样位姿/数据；其余为输出 | `0` 成功，负值失败 | 标定质量、质心、惯量和力偏移。 |

### 3.6 ManagerApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `PowerOn()` / `PowerOff()` | 无 | `0` 成功，负值失败 | 机器人上下电。 |
| `BrakeRelease()` / `BrakeLock()` | 无 | `0` 成功，负值失败 | 释放或锁定抱闸。 |
| `ClearFaults()` | 无 | `0` 成功，负值失败 | 清除可恢复故障。 |
| `GetCollisionSensitivity(arm_name, sensitivity)` | 机械臂名称；`CollisionSensitivity&` 输出 | `0` 成功，负值失败；失败输出 -1 | 读取指定臂碰撞等级：Off=0、Low=1、Medium=2、High=3；空名称要求各臂等级一致，否则返回错误。 |
| `SetCollisionSensitivity(arm_name, sensitivity)` | 机械臂名称；`CollisionSensitivity` | `0` 成功，负值失败 | 立即应用到指定臂；显式传入空名称时应用到全部臂。Scope 按臂保存到 `config/robot_config.yaml`，与安装位姿同一路径，并在启动时通过 SDK 恢复。 |
| `GetRobotDomainId()` | 无 | Domain ID；失败 `-1` | 获取配置中的 Domain ID。 |
| `SetRobotDomainId(domain_id)` | 新 Domain ID | `0` 成功，负值失败 | 修改后需重启 System/SDK。 |
| `Estop(trigger)` | `true` 触发，`false` 清除 | `0` 成功，负值失败 | 控制急停。 |
| `GetSafetyState()` | 无 | `SafetyState` 整数；失败 `-1` | 获取安全状态机。 |
| `GetRobotState()` | 无 | `RobotState` 整数；失败 `-1` | 获取机器人状态机。 |

### 3.7 IoApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `SetBoardDo(channel, value)` | 控制柜通道号、输出值 | `0` 成功，负值失败 | 设置控制柜 DO。 |
| `GetBoardDi()` / `GetBoardDo()` | 无 | DI/DO 状态数组；失败为空 | 获取控制柜 IO。 |
| `SetToolDo(channel, value)` | 工具端通道号、输出值 | `0` 成功，负值失败 | 设置工具端 DO。 |
| `GetToolDi()` / `GetToolDo()` | 无 | DI/DO 状态数组；失败为空 | 获取工具端 IO。 |

### 3.8 EefApi

通过机械臂工具板 RS485 连接的 EG2 夹爪，使用同一个 `EefApi` 下的
`GetToolGripper*`、`SetToolGripper*` 和 `ClearToolGripperFault` 接口。
这些接口按机械臂名称定位，每臂使用第一块工具板、夹爪地址 1，具体参数、反馈结构和命令行示例见
[工具板夹爪接口](工具板夹爪接口.md)。

## 4. ChassisApi

### 4.1 ChassisModelApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `GetSteeringJointNames()` | 无 | 转向关节名；失败为空 | 与转向状态顺序一致。 |
| `GetWheelJointNames()` | 无 | 轮关节名；失败为空 | 与轮速状态顺序一致。 |
| `GetWheelRadius()` | 无 | 轮半径 m；失败时非正值 | 获取车轮半径。 |
| `GetControlPeriod()` | 无 | 控制周期 s；失败时非正值 | 获取底盘控制周期。 |
| `GetMaxBaseLinearSpeed()` | 无 | 最大线速度 m/s | 获取线速度限制。 |
| `GetMaxBaseAngularSpeed()` | 无 | 最大角速度 rad/s | 获取角速度限制。 |
| `SetMaxBaseLinearSpeed(speed)` | 最大线速度 m/s | `0` 成功，负值失败 | 设置线速度限制。 |

### 4.2 ChassisStateApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `GetSteeringPositions()` | 无 | 转向位置 rad；失败为空 | 顺序与转向关节名一致。 |
| `GetWheelSpeeds()` | 无 | 轮速 rad/s；失败为空 | 顺序与轮关节名一致。 |
| `GetWheelPositions()` | 无 | 轮毂角度 rad；失败或后端不支持时为空 | 顺序与轮关节名一致，保留驱动上报原值，不做单圈归一化。 |
| `GetBaseVelocity()` | 无 | `ChassisBaseVelocity` | 包含 `linear_x/y`、`angular_z`。 |
| `GetBasePose()` | 无 | `ChassisBasePose` | 使用前检查 `valid`。 |
| `GetBatteryCurrents()` | 无 | `std::vector<double>`；失败为空 | 电流（A），顺序见 5.9 节。 |
| `GetBatteryVoltages()` | 无 | `std::vector<double>`；失败为空 | 电压（V），顺序见 5.9 节。 |
| `GetBatterySoc()` | 无 | `std::vector<double>`；失败为空 | 相对 SOC（%），顺序见 5.9 节。 |
| `GetBatteryTemperatures()` | 无 | `std::vector<double>`；失败为空 | 温度（℃），顺序见 5.9 节。 |
| `HasBattery()` | 无 | `std::vector<bool>`；失败为空 | 该舱是否装有电池，`true`=有电池。 |
| `GetBatteryChargingStates()` | 无 | `std::vector<ChargingState>`；失败为空 | `kNotConnected`/`kConnected`/`kCharging`/`kFullyCharged`；无电池时为 `kNotConnected`。 |
| `GetBatteryErrorBits()` | 无 | `std::vector<std::uint32_t>`；失败为空 | 错误位图（uint32），顺序见 5.9 节。 |
| `GetBatteryWarningBits()` | 无 | `std::vector<std::uint32_t>`；失败为空 | 警告位图（uint32），顺序见 5.9 节。 |
| `GetUltrasonicRadarData()` | 无 | `UltrasonicRadarData` | 地址 1 到 8 的最新距离（m）；无有效距离或查询失败为 `-1.0`。 |
| `GetServoState()` | 无 | `ServoState` 数组；失败为空 | 轮关节在前、转向关节在后；数值见 5.3 节。 |
| `GetJointFaultCode()` | 无 | `FaultCode` 数组；失败为空 | 当前故障，数值见 5.4 节。 |
| `GetJointEmcyCode()` | 无 | `FaultCode` 数组；无事件为 `kNone` | 读取后消费事件，数值见 5.4 节。 |
| `GetJointWarningCode()` | 无 | `WarningCode` 数组；失败为空 | 当前警告，数值见 5.5 节。 |

#### 4.2.1 电池分项查询

各接口只读取最新缓存，不发送 CAN 查询或充电控制。
数组顺序统一为服务端电池舱 `battery.id`；例如 `[1]` 时返回数组第 0 项代表舱 2。
未收到报文时返回默认 `0`；单块电池舱超过 8 秒未收到有效 CAN 报文时，
该舱全部电池分项返回零值，表示电池舱断联或未插入。
服务调用失败返回空数组，服务端没有电池 HAL 时返回 `-113002`，SDK 沿用通用服务错误日志。
不同分项查询独立执行，不保证各项对应同一接收时刻。

```cpp
modi_sdk::RobotClient client;
if (!client.Start()) return 1;
auto state = client.Chassis()->State();
const auto currents = state->GetBatteryCurrents();
const auto voltages = state->GetBatteryVoltages();
const auto soc = state->GetBatterySoc();
const auto temperatures = state->GetBatteryTemperatures();
client.Stop();
```

存在 `battery` 配置块即创建并初始化电池 HAL；删除整个配置块即不启用。
`modi_chassis_1` 保留电池配置，`modi_chassis` 没有电池配置。
无需 `enabled` 开关，也没有电池 `timeout_ms` 参数。

```yaml
battery:
  type: "jdbmscan"
  bus: "can1"
  id: [0, 1]
```

| 参数（均必填） | 当前参数文件值 | 作用 |
| --- | --- | --- |
| `type` | `"jdbmscan"` | 选择实现《JKJQ整车&充电CAN协议》电池读取的后端 |
| `bus` | `"can1"` | 电池 SocketCAN 接口，与 `chassis.bus` 独立；驱动不设置波特率 |
| `id` | `[0, 1]` | 返回的电池舱及数组顺序；0 为舱 1，1 为舱 2，非空且不能重复 |

DDS 发布间隔固定为 1000 ms，话题约 1 Hz 发布，不提供 YAML 参数。
协议状态 1 为 300 ms，状态 2 和错误/警告为 1000 ms；重复发布缓存不会增加测量次数。

电池 `type`、`bus`、`id` 三个参数必须在 YAML 中提供；缺项、`battery: {}` 或非法值会报错。
电池改接 `can0` 时只修改 `battery.bus`；
机械臂或底盘关节使用各自的 `bus`。配置在启动时读取，修改后需重启服务端。
运行时从所选 `modi_description` 安装目录加载机型 YAML，修改源码模板后需更新安装资源。

持续读取示例：`ros2 run modi_sdk chassis_example_battery_state chassis_001`。
首个参数为设备端 `system.chassis_id`，请将 `chassis_001` 换成实际值；示例 DDS Domain ID 默认使用 `0`。

#### 4.2.2 超声波雷达距离查询

通过 `client.Chassis()->State()->GetUltrasonicRadarData()` 查询八路最新距离，
调用方式与其他 `ChassisStateApi::Get...()` 接口一致。SDK 内部通过
`/Chassis/State/GetUltrasonicRadarData` 服务读取服务端 HAL 最新缓存，应用层
不需要注册话题、回调或使用生成消息类型。

HAL 使用独立线程逐路发送 Modbus 请求，每路完整回复或超时后等待至少 5 ms
总线静默，再查询下一路。某一路处理完成后立即更新对应缓存，不等待其余地址，
因此 `distance_1()` 到 `distance_8()` 始终是地址 1 到 8 的最新米制距离；
无有效距离时为 `-1.0`。量程由服务端机型配置的 `range_m` 决定，支持 `0.5`、
`1.5`、`2.5`、`3.5`、`5.0 m`。服务查询只复制 HAL 的八路原子缓存，不访问串口，
也不等待当前轮询完成。`UltrasonicRadarData` 提供 `distance_1` 到
`distance_8` 八个 `double` 成员；查询失败时返回的八路距离均为 `-1.0`。

完整调用示例：`example/chassis_example/example_ultrasonic_radar.cpp`。
运行：`ros2 run modi_sdk chassis_example_ultrasonic_radar chassis_001`。

### 4.3 ChassisMotionApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `SpeedTranslate(vx, vy)` | x/y 平移速度，m/s | `0` 成功，负值失败 | 底盘平移。 |
| `SpeedRotate(vx, vz)` | 前向速度 m/s、角速度 rad/s | `0` 成功，负值失败 | 前进并旋转。 |
| `MoveBase(x, y, yaw, linear_x, linear_y, angular_z)` | 目标位姿及三个速度约束 | `0` 成功，负值失败 | 二维位姿运动。 |
| `StopMotion(acceleration=0.5)` | 停止加速度 | `0` 成功，负值失败 | 停止底盘。 |

### 4.4 ChassisManagerApi

回充会话控制：

```cpp
auto manager = client.Chassis()->Manager();
std::string message;
int result = manager->StartDocking(&message);  // 启动回桩、充电和满电待机循环
modi_sdk::ChassisDockingState docking;
result = manager->GetDockingState(&docking);
result = manager->CancelDocking(&message);      // 请求停止回充运动及充电
// result == 0 仅表示关闭请求已接受；继续查询，直到 docking.active == false。

// 运行时启用 0.8 m 安全距离，并读回当前设置。
result = manager->SetSafetyDistance(true, 0.8);
bool safety_enabled = false;
double safety_distance_m = 0.0;
result = manager->GetSafetyDistance(safety_enabled, safety_distance_m);
```

`StartDocking()` 先检查充电器连接：至少一仓有电池且所有在位仓都已连接时，
服务端确认停稳后直接充电，不要求定位或配置桩位；未连接时才要求已有定位和有效
充电桩配置，并依次完成回桩、精对接、接触检查和充电。两条路径接受新任务时均初始化
`charged_once=false`。首次回桩接触检查最多等待
`charging_timeout_s`（默认 10 秒），横向搜索每步停稳后最多检查 3 秒，检测到连接即继续。
未连接则沿用横向触点搜索，达到原有 10 步上限仍未连接才报错关闭。
至少有一仓电池且所有有电池的仓都连接充电器才允许充电。
本轮收到充满反馈后，服务端设置 `charged_once=true`、关闭充电并保持满电待机；
有效参与电池 SOC 低于 `99%` 时再次充电。满电待机仍属于活动会话。
`CancelDocking()` 可在任意阶段请求关闭；服务端停止运动并确认禁止充电后，
`active` 才变为 `false`。充电或满电待机时连接丢失，或触点搜索达到上限仍未连接，
也会进入关闭流程。
SDK 通过回充会话管理充电，不提供直接充电开关接口。

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `StartDocking(message=nullptr)` | 可选的提示输出指针 | `int`：`0` 请求接受，负值为错误码 | 启动新的回充会话。 |
| `CancelDocking(message=nullptr)` | 可选的提示输出指针 | `int`：`0` 请求接受，负值为错误码 | 请求结束回充会话；轮询 `active=false` 确认关闭。 |
| `GetDockingState(state, timeout_ms=5000)` | 状态输出指针、查询超时毫秒数 | `int`：`0` 查询成功，负值为错误码 | 读取回充会话，失败保持输出不变。 |
| `GetDockingStateQuiet(timeout_ms, state)` | 查询超时毫秒数、状态输出指针 | 同上 | 无错误日志的轮询版本。 |
| `SetSafetyDistance(enabled, distance_m)` | 是否启用、有限距离（m） | `int`：`0` 成功，负值为错误码 | 运行时设置安全距离并设置自动安全运动禁止开关；距离必须不小于 `0.01 m`，无上限。 |
| `SetSafetyStopEnabled(enabled)` | `bool` | `int`：`0` 成功，负值为错误码 | 单独启用或关闭自动安全运动禁止；不会触发或清除手动急停。 |
| `GetSafetyDistance(enabled, distance_m)` | 两个输出引用 | `int`：`0` 成功，负值为错误码 | 读取当前运行时设置；失败时不修改输出值。 |
| `PowerOn()` / `PowerOff()` | 无 | `0` 成功，负值失败 | 底盘上下电。 |
| `BrakeRelease()` / `BrakeLock()` | 无 | `0` 成功，负值失败 | 释放或锁定抱闸。 |
| `ClearFaults()` | 无 | `0` 成功，负值失败 | 清除底盘故障。 |
| `GetRobotDomainId()` | 无 | Domain ID；失败 `-1` | 获取配置 Domain ID。 |
| `SetRobotDomainId(domain_id)` | 新 Domain ID | `0` 成功，负值失败 | 修改后需重启底盘节点和 SDK。 |
| `Estop(trigger)` | `true` 触发，`false` 清除 | `0` 成功，负值失败 | 控制底盘急停。 |
| `GetSafetyState()` | 无 | `SafetyState` 整数；失败 `-1` | 获取安全状态机。 |
| `GetChassisState()` | 无 | Init=0、PowerOff=1、PowerOn=2、Running=3；失败 `-1` | 获取底盘状态机。 |

`ChassisDockingState` 包含：

| 字段 | 含义 |
|---|---|
| `state` / `message` | 当前阶段及说明；充电相关阶段为 `charging`、`charged_waiting`、`closing`、`closed`，任务失败为 `failed`。 |
| `active` | 回桩、充电、满电待机和关闭确认期间为 `true`；关闭完成或失败终态为 `false`。 |
| `charged_once` | 本次会话是否充满过，只在接受新会话时重置。 |
| `charging_enabled` | 服务端是否已成功允许本轮充电；实际电流需读取电池状态。 |
| `recharge_soc_threshold` | 满电待机后的再充阈值，单位 %，当前为 `99.0`。 |
| `task_error_code` | 回充任务错误码，独立于查询接口返回码；查询成功也应检查此字段。 |
| `target_distance_m` / `travelled_distance_m` | 精对接目标距离和已行进距离，单位 m。 |
| `safety_override_active` | 回充期间是否使用安全距离覆盖设置。 |

安全距离设置立即生效但不改写 YAML；底盘节点重启后恢复
`system.safety_distance` 中的默认设置。`0.20 m` 是传感器融合分界：不大于
`0.20 m` 的近场距离由超声波判断；大于 `0.20 m` 的范围由激光雷达判断，
用户配置距离可以从 `0.01 m` 开始且无上限（实际有效范围受传感器量程和点云覆盖范围限制）。

安全判断会读取八路超声波中的有效近场测量，并结合激光点云的远场占用栅格；
无效的超声波读数不会被当作障碍。激光安全点云先转换到底盘坐标系，再按
`system.safety_distance.lidar_min_height_m` 和 `lidar_max_height_m` 裁剪高度，
默认范围为 `-0.10～1.50 m`。

## 5. 数据结构与数值含义

本章集中说明 SDK 公共类型中每个数字的含义。关节状态、故障和警告数组按系统配置中的
机器人、臂组、关节顺序排列；底盘 `ServoState`、故障和警告数组按“轮关节在前、
转向关节在后”排列。

### 5.1 PlannerStatus

| 枚举值 | 数值 | 含义 |
|---|---:|---|
| `kError` | `-1` | 规划失败、执行错误或状态查询失败。 |
| `kIdle` | `0` | 规划器空闲，当前没有运动任务。 |
| `kPlanning` | `1` | 正在规划或执行运动。 |
| `kFinished` | `2` | 当前运动任务已完成。 |

### 5.2 ControllerType

| 枚举值 | 数值 | 含义 |
|---|---:|---|
| `kCartesianAdmittance` | `0` | 笛卡尔空间导纳控制。 |
| `kJointAdmittance` | `1` | 关节空间导纳控制。 |
| `kCartesianImpedance` | `2` | 笛卡尔空间阻抗控制。 |
| `kJointImpedance` | `3` | 关节空间阻抗控制。 |

### 5.3 ServoState

`GetServoState()` 返回的每个数字都应转换为 `modi_sdk::ServoState` 后判断。

| 枚举值 | 十六进制 | 十进制 | 含义 |
|---|---:|---:|---|
| `kInit` | `0x00` | `0` | 初始化中，尚未进入可使能状态。 |
| `kDisabled` | `0x01` | `1` | 伺服未使能。 |
| `kReady` | `0x02` | `2` | 已就绪，等待使能。 |
| `kRunning` | `0x03` | `3` | 伺服已使能并处于运行状态。 |
| `kQuickStop` | `0x04` | `4` | 正在执行快速停机。 |
| `kFaultReaction` | `0x06` | `6` | 正在执行故障响应。 |
| `kFault` | `0x07` | `7` | 处于故障状态，需排除并清除故障。 |
| `kInitFault` | `0x08` | `8` | 初始化阶段发生故障。 |
| `kUnknown` | `0xFF` | `255` | 未知、无效或当前后端不支持的状态。 |

### 5.3.1 ChargingState

`GetBatteryChargingStates()` 返回 `modi_sdk::ChargingState` 枚举，直接按枚举判断；DDS 话题和服务按字符串名上报，SDK 负责解析。

| 枚举值 | 数值 | 含义 |
|---|---:|---|
| `kNotConnected` | `0` | 该舱无电池，或充电器未连接。 |
| `kConnected` | `1` | 充电器已连接，尚未检测到充电电流。 |
| `kCharging` | `2` | 充电器已连接，且检测到充电电流（大于 0.1 A）。 |
| `kFullyCharged` | `3` | BMS `0x180` 报告整组充电完成，映射当前参与充电的舱位。 |

### 5.4 FaultCode

`GetJointFaultCode()` 查询设备当前故障；`GetJointEmcyCode()` 读取待处理 EMCY 事件，
成功读取后事件会被消费。未收录的设备故障应保留原始 16 位数值，并结合对应设备手册
解释。

| 枚举值 | 故障码 | 含义 |
|---|---:|---|
| `kNone` | `0x0000` | 无故障。 |
| `kOverCurrent` | `0x2130` | 相电流过流。 |
| `kBusOverVoltage` | `0x3210` | 母线电压过高。 |
| `kBusUnderVoltage` | `0x3220` | 母线电压过低。 |
| `kMotorOverload` | `0x3230` | 电机过载。 |
| `kPhaseLoss` | `0x3331` | 电机动力线缺相。 |
| `kOverTemperature` | `0x4310` | 关节温度过高。 |
| `kUnderTemperature` | `0x4320` | 关节温度过低。 |
| `kParameterStorage` | `0x5520` | 硬件参数存储异常。 |
| `kBrake` | `0x7110` | 抱闸异常。 |
| `kMotorStall` | `0x7121` | 电机堵转。 |
| `kEncoder1` | `0x7303` | 编码器 1 异常。 |
| `kEncoder2` | `0x7304` | 编码器 2 异常。 |
| `kEncoder1Battery` | `0x7385` | 编码器 1 电池电压低。 |
| `kEncoder2Battery` | `0x7386` | 编码器 2 电池电压低。 |
| `kCommunication` | `0x8100` | 通信异常。 |
| `kOverSpeed` | `0x8400` | 关节速度超过限制。 |
| `kFollowingError` | `0x8611` | 位置跟随误差过大。 |
| `kCurrentSensor` | `0xFF00` | 电流传感器异常。 |
| `kEncoderZLoss` | `0xFF01` | 编码器 Z 信号丢失。 |
| `kPeakOverload` | `0xFF02` | 关节峰值电流过载。 |
| `kHardwareShortCircuit` | `0xFF03` | 硬件短路保护。 |
| `kEncoderHall` | `0xFF04` | 编码器 Hall 信号异常。 |
| `kSyncPeriod` | `0xFF05` | 同步周期误差过大。 |
| `kElectricalAngleIdentification` | `0xFF06` | 电角度辨识失败。 |
| `kServoTimeout` | `0xFF07` | 伺服程序运行超时。 |
| `kFirmware` | `0xFF08` | 固件与硬件不匹配。 |
| `kSyncCommand` | `0xFF09` | 同步指令中断或异常。 |

### 5.5 WarningCode

警告通常不立即触发停机，但应记录并根据状态降载或停机检查。

| 枚举值 | 警告码 | 含义 |
|---|---:|---|
| `kNone` | `0x0000` | 无警告。 |
| `kPcbHighTemperature` | `0x2101` | PCB 温度偏高。 |
| `kMotorHighTemperature` | `0x2102` | 电机温度偏高。 |
| `kGearboxHighTemperature` | `0x2103` | 减速器温度偏高。 |
| `kPhaseOverload` | `0x2201` | 相电流负载偏高。 |
| `kHighSpeed` | `0x2301` | 速度接近或超过警告阈值。 |
| `kPositionLimit` | `0x2401` | 位置超过限制值。 |
| `kFlashEraseCount` | `0x2601` | Flash 擦除次数过高。 |
| `kHighVoltage` | `0x2701` | 供电电压偏高。 |
| `kLowVoltage` | `0x2702` | 供电电压偏低。 |

### 5.6 系统状态机

| 数据类型 | 数值 | 含义 |
|---|---:|---|
| `SafetyState` | `0` | Normal：安全状态正常。 |
| `SafetyState` | `1` | Estop：急停已触发。 |
| `SafetyState` | `2` | Fault：安全系统处于故障。 |
| `SafetyState` | `3` | Recovery：安全系统正在恢复。 |
| `RobotState` / `ChassisSystemState` | `0` | Init：系统初始化中。 |
| `RobotState` / `ChassisSystemState` | `1` | PowerOff：系统已下电。 |
| `RobotState` / `ChassisSystemState` | `2` | PowerOn：系统已上电。 |
| `RobotState` / `ChassisSystemState` | `3` | Running：系统正在运行。 |

### 5.7 基础结构和数组顺序

| 类型/字段 | 单位或取值 | 含义 |
|---|---|---|
| `ChassisBaseVelocity::linear_x` | m/s | 底盘 x 方向线速度。 |
| `ChassisBaseVelocity::linear_y` | m/s | 底盘 y 方向线速度。 |
| `ChassisBaseVelocity::angular_z` | rad/s | 底盘绕 z 轴角速度。 |
| `ChassisBasePose::x` / `y` | m | 底盘二维位置。 |
| `ChassisBasePose::yaw` | rad | 底盘偏航角。 |
| `ChassisBasePose::valid` | `true` / `false` | 位姿是否有效；为 `false` 时不要使用 x/y/yaw。 |
| `Pose::data` | `[x,y,z,roll,pitch,yaw]` | 前三项 m，后三项 rad。 |
| `TwistData::data` | `[vx,vy,vz,wx,wy,wz]` | 前三项 m/s，后三项 rad/s。 |
| `WrenchData::data` | `[fx,fy,fz,tx,ty,tz]` | 前三项 N，后三项 N·m。 |
| `BoolArray::data` | `bool[]` | 含义由话题决定，例如限位标志或数字输入。 |
| 机器人关节数组 | `double[]` / 枚举数组 | 按机器人、臂组、关节的系统配置顺序。 |
| 底盘伺服/故障数组 | 枚举数组 | 轮关节在前，转向关节在后。 |
| 末端数组 | 二维数组 | 按 `GetEndEffectorNames()` 顺序。 |

### 5.8 未公开枚举的原始码

`ChassisErrorCode::error_code` 当前公开消息只定义为 `int32`，SDK 和
`modi_system_msgs` 没有发布对应的枚举表。因此该值应视为底盘控制器原始错误码，
不能转换为 `FaultCode`，需要结合所部署底盘控制器/驱动版本的错误码表解释。

### 5.9 电池分项数据

| SDK / HAL 接口 | SDK 返回值 | 含义 |
| --- | --- | --- |
| `GetBatteryCurrents()` | `std::vector<double>` | 电流，A |
| `GetBatteryVoltages()` | `std::vector<double>` | 电压，V |
| `GetBatterySoc()` | `std::vector<double>` | 相对 SOC，% |
| `GetBatteryTemperatures()` | `std::vector<double>` | 温度，℃ |
| `HasBattery()` | `std::vector<bool>` | 该舱是否装有电池，`true`=有电池 |
| `GetBatteryChargingStates()` | `std::vector<ChargingState>` | `kNotConnected`/`kConnected`/`kCharging`/`kFullyCharged`；无电池时为 `kNotConnected` |
| `GetBatteryErrorBits()` | `std::vector<std::uint32_t>` | 错误位图，uint32 |
| `GetBatteryWarningBits()` | `std::vector<std::uint32_t>` | 警告位图，uint32 |

各接口只读取最新缓存，不发送 CAN 查询或充电控制。
数组顺序统一为服务端 `battery.id`；例如 `[1]` 时返回数组第 0 项代表舱 2。
`id=0` 是舱 1，`id=1` 是舱 2，与插入电池的身份无关。收到该舱任意有效
状态 1、状态 2 或错误/警告报文即可刷新计时；超过固定 8 秒时清空整舱数据，
包括 `GetBatteryChargingStates()` 返回 `kNotConnected`、`HasBattery()` 返回
`false`，返回数组长度和顺序保持不变。
重新插入只更新新收字段，不恢复拔出前的旧字段；超时阈值不提供配置项。
未收到报文时返回默认 `0`；单块电池超过 8 秒未收到有效 CAN 报文时，
该块全部电池分项返回零值，表示电池仓断联或未插入。
服务调用失败返回空数组，服务端没有电池 HAL 时返回 `-113002`，SDK 沿用通用服务错误日志。
不同分项查询独立执行，不保证各项对应同一接收时刻。

读取舱 1（ID 0）的 `0x17FF2000～2002` 和舱 2（ID 1）的 `0x18FF2000～2002`。
电流按 U16LE ×0.1−1000 A 换算，相对 SOC 直接使用 BMS Byte 6，不读取绝对 SOC。
`GetBatteryChargingStates()` 返回 `ChargingState` 枚举：`kNotConnected` 表示该舱无电池
或充电器未连接；`kConnected` 表示充电器已连接但尚未检测到充电电流；
`kCharging` 表示已连接且电流超过 0.1 A；`kFullyCharged` 来自 BMS `0x180` 的整组充电完成标志，映射当前参与充电的舱位。
该标志不是各舱独立满电报文，也不由 SOC 达到 100% 或电流降低推断；`kConnected` 不能证明存在充电电流。
`HasBattery()` 只表示该舱是否在 8 秒内收到过有效报文，与 SOC 是否为零无关。
回充会话由服务端统一管理充电使能，至少存在一块电池且所有有电池的仓均连接
充电器才能开启；仅舱 1、仅舱 2 或两仓都有电池时均按这个条件检查。

## 6. 实时话题参考

以下名称是传给 `RegisterSubscriber()`、`Parser::Value()`、
`RegisterPublisher()` 和 `Publisher::Mutable()` 的 SDK 话题名。SDK 会在内部映射为
DDS 实时话题，用户不需要手动添加 `rt/` 前缀。

### 6.1 消息数据结构

消息类型由 `modi_system_msgs` 生成，使用时包含 `modi_type_def.h`。

| 消息类型 | 关键字段 | 数据结构与单位 |
|---|---|---|
| `JointPositions` | `joint_positions()` | `double[]`，关节位置，rad。 |
| `JointVelocities` | `joint_velocities()` | `double[]`，关节速度，rad/s。 |
| `JointAccelerations` | `joint_accelerations()` | `double[]`，关节加速度，rad/s²。 |
| `JointTorques` | `joint_torques()` | `double[]`，关节力矩，N·m。 |
| `JointCurrents` | `joint_currents()` | `double[]`，关节实际转矩电流，A。 |
| `JointVoltages` | `joint_voltages()` | `double[]`，关节母线电压，V。 |
| `JointTemperatures` | `joint_temperatures()` | `double[]`，关节电机温度，℃。 |
| `ServoStateArray` | `values()` | `uint32[]`，数值对应 `modi_sdk::ServoState`。 |
| `JointEmcyCodeArray` | `values()` | `uint32[]`，数值对应 `modi_sdk::FaultCode`。 |
| `BoolArray` | `data()` | `bool[]`，布尔状态数组。 |
| `Pose` | `data()` | `[x,y,z,roll,pitch,yaw]`，m/rad。 |
| `PoseArray` | `poses()` | `Pose[]`，按机器人或末端配置顺序排列。 |
| `TwistData` | `data()` | `[vx,vy,vz,wx,wy,wz]`；速度话题为 m/s、rad/s，加速度话题为 m/s²、rad/s²。 |
| `TwistArray` | `twists()` | `TwistData[]`，按末端配置顺序排列，单位由对应速度/加速度话题决定。 |
| `WrenchData` | `data()` | `[fx,fy,fz,tx,ty,tz]`，N/N·m。 |
| `WrenchArray` | `wrenches()` | `WrenchData[]`，按末端配置顺序排列。 |
| `PlannerStatus` | `status()` | `int32`：Idle=0、Planning=1、Finished=2、Error=-1。 |
| `SafetyState` | `state()` | `int32`：Normal=0、Estop=1、Fault=2、Recovery=3。 |
| `RobotState` | `state()` | `int32`：Init=0、PowerOff=1、PowerOn=2、Running=3。 |
| `DoubleArray` | `data()` | `double[]`，单位由话题决定：角度 rad、力 N、速度 rad/s、电流 A、温度 ℃。 |
| `ServoJointCommand` | `target_joints()`、`period()`、`target_velocity()`、`target_acceleration()` | 关节位置 rad、周期 s、速度 rad/s、加速度 rad/s²。 |
| `ServoLineCommand` | `target_poses()`、`period()` | 末端目标位姿数组和控制周期 s。 |
| `BaseVelocity` | `linear_x()`、`linear_y()`、`angular_z()` | 底盘速度，m/s、rad/s。 |
| `BasePose` | `x()`、`y()`、`yaw()`、`valid()` | 底盘二维位姿，m/rad；`valid` 表示数据有效。 |
| `ChassisErrorCode` | `error_code()` | `int32`，底盘控制器原始错误码；当前公共接口未定义统一枚举，参见 5.8 节。 |
| `ChassisSystemState` | `state()` | `int32`：Init=0、PowerOff=1、PowerOn=2、Running=3。 |

| `BatteryCurrents` | `battery_currents()` | 电流数组，A，顺序为 `battery.id`。 |
| `BatteryVoltages` | `battery_voltages()` | 电压数组，V，顺序为 `battery.id`。 |
| `BatterySoc` | `battery_soc()` | 相对 SOC数组，%，顺序为 `battery.id`。 |
| `BatteryTemperatures` | `battery_temperatures()` | 温度数组，℃，顺序为 `battery.id`。 |
| `BatteryPresence` | `battery_present()` | 该舱是否有电池，bool 数组，顺序为 `battery.id`。 |
| `BatteryChargingStates` | `battery_charging_states()` | 字符串数组，取值 `kNotConnected`/`kConnected`/`kCharging`/`kFullyCharged`，顺序为 `battery.id`。 |
| `BatteryErrorBits` | `battery_error_bits()` | 错误位图数组，uint32，顺序为 `battery.id`。 |
| `BatteryWarningBits` | `battery_warning_bits()` | 警告位图数组，uint32，顺序为 `battery.id`。 |
| `UltrasonicRadarState` | `distance_1()` ... `distance_8()` | 地址 1 到 8 的最新距离，单位 m，无有效距离为 -1.0。 |

### 6.2 机器人实时订阅话题

这些话题可传给 `RobotClient::RegisterSubscriber()`。

| SDK 话题名 | 消息类型 | 数据内容 | 典型用途 |
|---|---|---|---|
| `/Robot/State/JointPositions` | `JointPositions` | `joint_positions: double[]`，rad | 实时关节位置反馈、轨迹监控。 |
| `/Robot/State/JointVelocities` | `JointVelocities` | `joint_velocities: double[]`，rad/s | 速度反馈和运动状态判断。 |
| `/Robot/State/JointAccelerations` | `JointAccelerations` | `joint_accelerations: double[]`，rad/s² | 加速度监控与算法输入。 |
| `/Robot/State/JointTorques` | `JointTorques` | `joint_torques: double[]`，N·m | 力矩监控、碰撞或负载分析。 |
| `/Robot/State/JointCurrents` | `JointCurrents` | `joint_currents: double[]`，A | 驱动电流监控。 |
| `/Robot/State/JointVoltages` | `JointVoltages` | `joint_voltages: double[]`，V | 驱动电压监控。 |
| `/Robot/State/JointTemperatures` | `JointTemperatures` | `joint_temperatures: double[]`，℃ | 关节温度和过热监控。 |
| `/Robot/State/ServoState` | `ServoStateArray` | `values: uint32[]` | 判断关节是否就绪、运行或故障。 |
| `/Robot/State/JointEmcyCode` | `JointEmcyCodeArray` | `values: uint32[]` | 获取关节实时 EMCY 故障事件。 |
| `/Robot/State/JointLimitFlags` | `BoolArray` | `data: bool[]` | 监控关节限位状态。 |
| `/Robot/Io/BoardDi` | `BoolArray` | `data: bool[]` | 读取控制柜数字输入。 |
| `/Robot/Io/ToolDi` | `BoolArray` | `data: bool[]` | 读取工具端数字输入。 |
| `/Robot/State/EndEffectorPoses` | `PoseArray` | `poses: Pose[]` | 获取各末端实时位姿。 |
| `/Robot/State/EndEffectorTwists` | `TwistArray` | `twists: TwistData[]` | 获取各末端实时速度。 |
| `/Robot/State/EndEffectorAccels` | `TwistArray` | `twists: TwistData[]`，m/s²、rad/s² | 获取各末端线/角加速度。 |
| `/Robot/State/EndEffectorWrenches` | `WrenchArray` | `wrenches: WrenchData[]` | 获取各末端六维力反馈。 |
| `/Robot/State/ComPose` | `PoseArray` | `poses: Pose[]` | 获取各机型质心位姿。 |
| `/Robot/State/FbasePose` | `PoseArray` | `poses: Pose[]` | 获取各机型浮动基位姿。 |
| `/Robot/Force/GoalWrenches` | `WrenchArray` | `wrenches: WrenchData[]` | 监控力控目标六维力。 |
| `/Robot/Force/RefPointWrenches` | `WrenchArray` | `wrenches: WrenchData[]` | 监控力控参考点六维力。 |
| `/Robot/Motion/PlannerStatus` | `PlannerStatus` | `status: int32` | 判断规划器空闲、运行、完成或错误。 |
| `/Robot/Manager/SafetyState` | `SafetyState` | `state: int32` | 监控 Normal、急停、故障和恢复状态。 |
| `/Robot/Manager/RobotState` | `RobotState` | `state: int32` | 监控机器人上下电和运行状态。 |

关节数组按系统配置中的机器人、臂组、关节顺序排列；末端数组按
`ModelApi::GetEndEffectorNames()` 的顺序排列。

### 6.3 机器人实时发布话题

这些话题可传给 `RobotClient::RegisterPublisher()`，并在回调中通过
`Publisher::Mutable<MsgT>()` 填充本周期样本。

| SDK 话题名 | 消息类型 | 数据内容 | 典型用途 |
|---|---|---|---|
| `/Robot/Model/SetFbasePose` | `PoseArray` | `poses: Pose[]` | 周期更新各机型浮动基位姿。 |
| `/Robot/Motion/ServoJoint` | `ServoJointCommand` | 目标关节、周期、可选速度/加速度 | 实时关节伺服控制。 |
| `/Robot/Motion/ServoLine` | `ServoLineCommand` | 目标末端位姿数组、周期 | 实时笛卡尔伺服控制。 |

`ServoJointCommand::target_joints` 的长度必须与驱动关节自由度一致；
`ServoLineCommand::target_poses` 的外层顺序必须与末端配置一致。`period` 单位为秒，
应与调用方实际发送周期匹配。

### 6.4 底盘实时订阅话题

当前 SDK 没有注册底盘实时发布话题，仅提供以下实时订阅话题。

| SDK 话题名 | 消息类型 | 数据内容 | 典型用途 |
|---|---|---|---|
| `/Chassis/State/JointPositions` | `JointPositions` | `joint_positions: double[]`，rad | 获取转向关节位置等底盘关节位置。 |
| `/Chassis/State/WheelPositions` | `JointPositions` | `joint_positions: double[]`，rad | 轮毂角度反馈，按轮关节顺序，不做单圈归一化；不支持时为空。 |
| `/Chassis/State/JointVelocities` | `JointVelocities` | `joint_velocities: double[]`，rad/s | 获取轮关节速度等底盘关节速度。 |
| `/Chassis/State/BaseVelocity` | `BaseVelocity` | `linear_x/y`、`angular_z` | 底盘里程计速度和运动监控。 |
| `/Chassis/State/BasePose` | `BasePose` | `x`、`y`、`yaw`、`valid` | 底盘定位位姿；使用前检查 `valid`。 |
| `/Chassis/State/BatteryCurrents` | `BatteryCurrents` | `battery_currents` 数组，A | 接收最新电流缓存。 |
| `/Chassis/State/BatteryVoltages` | `BatteryVoltages` | `battery_voltages` 数组，V | 接收最新电压缓存。 |
| `/Chassis/State/BatterySoc` | `BatterySoc` | `battery_soc` 数组，% | 接收最新相对 SOC缓存。 |
| `/Chassis/State/BatteryTemperatures` | `BatteryTemperatures` | `battery_temperatures` 数组，℃ | 接收最新温度缓存。 |
| `/Chassis/State/BatteryPresence` | `BatteryPresence` | `battery_present` 数组，bool | 该舱是否装有电池。 |
| `/Chassis/State/BatteryChargingStates` | `BatteryChargingStates` | `battery_charging_states` 字符串数组 | `kNotConnected`/`kConnected`/`kCharging`/`kFullyCharged`；无电池时为 `kNotConnected`。 |
| `/Chassis/State/BatteryErrorBits` | `BatteryErrorBits` | `battery_error_bits` 数组，uint32 | 接收最新错误位图缓存。 |
| `/Chassis/State/BatteryWarningBits` | `BatteryWarningBits` | `battery_warning_bits` 数组，uint32 | 接收最新警告位图缓存。 |
| `/Chassis/State/UltrasonicRadar` | `UltrasonicRadarState` | 地址 1 到 8 的最新距离，单位 m | 接收当前八路距离。 |
| `/Chassis/State/ErrorCode` | `ChassisErrorCode` | `error_code: int32` | 监控底盘系统错误码。 |
| `/Chassis/State/ServoState` | `ServoStateArray` | `values: uint32[]` | 监控轮关节和转向关节伺服状态。 |
| `/Chassis/State/JointEmcyCode` | `JointEmcyCodeArray` | `values: uint32[]` | 获取底盘关节 EMCY 故障事件。 |
| `/Chassis/Manager/SafetyState` | `SafetyState` | `state: int32` | 监控底盘安全状态机。 |
| `/Chassis/Manager/ChassisState` | `ChassisSystemState` | `state: int32` | 监控底盘初始化、上下电和运行状态。 |

电池各分项话题与对应查询接口读取同一缓存，固定以 1000 ms 周期发布，
Best Effort、历史深度 1；无电池配置时不注册或发布电池话题。
单块电池超过 8 秒未收到有效 CAN 报文时，该块各分项话题发布零值。

### 6.5 使用注意事项

- 话题注册必须在 `RobotClient::Start()` 前完成。
- SDK 话题名必须以单个 `/` 开头，并严格匹配表格中的大小写。
- `Parser::Value<MsgT>()` 和 `Publisher::Mutable<MsgT>()` 的类型必须与表格一致。
- 回调运行在 SDK 实时线程，应避免终端/文件 I/O、服务调用、长时间持锁和不可控分配。
- 回调与主线程共享数据时，应使用互斥量、无锁队列或其他线程安全机制。
- `GetJointEmcyCode()` 对应的 EMCY 是事件数据；读取后会被消费，不应当作持续状态使用。
- 电池数组按服务端 `battery.id` 排列；单块电池超过 8 秒未收到有效 CAN 报文时，
  该块各分项返回零值，表示电池仓断联或未插入。
