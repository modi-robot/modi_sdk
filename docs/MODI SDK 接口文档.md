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
| `Eef()` | `std::shared_ptr<EefApi>` | 夹爪、灵巧手等硬件末端。 |

### 3.1 ModelApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `GetAllJointDOF()` | 无 | 全部关节自由度；失败为负值 | 包含非驱动关节。 |
| `GetActuatorJointDOF()` | 无 | 驱动关节自由度；失败为负值 | 运动目标长度应与此一致。 |
| `GetAllJointNames()` | 无 | 全部关节名；失败为空 | 顺序与系统配置一致。 |
| `GetActuatorJointNames()` | 无 | 驱动关节名；失败为空 | 与状态和目标数组顺序一致。 |
| `GetActuatorJointLimits(lower, upper)` | `lower`、`upper`：输出上下限 | `0` 成功，负值失败 | 单位 rad。 |
| `GetEndEffectorNames()` | 无 | 末端坐标系名称；失败为空 | 外层末端数组顺序依据。 |
| `GetRobotNames()` | 无 | 机器人名称；失败为空 | 与配置顺序一致。 |
| `AddToolModel(tool_name, ee_name, frame_xyzrpy, mass, com, inertia)` | 工具名、末端名、位姿、质量 kg、质心 m、惯量 | `0` 成功，负值失败 | 添加工具与负载模型。 |
| `RemoveToolModel(tool_name)` | `tool_name`：工具名 | `0` 成功，负值失败 | 删除工具模型。 |
| `GetTcpOffset(name, pose)` | `name`：工具/末端名；`pose`：输出位姿 | `0` 成功，负值失败 | 获取 TCP 偏移。 |
| `GetPayload(name, mass, com, inertia)` | `name`：工具名；其余为输出 | `0` 成功，负值失败 | 获取负载参数。 |
| `GetControlPeriod()` | 无 | 控制周期，单位 s；失败时非正值 | 获取系统控制周期。 |
| `LoadCollisionPairs(path)` | `path`：碰撞对配置文件 | `0` 成功，负值失败 | 加载自碰撞检测对。 |
| `RemoveAllCollisionPairs()` | 无 | `0` 成功，负值失败 | 清空碰撞检测对。 |
| `GetFbasePosition()` | 无 | 每个机型的基座位姿；失败为空 | 外层顺序与 `robot_name` 一致。 |
| `SetInstallationPosition(poses)` | `poses`：每机型一个六维位姿 | `0` 成功，负值失败 | 设置机器人安装位姿。 |

### 3.2 StateApi

以下关节数组按机器人、臂组、关节的系统配置顺序排列。

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
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
| `StopMotion(acceleration=2.0)` | 停止加速度 | `0` 成功，负值失败 | 停止当前运动。 |
| `GetPlannerStatus()` | 无 | `PlannerStatus` 整数值；失败 `-1` | 获取规划器状态。 |

### 3.4 ForceApi

二维参数外层顺序与 `GetEndEffectorNames()` 一致。

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `FcEnable(controller_type=0)` | `ControllerType` 对应整数 | `0` 成功，负值失败 | 启用力控。 |
| `FcDisable()` | 无 | `0` 成功，负值失败 | 禁用力控。 |
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

数组按末端执行器驱动器顺序排列。

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `GetActuatorCount()` | 无 | 驱动器数量；失败为负值 | 获取末端驱动器数量。 |
| `GetActuatorRadians()` / `GetFingerRadians()` | 无 | 角度数组 rad；失败为空 | 获取驱动器/手指角度。 |
| `GetActuatorForce()` | 无 | 实际力数组，N；失败为空 | 获取末端驱动器力反馈。 |
| `GetActuatorCurrent()` | 无 | 实际电流数组，A；失败为空 | 获取末端驱动器电流反馈。 |
| `GetActuatorTemperature()` | 无 | 温度数组，℃；失败为空 | 获取末端驱动器温度。 |
| `GetActuatorFaultCode()` | 无 | `int16_t` 原始故障码数组；失败为空 | 含义取决于末端型号，见 5.8 节。 |
| `GetActuatorStatus()` | 无 | `int16_t` 状态码数组；失败为空 | 标准状态值见 5.8 节。 |
| `SetActuatorAngle(angle)` | 每驱动器目标角度，rad | `0` 成功，负值失败 | 设置角度。 |
| `SetActuatorForce(force)` | 每驱动器目标力，N | `0` 成功，负值失败 | 设置力。 |
| `SetActuatorSpeed(speed)` | 每驱动器目标角速度，rad/s | `0` 成功，负值失败 | 设置速度。 |
| `SetMotionControlEnable(enable)` | 是否启用运动控制 | `0` 成功，负值失败 | 控制末端使能。 |
| `ClearFault()` | 无 | `0` 成功，负值失败 | 清除末端故障。 |
| `EmergencyStop(enable)` | `true` 触发，`false` 清除 | `0` 成功，负值失败 | 控制末端急停。 |

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
| `GetBaseVelocity()` | 无 | `ChassisBaseVelocity` | 包含 `linear_x/y`、`angular_z`。 |
| `GetBasePose()` | 无 | `ChassisBasePose` | 使用前检查 `valid`。 |
| `GetServoState()` | 无 | `ServoState` 数组；失败为空 | 轮关节在前、转向关节在后；数值见 5.3 节。 |
| `GetJointFaultCode()` | 无 | `FaultCode` 数组；失败为空 | 当前故障，数值见 5.4 节。 |
| `GetJointEmcyCode()` | 无 | `FaultCode` 数组；无事件为 `kNone` | 读取后消费事件，数值见 5.4 节。 |
| `GetJointWarningCode()` | 无 | `WarningCode` 数组；失败为空 | 当前警告，数值见 5.5 节。 |

### 4.3 ChassisMotionApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `SpeedTranslate(vx, vy)` | x/y 平移速度，m/s | `0` 成功，负值失败 | 底盘平移。 |
| `SpeedRotate(vx, vz)` | 前向速度 m/s、角速度 rad/s | `0` 成功，负值失败 | 前进并旋转。 |
| `MoveBase(x, y, yaw, linear_x, linear_y, angular_z)` | 目标位姿及三个速度约束 | `0` 成功，负值失败 | 二维位姿运动。 |
| `StopMotion(acceleration=0.5)` | 停止加速度 | `0` 成功，负值失败 | 停止底盘。 |

### 4.4 ChassisManagerApi

| 接口 | 参数说明 | 返回值 | 说明 |
|---|---|---|---|
| `PowerOn()` / `PowerOff()` | 无 | `0` 成功，负值失败 | 底盘上下电。 |
| `BrakeRelease()` / `BrakeLock()` | 无 | `0` 成功，负值失败 | 释放或锁定抱闸。 |
| `ClearFaults()` | 无 | `0` 成功，负值失败 | 清除底盘故障。 |
| `GetRobotDomainId()` | 无 | Domain ID；失败 `-1` | 获取配置 Domain ID。 |
| `SetRobotDomainId(domain_id)` | 新 Domain ID | `0` 成功，负值失败 | 修改后需重启底盘节点和 SDK。 |
| `Estop(trigger)` | `true` 触发，`false` 清除 | `0` 成功，负值失败 | 控制底盘急停。 |
| `GetSafetyState()` | 无 | `SafetyState` 整数；失败 `-1` | 获取安全状态机。 |
| `GetChassisState()` | 无 | Init=0、PowerOff=1、PowerOn=2、Running=3；失败 `-1` | 获取底盘状态机。 |

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

`EefApi::GetActuatorFaultCode()` 返回末端设备通道原始故障码。当前公共 SDK 没有为所有
末端型号统一故障枚举，必须查询对应末端设备手册，不能套用关节 `FaultCode`。

当前标准末端 HAL 对 `GetActuatorStatus()` 使用以下状态值；若部署其他末端型号，应以
该型号设备协议为准，且不能套用关节 `ServoState`。

| 末端状态值 | 含义 |
|---:|---|
| `0` | Releasing：正在松开。 |
| `1` | Grasping：正在抓取。 |
| `2` | PositionReached：位置到位停止。 |
| `3` | ForceReached：力控到位停止。 |
| `5` | OverCurrent：电流保护停止。 |
| `6` | Stall：电缸堵转停止。 |
| `7` | Fault：电缸故障停止。 |

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
| `/Robot/Eef/ActuatorRadians` | `DoubleArray` | `data: double[]`，rad | 获取末端驱动器角度。 |
| `/Robot/Eef/FingerRadians` | `DoubleArray` | `data: double[]`，rad | 获取手指关节角度。 |
| `/Robot/Eef/ActuatorForce` | `DoubleArray` | `data: double[]`，N | 获取末端驱动器力。 |
| `/Robot/Eef/ActuatorCurrent` | `DoubleArray` | `data: double[]`，A | 获取末端驱动器电流。 |
| `/Robot/Eef/ActuatorTemperature` | `DoubleArray` | `data: double[]`，℃ | 获取末端驱动器温度。 |

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
| `/Robot/Eef/SetActuatorAngle` | `DoubleArray` | `data: double[]`，rad | 周期设置末端驱动器角度。 |
| `/Robot/Eef/SetActuatorForce` | `DoubleArray` | `data: double[]`，N | 周期设置末端驱动器力。 |
| `/Robot/Eef/SetActuatorSpeed` | `DoubleArray` | `data: double[]`，rad/s | 周期设置末端驱动器速度。 |

`ServoJointCommand::target_joints` 的长度必须与驱动关节自由度一致；
`ServoLineCommand::target_poses` 的外层顺序必须与末端配置一致。`period` 单位为秒，
应与调用方实际发送周期匹配。

### 6.4 底盘实时订阅话题

当前 SDK 没有注册底盘实时发布话题，仅提供以下实时订阅话题。

| SDK 话题名 | 消息类型 | 数据内容 | 典型用途 |
|---|---|---|---|
| `/Chassis/State/JointPositions` | `JointPositions` | `joint_positions: double[]`，rad | 获取转向关节位置等底盘关节位置。 |
| `/Chassis/State/JointVelocities` | `JointVelocities` | `joint_velocities: double[]`，rad/s | 获取轮关节速度等底盘关节速度。 |
| `/Chassis/State/BaseVelocity` | `BaseVelocity` | `linear_x/y`、`angular_z` | 底盘里程计速度和运动监控。 |
| `/Chassis/State/BasePose` | `BasePose` | `x`、`y`、`yaw`、`valid` | 底盘定位位姿；使用前检查 `valid`。 |
| `/Chassis/State/ErrorCode` | `ChassisErrorCode` | `error_code: int32` | 监控底盘系统错误码。 |
| `/Chassis/State/ServoState` | `ServoStateArray` | `values: uint32[]` | 监控轮关节和转向关节伺服状态。 |
| `/Chassis/State/JointEmcyCode` | `JointEmcyCodeArray` | `values: uint32[]` | 获取底盘关节 EMCY 故障事件。 |
| `/Chassis/Manager/SafetyState` | `SafetyState` | `state: int32` | 监控底盘安全状态机。 |
| `/Chassis/Manager/ChassisState` | `ChassisSystemState` | `state: int32` | 监控底盘初始化、上下电和运行状态。 |

### 6.5 订阅示例

```cpp
#include "modi_sdk/robot_client.hpp"
#include "modi_type_def.h"

modi_sdk::RobotClient client;

const int result = client.RegisterSubscriber(
    {"/Robot/State/JointPositions", "/Robot/Motion/PlannerStatus"},
    [](const modi_sdk::Parser& parser) {
      const auto joints = parser.Value<JointPositions>(
          "/Robot/State/JointPositions");
      const auto planner = parser.Value<PlannerStatus>(
          "/Robot/Motion/PlannerStatus");

      // 回调运行在实时线程，只复制必要数据，不执行阻塞操作。
      (void)joints;
      (void)planner;
    });

if (result != 0 || !client.Start()) {
  return 1;
}
```

### 6.6 使用注意事项

- 话题注册必须在 `RobotClient::Start()` 前完成。
- SDK 话题名必须以单个 `/` 开头，并严格匹配表格中的大小写。
- `Parser::Value<MsgT>()` 和 `Publisher::Mutable<MsgT>()` 的类型必须与表格一致。
- 回调运行在 SDK 实时线程，应避免终端/文件 I/O、服务调用、长时间持锁和不可控分配。
- 回调与主线程共享数据时，应使用互斥量、无锁队列或其他线程安全机制。
- `GetJointEmcyCode()` 对应的 EMCY 是事件数据；读取后会被消费，不应当作持续状态使用。
