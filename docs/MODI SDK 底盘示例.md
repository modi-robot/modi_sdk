# 底盘 SDK 示例

本文说明 `chassis_example_*` 示例的运行方式和命令行参数含义。运动、建图和导航示例
都会控制真实底盘，运行前确认底盘周围无人、急停可用、地图和定位状态符合预期。

所有底盘示例的第一个命令行参数均为 `<chassis_id>`，填写设备端 `system.chassis_id`
的值。下文以 `chassis_001` 为例，请替换为实际 ID。DDS Domain ID 默认使用 `0`，
与常规机械臂示例一致；使用其他域时调整示例中 `RobotClient::Config::domain_id`。

运行前先按 SDK 使用说明完成编译，然后进入 SDK 发布包根目录并加载环境：

```bash
cd modi_sdk_0.0.4_linux_arm64_728be8e
source ./setup.bash
```

以下命令均在该目录中执行。编译生成的示例位于 `./build/example/`；如果直接使用发布包
自带的已编译示例，将命令中的 `./build/example/` 替换为 `./lib/modi_sdk/`。

## 1. 运行前权限

SDK 客户端会尝试使用实时线程。若普通用户直接运行时报实时调度权限不足，可以先给
示例程序临时授予 `CAP_SYS_NICE` 能力：

```bash
find ./build/example -maxdepth 1 -type f \
  -name 'chassis_example_*' \
  -exec sudo setcap cap_sys_nice+ep {} \;
```

如果此前曾使用 `sudo` 运行示例，日志目录可能属于 root，普通用户运行时会提示
`Failed opening file .../log/modi.log for writing: Permission denied`。执行：

```bash
sudo chown -R "$USER":"$(id -gn)" ./log
```

该权限只作用于当前这些可执行文件；重新编译或更换目录后需要重新设置。需要取消时：

```bash
find ./build/example -maxdepth 1 -type f \
  -name 'chassis_example_*' \
  -exec sudo setcap -r {} \;
```

## 2. 基础读取示例

读取底盘模型参数：

```bash
./build/example/chassis_example_robot_model chassis_001
```

输出内容包括转向关节名、轮组数量、轮径、控制周期、最大速度、最大加速度和轮组安装位姿。

读取底盘状态：

```bash
./build/example/chassis_example_robot_state chassis_001
```

`chassis_id` 后可选传入 `x y yaw` 重置当前基座位姿：

```bash
./build/example/chassis_example_robot_state chassis_001 0.0 0.0 0.0
```

参数含义：

| 参数    | 单位 | 含义                      |
| ------- | ---- | ------------------------- |
| `chassis_id` | — | 设备端 `system.chassis_id` 的值 |
| `x`   | m    | 地图坐标系下的基座 X 坐标 |
| `y`   | m    | 地图坐标系下的基座 Y 坐标 |
| `yaw` | rad  | 地图坐标系下的基座航向角  |

`chassis_example_robot_state` 会在每轮查询后等待 500 ms，输出底盘速度、位姿、导航状态、
舵轮位置、轮速、关节电流/电压/温度、电池状态、伺服状态和故障码。关节电流、电压、
温度数组的顺序均为“轮关节 → 转向关节”；电池数据通过独立接口读取。按 `Ctrl-C` 退出。

### 2.1 电池信息读取

电池由独立的 `battery_hal/jdbmscan_hal` 实现，继承统一 `BatteryHal`，
使用《JKJQ整车&充电CAN协议》，与关节 CANopen/CanEvo 接口分开。

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

DDS 发布间隔固定为 `1000 ms`，约 1 Hz；协议状态 1 为 300 ms，状态 2 和错误/警告为 1000 ms。
话题可重复发送相同缓存，主动调用分项查询接口不受话题周期限制。

电池 `type`、`bus`、`id` 三个参数必须在 YAML 中提供；缺项、`battery: {}` 或非法值会报错。
电池改接 `can0` 时只修改 `battery.bus`；
机械臂或底盘关节使用各自的 `bus`。配置在启动时读取，修改后需重启服务端。
运行时从所选 `modi_description` 安装目录加载机型 YAML，修改源码模板后需更新安装资源。

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
`id` 是固定舱位编号，与插入电池身份无关；拔出电池后数组长度和舱位顺序不变。
收到该舱任意有效状态 1、状态 2 或错误/警告报文即可刷新该舱 8 秒计时。
未收到报文时返回默认 `0`；单块电池超过 8 秒未收到有效 CAN 报文时，
该块全部分项返回零值、`HasBattery()` 返回 `false`、
`GetBatteryChargingStates()` 返回 `kNotConnected`，表示电池仓断联或未插入。
服务调用失败返回空数组，服务端没有电池 HAL 时返回 `-113002`，SDK 沿用通用服务错误日志。
不同分项查询独立执行，不保证各项对应同一接收时刻。

`kFullyCharged` 来自 BMS `0x180` 的整组充电完成标志，映射当前参与充电的舱位；它不是各舱独立满电报文，也不由 SOC 达到 100% 或电流降低推断。

```cpp
modi_sdk::RobotClient::Config config;
config.target_robot_id = argv[1];  // 第一个命令行参数：chassis_id
modi_sdk::RobotClient client(config);
if (!client.Start()) return 1;
auto state = client.Chassis()->State();
const auto currents = state->GetBatteryCurrents();
const auto voltages = state->GetBatteryVoltages();
const auto soc = state->GetBatterySoc();
const auto temperatures = state->GetBatteryTemperatures();
const auto present = state->HasBattery();                 // 该舱是否有电池
const auto charging_states = state->GetBatteryChargingStates();  // 未连接/已连接/充电中/已充满
client.Stop();
```

示例源码：`example/chassis_example/example_battery_state.cpp`。
保留原可执行名，内部依次调用上述分项接口，每轮查询后等待 1000 ms，Ctrl-C 退出：

```bash
ros2 run modi_sdk chassis_example_battery_state chassis_001
```

首参传入 `chassis_id`，DDS Domain ID 默认使用 `0`；使用其他域时，
调整示例中的 `RobotClient::Config::domain_id`。
持续订阅使用各分项话题 `/Chassis/State/BatteryCurrents`、`BatteryVoltages`、
`BatterySoc`、`BatteryTemperatures`、`BatteryPresence`、`BatteryChargingStates`、`BatteryErrorBits`、`BatteryWarningBits`。
Scope 已订阅这些数组并通过网关 JSON 输出到前端；前端电池状态直接使用上述接口结果，不再按电流或电压自行判断。

### 2.2 键盘控制回充会话

示例源码 `example/chassis_example/example_docking_session.cpp`：

```bash
ros2 run modi_sdk chassis_example_docking_session chassis_001
```

| 按键 | 操作 |
| --- | --- |
| `s` | `Manager()->StartDocking()`，一键回充 |
| `x` | `Manager()->CancelDocking()`，请求结束充电 |
| `p` | `Manager()->GetDockingState()`，显示会话阶段、活动状态及任务错误 |
| `q` / Esc / Ctrl-C | 本示例发出过启动请求时，请求关闭并最多等待 10 秒确认后退出 |

通过第一个命令行参数传入 `chassis_id`，DDS Domain ID 默认使用 `0`，按键无需回车。
至少一仓有电池且所有在位仓均已连接充电器时，按 `s` 将在确认停稳后
直接充电；未连接时需完成定位并配置有效充电桩，再执行回桩、精对接及充电控制。
SDK 不提供直接充电开关；`StartDocking()` 返回 `0` 表示新会话接受。
首次接触检查最多等待 `charging_timeout_s`（默认 10 秒），横向搜索每步停稳后最多检查
3 秒，检测到连接即继续；达到原有 10 步上限仍未连接才报错关闭。
充满后进入 `charged_waiting`，`active=true`，服务端保持关闭，
有效 SOC 低于 `99%` 后自动再充；`charged_once` 保留本会话充满过的记录。
`CancelDocking()` 返回 `0` 仅表示请求接受；`closing` 期间仍可能 `active=true`，
直到充电关闭确认成功才变成 `closed` 或 `failed`、`active=false`。
示例退出时会轮询确认关闭，并报告查询超时；任务故障使用 `task_error_code` 读取，
它独立于 `GetDockingState()` 的查询返回码。客户端退出不会代替服务端的关闭确认。
设备配置见[电池 HAL 说明](../../../modi_chassis_server/modi_chassis_hardware/battery_hal/docs/电池HAL说明.md)。

### 2.3 超声波雷达读取

```bash
ros2 run modi_sdk chassis_example_ultrasonic_radar chassis_001
```

SDK 发布包中也可运行：

```bash
./build/example/chassis_example_ultrasonic_radar chassis_001
```

示例源码：`example/chassis_example/example_ultrasonic_radar.cpp`。
调用方式与其他底盘状态示例一致：通过 `client.Chassis()->State()` 获取状态接口，
循环调用 `GetUltrasonicRadarData()` 并打印结果。示例不注册 DDS 话题、不使用
`Parser`，每 200 ms 查询一次，按 `Ctrl-C` 退出。

服务端必须启用 `ultrasonic_radar`；SDK 客户端不直接访问 `/dev/ttyS1`。
示例打印地址 1 到 8 的最新米制距离；无有效距离时显示 `-1.000`。
服务端逐路等待完整回复，回复或失败后总线静默至少 5 ms，再读取下一路；
某一路更新后不需要等待整轮完成。查询只读取服务端最新缓存，不阻塞串口轮询。

## 3. 设备管理示例

查看 Domain ID，并依次执行清故障、上电、松抱闸、抱闸、下电：

```bash
./build/example/chassis_example_manager chassis_001
```

触发软件急停，使机器人进入紧急停止状态：

```bash
./build/example/chassis_example_manager chassis_001 estop-on
```

解除软件急停：

```bash
./build/example/chassis_example_manager chassis_001 estop-off
```

`estop-off` 仅发送软件急停解除请求，不能代替实体急停按钮复位。若实体急停按钮仍被
按下，需要先人工旋转或拉起急停按钮，再按机器人操作流程清除故障并重新上电。

设置机器人端 DDS Domain ID：

```bash
./build/example/chassis_example_manager chassis_001 set-domain 0
```

参数含义：

| 参数                | 含义                                    |
| ------------------- | --------------------------------------- |
| `estop-on`        | 触发软件急停，使机器人进入紧急停止状态  |
| `estop-off`       | 请求解除软件急停，不会复位实体急停按钮  |
| `set-domain <id>` | 将机器人端 DDS Domain ID 设置为`<id>` |

`set-domain` 后 SDK 端和机器人端必须使用相同 Domain ID 才能通信。默认 SDK 配置使用
`0`。

## 4. 速度控制示例

综合速度控制示例：

```bash
./build/example/chassis_example_motion_control chassis_001
```

该示例会清故障、上电，设置较低的速度和加速度限制，然后依次执行：

1. `SpeedTranslate(0.2, 0.0)`：向前平移 3 秒。
2. `SpeedAckermann(0.15, 0.3)`：阿克曼模式前进并转向 3 秒。
3. `SpeedHolonomic(0.15, 0.1, 0.2)`：全向模式运动 3 秒。
4. `StopMotion()` 并下电。

单项低速测试：

```bash
./build/example/chassis_example_speed_translate chassis_001
./build/example/chassis_example_speed_ackermann chassis_001
./build/example/chassis_example_speed_holonomic chassis_001
```

这些示例只接收 `chassis_id`，速度值写在源码中：

| 示例                                | 内部命令                           | 参数含义                                   |
| ----------------------------------- | ---------------------------------- | ------------------------------------------ |
| `chassis_example_speed_translate` | `SpeedTranslate(0.2, 0.0)`       | `vx`、`vy`，单位 m/s                   |
| `chassis_example_speed_ackermann` | `SpeedAckermann(0.1, 0.2)`       | `vx` 单位 m/s，`wz` 单位 rad/s         |
| `chassis_example_speed_holonomic` | `SpeedHolonomic(0.1, 0.1, 0.15)` | `vx`、`vy` 单位 m/s，`wz` 单位 rad/s |

## 5. 键盘控制综合示例

```bash
./build/example/chassis_example_keyboard_control chassis_001
```

该示例会清故障并上电，支持平移模式和阿克曼模式。运行时终端会原位刷新中英双语按键
说明、当前控制模式、目标控制速度、底盘实际速度、轮毂实际速度和轮毂实际转向角。

按键功能：

| 按键（Key）                   | 功能（Function）                                                                                                     |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `m`                         | 切换平移/阿克曼模式（switch translate/Ackermann mode）                                                               |
| `w` / `s`                 | 前进/后退（forward/backward）                                                                                        |
| `a` / `d`                 | 平移模式下左移/右移，阿克曼模式下左转/右转（strafe left/right in translate mode, turn left/right in Ackermann mode） |
| `q` / `e` / `z` / `c` | 分别表示左前、右前、左后、右后（front-left, front-right, rear-left, rear-right）                                     |
| `+` / `-`                 | 按每次 10% 提高/降低速度倍率，范围为 10% 到 100%，默认 60%（adjust speed scale by 10%, default 60%）                 |
| `space` 或 `x`            | 停止（stop）                                                                                                         |
| `Esc` 或 `Ctrl-C`         | 退出示例，退出前停止运动并下电（quit after stopping motion and powering off）                                        |

`q/e/z/c` 分别对应左前、右前、左后、右后。平移模式下这些按键用于斜向移动；
阿克曼模式下用于前进/后退同时转弯。

速度倍率默认是底盘当前配置速度上限的 60%，可使用 `+` 或 `=` 提高 10%，使用 `-` 降低
10%。速度倍率限制在 10% 到 100% 之间，并实时显示在终端界面中。该倍率只影响本键盘示例
发送的 `SpeedTranslate()` 和 `SpeedAckermann()` 目标速度，不会修改底盘模型中的速度配置。

速度含义：

| 模式       | SDK 调用                   | 参数含义                                                     |
| ---------- | -------------------------- | ------------------------------------------------------------ |
| 平移模式   | `SpeedTranslate(vx, vy)` | `vx` 为前后速度，`vy` 为左右速度，单位 m/s               |
| 阿克曼模式 | `SpeedAckermann(vx, wz)` | `vx` 为前后速度，单位 m/s；`wz` 为转向角速度，单位 rad/s |

键盘控制示例读取交互式终端的标准输入，适合通过 SSH 远程运行，不依赖本地或远程
机器上的 `/dev/input/event*` 设备。方向键 `w/s/a/d` 每次输入会维持约 150 ms；
斜向键 `q/e/z/c` 每次输入也会维持约 150 ms；需要持续按住或重复输入方向键，
按 `space` 或 `x` 可立即锁存停止，按 `Esc` 或 `Ctrl-C` 退出。
终端输入没有物理键的松开事件，因此不要直接把程序输入重定向到普通文件或管道。

如果启动时报 `ClearFaults: fault persists`，表示底盘仍存在无法直接清除的故障。此时示例
不会进入键盘控制循环，需要先检查实体急停是否复位、底盘是否正常上电、轮毂/转向伺服
是否有故障，并查看 `safety`、`servo`、`fault_code`、`emcy_code` 的输出。常见状态含义：

| 状态         | 含义                                       |
| ------------ | ------------------------------------------ |
| `safety=0` | 安全状态正常                               |
| `safety=1` | 急停触发                                   |
| `safety=2` | 安全系统故障                               |
| `servo=7`  | 对应伺服处于故障状态，需要排除故障后再清除 |

## 6. SLAM 与导航示例

启动建图：

```bash
./build/example/chassis_example_slam chassis_001 mapping 一楼大厅
```

建图临时目录由远端 System 管理；按 `Ctrl-C` 后示例调用 `SaveMapping(map_name)`，
同步保存为指定名称的正式地图。

启动定位：

```bash
./build/example/chassis_example_slam chassis_001 localization 一楼大厅
```

建图时按 `Ctrl-C` 会同步保存地图；定位时按 `Ctrl-C` 会切回空闲模式。参数含义：

| 参数             | 含义                                                         |
| ---------------- | ------------------------------------------------------------ |
| `chassis_id` | 设备端 `system.chassis_id` 的值 |
| `mapping`      | 启动建图，退出时按名称保存地图 |
| `localization` | 按名称加载地图后启动定位      |
| `map_name`     | 由远端 System 管理的地图名称                               |

在已加载地图且定位有效时，执行一次全向导航：

```bash
./build/example/chassis_example_navigation chassis_001
```

该示例内部目标点为 `{x=0.5, y=0.0, yaw=0.0}`，单位分别为 m、m、rad。

加载地图、等待定位有效后再导航：

```bash
./build/example/chassis_example_localization_navigation chassis_001 一楼大厅
```

除首参 `chassis_id` 外，示例接收地图名称，由远端 System 解析；
不读取 SDK 本机地图目录，也不接受任意绝对路径。目标点仍是源码常量：

| 项目        | 当前值              | 含义                          |
| ----------- | ------------------- | ----------------------------- |
| 第二个参数  | `一楼大厅`        | 工作区`maps` 下的地图目录名 |
| `kTarget` | `{0.0, 0.0, 0.0}` | 导航目标`x`、`y`、`yaw` |

需要修改导航目标时调整源码中的 `kTarget` 并重新编译。

## 7. 运行顺序建议

首次联调建议按以下顺序执行：

1. `chassis_example_robot_model`：确认能连上机器人并读取模型。
2. `chassis_example_robot_state`：确认状态、位姿和故障码正常。
3. `chassis_example_manager`：清故障并确认上电、下电流程正常。
4. `chassis_example_speed_translate`：低速单项运动测试。
5. `chassis_example_keyboard_control`：键盘低速联调平移和阿克曼运动。
6. `chassis_example_slam <chassis_id> mapping <map_name>`：建图。
7. `chassis_example_slam <chassis_id> localization <map_name>`：定位。
8. `chassis_example_navigation`：在地图和定位有效后再运行。

若接口返回负值，先检查机器人端服务是否启动、DDS Domain ID 是否一致、网络是否互通、
急停和故障状态是否正常。
