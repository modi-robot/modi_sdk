# 底盘 SDK 示例

本文说明 `chassis_example_*` 示例的运行方式和命令行参数含义。运动、建图和导航示例
都会控制真实底盘，运行前确认底盘周围无人、急停可用、地图和定位状态符合预期。

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
./build/example/chassis_example_robot_model
```

输出内容包括转向关节名、轮组数量、轮径、控制周期、最大速度、最大加速度和轮组安装位姿。

读取底盘状态：

```bash
./build/example/chassis_example_robot_state
```

可选传入 `x y yaw` 重置当前基座位姿：

```bash
./build/example/chassis_example_robot_state 0.0 0.0 0.0
```

参数含义：

| 参数    | 单位 | 含义                      |
| ------- | ---- | ------------------------- |
| `x`   | m    | 地图坐标系下的基座 X 坐标 |
| `y`   | m    | 地图坐标系下的基座 Y 坐标 |
| `yaw` | rad  | 地图坐标系下的基座航向角  |

`chassis_example_robot_state` 会每 500 ms 输出一次底盘速度、位姿、导航状态、舵轮位置、
轮速、伺服状态和故障码。按 `Ctrl-C` 退出。

## 3. 设备管理示例

查看 Domain ID，并依次执行清故障、上电、松抱闸、抱闸、下电：

```bash
./build/example/chassis_example_manager
```

触发软件急停，使机器人进入紧急停止状态：

```bash
./build/example/chassis_example_manager estop-on
```

解除软件急停：

```bash
./build/example/chassis_example_manager estop-off
```

`estop-off` 仅发送软件急停解除请求，不能代替实体急停按钮复位。若实体急停按钮仍被
按下，需要先人工旋转或拉起急停按钮，再按机器人操作流程清除故障并重新上电。

设置机器人端 DDS Domain ID：

```bash
./build/example/chassis_example_manager set-domain 0
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
./build/example/chassis_example_motion_control
```

该示例会清故障、上电，设置较低的速度和加速度限制，然后依次执行：

1. `SpeedTranslate(0.2, 0.0)`：向前平移 3 秒。
2. `SpeedAckermann(0.15, 0.3)`：阿克曼模式前进并转向 3 秒。
3. `SpeedHolonomic(0.15, 0.1, 0.2)`：全向模式运动 3 秒。
4. `StopMotion()` 并下电。

单项低速测试：

```bash
./build/example/chassis_example_speed_translate
./build/example/chassis_example_speed_ackermann
./build/example/chassis_example_speed_holonomic
```

这些示例内部没有命令行参数，速度值写在源码中：

| 示例                                | 内部命令                           | 参数含义                                   |
| ----------------------------------- | ---------------------------------- | ------------------------------------------ |
| `chassis_example_speed_translate` | `SpeedTranslate(0.2, 0.0)`       | `vx`、`vy`，单位 m/s                   |
| `chassis_example_speed_ackermann` | `SpeedAckermann(0.1, 0.2)`       | `vx` 单位 m/s，`wz` 单位 rad/s         |
| `chassis_example_speed_holonomic` | `SpeedHolonomic(0.1, 0.1, 0.15)` | `vx`、`vy` 单位 m/s，`wz` 单位 rad/s |

## 5. 键盘控制综合示例

```bash
./build/example/chassis_example_keyboard_control
```

该示例会清故障并上电，支持平移模式和阿克曼模式。运行时终端会原位刷新中英双语按键
说明、当前控制模式、目标控制速度、底盘实际速度、轮毂实际速度和轮毂实际转向角。

按键功能：

| 按键（Key）        | 功能（Function）                                                                                                     |
| ------------------ | -------------------------------------------------------------------------------------------------------------------- |
| `m`              | 切换平移/阿克曼模式（switch translate/Ackermann mode）                                                               |
| `w` / `s`      | 前进/后退（forward/backward）                                                                                        |
| `a` / `d`      | 平移模式下左移/右移，阿克曼模式下左转/右转（strafe left/right in translate mode, turn left/right in Ackermann mode） |
| `q` / `e` / `z` / `c` | 分别表示左前、右前、左后、右后（front-left, front-right, rear-left, rear-right） |
| `+` / `-`      | 按每次 10% 提高/降低速度倍率，范围为 10% 到 100%，默认 60%（adjust speed scale by 10%, default 60%）               |
| `space` 或 `x` | 停止（stop）                                                                                                         |
| `Esc` 或 `Ctrl-C` | 退出示例，退出前停止运动并下电（quit after stopping motion and powering off）                                  |

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
MAP_DIR="$(mktemp -d /tmp/modi_chassis_mapping_XXXXXX)"
./build/example/chassis_example_slam mapping "$MAP_DIR"
```

该底层示例用于诊断 SLAM，会把结果留在 `/tmp`。正式建图请通过底盘 Scope 的
`/api/mapping/start` 和 `/api/mapping/stop` 完成跨文件系统校验归档。

启动定位：

```bash
MAP_DIR="${MODI_WS_ROOT:-$HOME/modi_ws}/maps/一楼大厅"
./build/example/chassis_example_slam localization "$MAP_DIR"
```

按 `Ctrl-C` 后示例会切回空闲模式。参数含义：

| 参数             | 含义                                        |
| ---------------- | ------------------------------------------- |
| `mapping`      | 启动建图，路径必须是 `/tmp/modi_chassis_mapping_*` 会话目录 |
| `localization` | 启动定位，路径必须是 `$MODI_WS_ROOT/maps` 的直接子目录     |
| `map_path`     | 受上述工作区边界约束的地图目录                            |

在已加载地图且定位有效时，执行一次全向导航：

```bash
./build/example/chassis_example_navigation
```

该示例内部目标点为 `{x=0.5, y=0.0, yaw=0.0}`，单位分别为 m、m、rad。

加载地图、等待定位有效后再导航：

```bash
MODI_WS_ROOT=/home/user/modi_ws \
  ./build/example/chassis_example_localization_navigation 一楼大厅
```

示例只接受地图名称，并将其解析为
`$MODI_WS_ROOT/maps/<地图名称>`；不接受任意绝对路径。目标点仍是源码常量：

| 项目          | 当前值              | 含义                         |
| ------------- | ------------------- | ---------------------------- |
| 第一个参数    | `一楼大厅`          | 工作区 `maps` 下的地图目录名 |
| `kTarget`     | `{0.0, 0.0, 0.0}`   | 导航目标 `x`、`y`、`yaw`    |

需要修改导航目标时调整源码中的 `kTarget` 并重新编译。

## 7. 运行顺序建议

首次联调建议按以下顺序执行：

1. `chassis_example_robot_model`：确认能连上机器人并读取模型。
2. `chassis_example_robot_state`：确认状态、位姿和故障码正常。
3. `chassis_example_manager`：清故障并确认上电、下电流程正常。
4. `chassis_example_speed_translate`：低速单项运动测试。
5. `chassis_example_keyboard_control`：键盘低速联调平移和阿克曼运动。
6. `chassis_example_slam mapping <map_path>`：建图。
7. `chassis_example_slam localization <map_path>`：定位。
8. `chassis_example_navigation`：在地图和定位有效后再运行。

若接口返回负值，先检查机器人端服务是否启动、DDS Domain ID 是否一致、网络是否互通、
急停和故障状态是否正常。
