<p align="center">
  <img src="docs/pics/modi.png" alt="MODI" width="300">
</p>

<h1 align="center">MODI SDK</h1>

<p align="center">
  面向 MODI 机器人系统的 Linux C++ 开发套件
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Release-v0.0.1-2f80ed" alt="Release v0.0.1">
  <img src="https://img.shields.io/badge/Platform-Linux-fcc624?logo=linux&logoColor=black" alt="Linux">
  <img src="https://img.shields.io/badge/Arch-x86__64%20%7C%20ARM64-5c6bc0" alt="x86_64 and ARM64">
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314e?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599c?logo=c%2B%2B" alt="C++17">
  <img src="https://img.shields.io/badge/DDS-Fast%20DDS-0082c9" alt="Fast DDS">
</p>

<p align="center">
  <a href="https://github.com/modi-robot/modi_sdk/releases"><strong>下载 SDK</strong></a>
  ·
  <a href="docs/MODI%20SDK%20使用说明.md"><strong>使用说明</strong></a>
  ·
  <a href="docs/MODI%20SDK%20快速开发指南.md"><strong>快速开发</strong></a>
  ·
  <a href="docs/MODI%20SDK%20接口文档.md"><strong>接口文档</strong></a>
  ·
  <a href="docs/MODI%20SDK%20错误码说明.md"><strong>错误码</strong></a>
  ·
  <a href="example"><strong>示例代码</strong></a>
</p>

---

MODI SDK 通过 Fast DDS 与 `modi_system` 通信，为 MODI 机器人应用提供统一的
C++ 接口。SDK 支持机械臂、双臂和移动底盘，可用于模型查询、状态读取、运动控制、
力控制、运动学与动力学计算、设备管理、数字 IO 和末端执行器控制。

正式发布包包含预编译动态库、公开头文件、CMake 配置、运行依赖、示例源码和用户文档。
用户无需编译 SDK 本身，只需安装兼容的 ROS 2 环境并下载对应架构的
发布包，即可开始应用开发。

## 主要功能

| 功能 | 说明 |
|---|---|
| 机器人模型 | 查询关节、末端、自由度、限位、工具、负载和安装位姿 |
| 状态读取 | 获取关节位置、速度、加速度、力矩、电流、电压、温度和故障状态 |
| 运动控制 | 支持关节、直线、位姿、圆弧、速度和周期伺服控制 |
| 力控制 | 支持笛卡尔/关节导纳与阻抗控制、目标力和参数配置 |
| 算法接口 | 提供正逆运动学、逆动力学、任务权重和负载标定 |
| 设备管理 | 提供上下电、抱闸、故障清除、急停和状态机查询 |
| 末端与 IO | 支持数字 IO 和工具板 EG2 夹爪 |
| 移动底盘 | 支持底盘模型、状态、速度、位姿运动和安全管理 |
| 实时话题 | 提供固定周期的实时订阅与发布回调 |

## 支持环境

| 项目 | 要求 |
|---|---|
| 操作系统 | Linux，推荐 Ubuntu 22.04 |
| 处理器架构 | x86_64、ARM64（aarch64） |
| ROS 2 | Humble，需与发布包构建版本兼容 |
| 编译器 | 支持 C++17 的 GCC 或 Clang |
| 构建工具 | CMake 3.16 或更高版本 |
| 服务端 | 与 SDK 版本兼容的 `modi_system` |
| 通信 | Fast DDS；两端 Domain ID、消息和 QoS 配置必须兼容 |

当前 SDK 的构建和运行依赖 ROS 2/ament 环境。它不是完全脱离 ROS 2 的独立 Fast DDS
软件包。

## 下载 SDK

前往 **[GitHub Releases](https://github.com/modi-robot/modi_sdk/releases)** 下载与目标机器
架构匹配的发布包。

文件名格式：

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>.zip
```

例如：

```text
modi_sdk_0.0.1_linux_x86_64_a1b2c3d.zip
modi_sdk_0.0.1_linux_arm64_a1b2c3d.zip
```

查看目标机器架构：

```bash
uname -m
```

## 快速开始

### 1. 安装基础环境

推荐使用 Ubuntu 22.04 和 ROS 2 Humble。编译示例还需要：

```bash
sudo apt update
sudo apt install -y build-essential cmake unzip
```

### 2. 解压并加载环境

```bash
unzip modi_sdk_*_linux_*.zip
cd modi_sdk_*_linux_*

source /opt/ros/humble/setup.bash
source ./setup.bash
```

如果当前目录有多个发布包，请将通配符替换为实际文件名和解压目录名。

### 3. 编译示例

```bash
cmake -S example -B build/example -DCMAKE_BUILD_TYPE=Release
cmake --build build/example --parallel "$(nproc)"
```

先运行不会主动下发运动目标的状态示例：

```bash
./build/example/single_arm_example_robot_state
```

运行前请先启动机器人侧 `modi_system`，并确保 SDK 与 System 的 DDS Domain ID 一致。

## 示例中心

| 目录/程序 | 内容 |
|---|---|
| [`example/single_arm_example`](example/single_arm_example) | 单臂模型、状态、运动、算法、实时话题和末端执行器示例 |
| [`example/dual_arm_example`](example/dual_arm_example) | 双臂模型、状态、运动、算法和实时话题示例 |
| [`example/single_arm_example/example_free_drive.cpp`](example/single_arm_example/example_free_drive.cpp) | 按臂启用拖动示教、读取 HAL 重力及工作模式、退出时禁用拖动 |
| [`example/dual_arm_example/example_follow_joint.cpp`](example/dual_arm_example/example_follow_joint.cpp) | MoveJoint 到预设双臂关节位置后，以 10 Hz 采样中心附近随机笛卡尔目标，经 IK 下发 FollowJoint |
| [`example/dual_arm_example/example_follow_pose.cpp`](example/dual_arm_example/example_follow_pose.cpp) | MoveJoint 到相同预设关节后，以 10 Hz 直接下发随机 xyzRPY 给 FollowPose，由 WBC 内部求 IK |
| [`example/dual_arm_example/example_servo_joint.cpp`](example/dual_arm_example/example_servo_joint.cpp) | 双臂全部被控关节 MoveJoint 回零后，以 5 ms 周期、0.01° 步进在 ±30° 间循环 |
| [`example/dual_arm_example/example_tool_gripper.cpp`](example/dual_arm_example/example_tool_gripper.cpp) | 每侧夹爪按指定周期调用 SetToolGripperPosition，默认 100 ms（10 Hz），每十次发送最大开度并等待到位，只打印启动信息和错误 |
| [`example/chassis_example`](example/chassis_example) | 底盘上电、速度控制、状态读取、停止和下电示例 |

运动控制示例会驱动真实设备。首次运行前应阅读源码，降低目标位置、速度和加速度，
确认机器人固定可靠、运动区域安全并且急停可用。

FollowJoint 示例先等待 MoveJoint 到预设初始关节，再记录双臂当前末端位姿作为固定中心。
循环在世界坐标 xyz 各 ±10 cm、RPY 各 ±30° 内独立采样，使用当前实测关节作为 IK 参考，
求解成功且关节解在安全限位内时下发 FollowJoint；无解目标跳过，Ctrl+C 调用 StopMotion。
FollowJoint 与 MoveJoint 共用按臂后台调度；重规划期间保持最后位置输出，完成后恢复采样。
连续 Follow 的规划起点继承最近指令的关节位置、速度和加速度，不预先 StopMotion 或 Reset；
后台保持期间的实际速度、加速度连续性不作保证。
`Motion()->FollowPose(target_poses, velocity, acceleration)` 可直接提交世界坐标系
`xyzRPY` 目标，位置单位 m，姿态单位 rad，目标顺序与末端列表一致。
WBC 以最近输出的关节指令为 IK 参考，复用支持臂角搜索的后台求解器，成功后
调用 FollowJoint；速度和加速度仍为关节上限（rad/s、rad/s²），默认 0.1、0.5。
IK 或规划失败时返回错误并保留原轨迹；末端路径不保证为笛卡尔直线。
目标循环频率为 10 Hz，每秒打印实际循环频率、接受/跳过次数和周期超时次数；不补发积压周期。
需要配套使用后台 IK 查询版本的服务端，求解过程不暂停 WBC，也不重置正在执行的轨迹。
后台普通 CCS IK 在 A/B 分支无解或解的关节限位余量不足 3° 时补充搜索其他臂角，
普通解与搜索解均保留原约束及至少 3° 关节限位余量；
几何不可达或有限采样仍无解的目标继续跳过。
运行前手动上电、退出拖动并结束已有运动。

```bash
./build/example/dual_arm_example_follow_joint <robot_id> [domain_id]
```

FollowPose 示例沿用相同初始关节、固定中心和随机范围，直接调用 FollowPose，
客户端不调用 SolveIkPos，也不逐周期读取参考关节。IK 无解或轨迹规划失败计为跳过，
其他服务错误结束循环；每秒打印实际频率、接受/跳过次数、周期超时和最近返回码。
Ctrl+C 调用 StopMotion；后台重规划期间保持最后位置输出，末端路径不保证为直线。

```bash
./build/example/dual_arm_example_follow_pose <robot_id> [domain_id]
```

双臂夹爪示例从模型获取两个机械臂名称，每侧按指定周期发送独立随机开度。
周期默认 100 ms（每侧 10 Hz，合计每秒目标 20 次 SDK 调用），可通过最后的 `周期_ms` 参数设置，须为正整数。
默认范围为 1~100%，包含上下限；可指定范围、运行秒数和 DDS 域，原有参数顺序保持不变。
每侧第 10、20、30……次调用固定发送设定的最大开度，其余次数在指定范围内随机取值。
每次发送最大开度（含随机命中最大值）后，该侧暂停后续指令，等待发送后新收到的有效反馈确认到位：
实际开度与目标相差不超过 1%，运行状态不再处于夹取中或张开中，且无设备故障。
等待最长 10 秒，反馈超过 500 ms 未更新、反馈无效或设备故障时结束测试；另一侧独立运行。
到位后按指定周期重新发送，不补发等待期间的目标，因此包含等待时间的实际平均发送频率会低于目标频率。
运行前须手动上电，使系统允许运动。只打印启动信息和错误，反馈订阅仅用于到位判断。
调用超期会跳过错过的周期，不集中补发；
任一调用返回非零时结束双侧发送。运行时间为 0 时持续到 Ctrl+C，退出时请求停止两侧夹爪。
返回 0 表示目标已接收，不能据此判断夹爪已经到位；设定周期控制 SDK 调用频率，HAL 可能合并待发目标。

```bash
./build/example/dual_arm_example_tool_gripper <robot_id>
# 在 1~100% 之间随机发送，运行 60 秒，DDS domain_id 为 0：
./build/example/dual_arm_example_tool_gripper <robot_id> 1 100 60 0
# 显式指定 50 ms 周期（每侧 20 Hz）：
./build/example/dual_arm_example_tool_gripper <robot_id> 1 100 60 0 50
```

## 文档中心

| 文档 | 内容 |
|---|---|
| [MODI SDK 使用说明](docs/MODI%20SDK%20使用说明.md) | 发布包获取、环境加载、示例编译运行、数据约定和常见问题 |
| [MODI SDK 快速开发指南](docs/MODI%20SDK%20快速开发指南.md) | 从零创建用户工程，完成状态读取、关节运动、实时订阅和底盘控制 |
| [MODI SDK 接口文档](docs/MODI%20SDK%20接口文档.md) | API 参数、返回值、单位、数据结构、故障码和全部实时话题 |
| [MODI SDK 错误码说明](docs/MODI%20SDK%20错误码说明.md) | 操作错误码和设备诊断码的名称、含义及排查或恢复建议 |
| [底盘 SDK 示例](docs/%E5%BA%95%E7%9B%98%20SDK%20%E7%A4%BA%E4%BE%8B.md) | 移动底盘示例的编译、运行和安全说明 |

## 发布包结构

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>/
├── README.md                  # SDK 总览和文档入口
├── docs/                      # 用户文档和自动生成的错误码说明
├── example/                   # 可独立编译的示例源码
├── include/                    # SDK 和依赖的公开头文件
├── lib/                        # SDK、DDS 和运行依赖库
├── share/                      # CMake、ROS 2 和运行时资源
├── setup.bash
├── local_setup.bash
├── version
└── CHANGELOG.md
```

## 使用提示

- SDK 与 `modi_system` 应使用兼容版本，并保持 DDS Domain ID 一致。
- 实时回调中不要执行文件 I/O、终端输出、服务调用或其他阻塞操作。
- 关节和末端数组的顺序应通过模型接口确认，不要硬编码与机型相关的数组长度。
- 控制程序退出前应主动停止运动、下电并调用 `RobotClient::Stop()`。
- 设备故障、急停或通信异常时，不要继续发送运动目标。

## 问题反馈

使用中发现问题时，请在 [Issues](https://github.com/modi-robot/modi_sdk/issues) 页面提交，
并尽量提供：

- SDK 与 `modi_system` 版本
- 发布包完整文件名及系统架构
- Ubuntu、ROS 2 和内核版本
- DDS Domain ID 与网络环境
- 可复现步骤、完整错误输出和相关日志
