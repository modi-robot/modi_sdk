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
  <a href="example"><strong>示例代码</strong></a>
</p>

---

MODI SDK 通过 Fast DDS 与 `modi_system` 通信，为 MODI 机器人应用提供统一的
C++ 接口。SDK 支持机械臂、双臂和移动底盘，可用于模型查询、状态读取、运动控制、
力控制、运动学与动力学计算、设备管理、数字 IO 和末端执行器控制。

正式发布包包含预编译动态库、公开头文件、CMake 配置、运行依赖、示例源码、已编译
示例和用户文档。用户无需编译 SDK 本身，只需安装兼容的 ROS 2 环境并下载对应架构的
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
| 末端与 IO | 支持数字 IO、夹爪和灵巧手等硬件末端执行器 |
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
cd share/modi_sdk/example
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
```

先运行不会主动下发运动目标的状态示例：

```bash
./build/single_arm_example_robot_state
```

发布包也提供预编译示例。在发布包根目录可直接执行：

```bash
./lib/modi_sdk/single_arm_example_robot_state
```

运行前请先启动机器人侧 `modi_system`，并确保 SDK 与 System 的 DDS Domain ID 一致。

## 示例中心

| 目录/程序 | 内容 |
|---|---|
| [`example/single_arm_example`](example/single_arm_example) | 单臂模型、状态、运动、算法、实时话题和末端执行器示例 |
| [`example/dual_arm_example`](example/dual_arm_example) | 双臂模型、状态、运动、算法和实时话题示例 |
| [`example/chassis_example`](example/chassis_example) | 底盘上电、速度控制、状态读取、停止和下电示例 |

运动控制示例会驱动真实设备。首次运行前应阅读源码，降低目标位置、速度和加速度，
确认机器人固定可靠、运动区域安全并且急停可用。

## 文档中心

| 文档 | 内容 |
|---|---|
| [MODI SDK 使用说明](docs/MODI%20SDK%20使用说明.md) | 发布包获取、环境加载、示例编译运行、数据约定和常见问题 |
| [MODI SDK 快速开发指南](docs/MODI%20SDK%20快速开发指南.md) | 从零创建用户工程，完成状态读取、关节运动、实时订阅和底盘控制 |
| [MODI SDK 接口文档](docs/MODI%20SDK%20接口文档.md) | API 参数、返回值、单位、数据结构、故障码和全部实时话题 |

## 发布包结构

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>/
├── include/                    # SDK 和依赖的公开头文件
├── lib/                        # SDK、DDS、运行库及预编译示例
├── share/
│   └── modi_sdk/
│       ├── docs/               # 用户文档
│       └── example/            # 可独立编译的示例源码
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
