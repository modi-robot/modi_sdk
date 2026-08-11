# MODI SDK 使用说明

## 1. SDK 简介

MODI SDK 是面向 MODI 机器人系统的 C++17 客户端开发套件。SDK 通过 DDS 与
`modi_system` 通信，提供机械臂、双臂和移动底盘的状态读取、运动控制、算法、
力控、设备管理、数字 IO 与末端执行器接口。

SDK 不直接驱动电机。运行 SDK 程序前，应先在机器人控制器上启动与 SDK 版本兼容的
`modi_system`。

## 2. 运行要求

- Linux x86_64 或 ARM64。
- 与发布包一致的 Ubuntu、glibc 和 ROS 2 发行版；推荐 Ubuntu 22.04、ROS 2 Humble。
- SDK 与 `modi_system` 使用兼容的 `modi_system_msgs`、Fast DDS 主次版本和 QoS。
- SDK 的 DDS Domain ID 与机器人系统一致。
- 运动控制前确认机器人周围无人、急停可用且机械限位正确。

## 3. 获取并加载 SDK

从发布页下载与目标平台匹配的压缩包：

[https://github.com/modi-robot/modi_sdk/releases](https://github.com/modi-robot/modi_sdk/releases)

文件名格式：

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>.zip
```

解压并加载环境：

```bash
unzip modi_sdk_*.zip -d modi_sdk
cd modi_sdk
source /opt/ros/humble/setup.bash
source setup.bash
```

每次打开新终端都需要执行两条 `source` 命令。若希望自动加载，可将实际路径写入
`~/.bashrc`。

## 4. 发布包结构

```text
modi_sdk/
├── include/                 # SDK 及依赖的公开头文件
├── lib/                     # SDK、DDS 与随包运行库
│   └── modi_sdk/            # 已编译示例程序
├── share/                   # CMake、ament 与接口资源
├── setup.bash
├── local_setup.bash
├── version
└── CHANGELOG.md
```

用户文档安装在 `share/modi_sdk/docs/`。

## 5. 编译并运行示例

发布包在 `share/modi_sdk/example/` 中提供示例源码。无需获取 SDK 内部源码，
只需要加载 ROS 2 和当前 SDK 发布包环境即可编译。

进入 SDK 解压目录并执行：

```bash
source /opt/ros/humble/setup.bash
source setup.bash

cd share/modi_sdk/example
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

编译完成后，可先运行不会主动下发运动目标的单臂状态示例：

```bash
./build/single_arm_example_robot_state
```

如果不需要重新编译，发布包的 `lib/modi_sdk/` 目录也包含已编译的示例程序。
在 SDK 解压目录可以直接运行：

```bash
./lib/modi_sdk/single_arm_example_robot_state
```

其他示例包括：

- `single_arm_example_robot_model`：读取单臂模型信息。
- `single_arm_example_motion_control`：单臂运动控制。
- `single_arm_example_algorithm`：运动学与动力学算法。
- `single_arm_example_rt_topic`：实时话题订阅。
- `single_arm_example_topic_publish`：实时话题发布。
- `single_arm_example_eef`：末端执行器控制。
- `dual_arm_example_*`：双臂对应示例。
- `chassis_example_speed_translate`：底盘上电、平移、停止和下电。

运动类示例会驱动真实设备。首次运行前应阅读源码并减小目标位置、速度和加速度。

## 6. 客户端配置与生命周期

```cpp
modi_sdk::RobotClient::Config config;
config.domain_id = 0;
config.spin_frequency_hz = 1000.0;
config.spin_cpu_id = 3;
config.spin_priority = 90;

modi_sdk::RobotClient client(config);
if (!client.Start()) {
  return 1;
}

// 调用 Robot() 或 Chassis() 下的接口。

client.Stop();
```

使用规则：

1. 实时订阅和发布必须在 `Start()` 前注册。
2. `Start()` 成功后再读取状态或调用服务。
3. `Robot()`、`Chassis()` 及其子接口与客户端共享通信资源。
4. 子接口不得在 `RobotClient` 销毁后继续使用。
5. 程序退出前调用 `Stop()`。

## 7. 返回值与数据约定

多数控制和配置接口遵循以下约定：

- `0`：成功。
- 负值：本地参数、通信、超时或服务端拒绝等错误。
- 状态查询返回空向量：通常表示数据尚未收到或通信配置不匹配。

常用单位：

| 数据 | 单位或格式 |
|---|---|
| 关节位置、速度、加速度 | rad、rad/s、rad/s² |
| 关节力矩 | N·m |
| 笛卡尔位姿 | `[x,y,z,roll,pitch,yaw]`，m/rad |
| 笛卡尔速度 | `[vx,vy,vz,wx,wy,wz]`，m/s、rad/s |
| 六维力 | `[fx,fy,fz,tx,ty,tz]`，N/N·m |
| 底盘位置和速度 | m、m/s、rad、rad/s |

多机器人、多末端的二维向量外层顺序以系统配置和模型查询结果为准。

## 8. 常见问题

### 8.1 `RobotClient::Start()` 失败

- 确认已加载 ROS 2 和 SDK 的 `setup.bash`。
- 确认发布包架构与目标机一致。
- 检查依赖动态库：`ldd ./lib/libmodi_sdk.so | grep "not found"`。

### 8.2 状态接口持续返回空向量

- 确认 `modi_system` 已启动。
- 确认两端 Domain ID 相同。
- 确认两端网络互通，且防火墙未拦截 DDS 流量。
- 确认 SDK 与 System 使用兼容版本。

### 8.3 服务接口返回负值

- `-1` 通常表示本地创建或参数错误。
- `-2` 通常表示服务调用超时。
- `-3` 通常表示服务端拒绝请求。

实际含义以运行日志和具体接口实现为准。

### 8.4 实时线程启动失败

检查 `spin_cpu_id` 是否存在、CPU 是否在线，以及当前用户是否具有实时调度权限。
调试阶段可先将 CPU 设为当前机器存在的逻辑核，并降低实时优先级。

## 9. 相关文档

- [MODI SDK 快速开发指南](MODI%20SDK%20快速开发指南.md)
- [MODI SDK 接口文档](MODI%20SDK%20接口文档.md)
