# MODI SDK 快速开发指南

本文从一个空目录开始创建 MODI SDK C++ 程序，完成客户端启动、机器人状态读取和
安全的关节运动调用。

## 1. 准备环境

先启动机器人侧 `modi_system`。然后打开终端，进入 SDK 解压后的根目录并加载环境：

```bash
source /opt/ros/humble/setup.bash
source ./setup.bash
```

执行 `source ./setup.bash` 时，当前目录中应能看到 `setup.bash` 文件。

## 2. 创建工程

```bash
mkdir -p ~/modi_sdk_demo/src
cd ~/modi_sdk_demo
```

创建 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.16)
project(modi_sdk_demo LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(modi_sdk REQUIRED)

add_executable(modi_sdk_demo src/main.cpp)
target_compile_features(modi_sdk_demo PRIVATE cxx_std_17)
ament_target_dependencies(modi_sdk_demo modi_sdk)

install(TARGETS modi_sdk_demo DESTINATION lib/${PROJECT_NAME})
ament_package()
```

## 3. 完整状态读取程序

创建 `src/main.cpp`：

```cpp
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void SignalHandler(int) { running = 0; }
}

int main() {
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config config;
  config.domain_id = 0;  // 必须与机器人系统一致

  modi_sdk::RobotClient client(config);
  auto robot = client.Robot();
  auto model = robot->Model();
  auto state = robot->State();

  if (!client.Start()) {
    std::cerr << "启动 MODI SDK 失败\n";
    return 1;
  }

  const auto names = model->GetActuatorJointNames();
  std::cout << "驱动关节数量: " << names.size() << '\n';

  while (running) {
    const auto positions = state->GetActuatorJointPositions();
    if (positions.empty()) {
      std::cerr << "尚未收到关节状态\n";
    } else {
      for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto name = i < names.size() ? names[i] : std::to_string(i);
        std::cout << name << "=" << positions[i] << " rad ";
      }
      std::cout << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  client.Stop();
  return 0;
}
```

## 4. 编译与运行

```bash
cd ~/modi_sdk_demo
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
./build/modi_sdk_demo
```

按 `Ctrl+C` 停止程序。

## 5. 完整关节运动示例

本节给出一个完整的单臂关节运动程序。程序启动 SDK 后等待有效关节状态，清除故障并
上电，然后让第一个驱动关节相对当前位置移动 `0.05 rad`，最后停止运动并下电。

首次运行运动程序前应确认：

- 机器人安装牢固，运动范围内没有人员和障碍物。
- 急停按钮可用，机械臂没有未处理故障。
- SDK 的 Domain ID 与机器人系统一致。
- 当前状态返回的关节数量与实际机型一致。

将 `src/main.cpp` 替换为：

```cpp
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"

int main() {
  modi_sdk::RobotClient::Config config;
  config.domain_id = 0;

  modi_sdk::RobotClient client(config);
  auto robot = client.Robot();
  auto model = robot->Model();
  auto state = robot->State();
  auto motion = robot->Motion();
  auto manager = robot->Manager();

  if (!client.Start()) {
    std::cerr << "启动 MODI SDK 失败\n";
    return 1;
  }

  // SDK 启动后 DDS 数据可能尚未到达，最多等待 5 秒。
  std::vector<double> current_positions;
  const auto state_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < state_deadline) {
    current_positions = state->GetActuatorJointPositions();
    if (!current_positions.empty()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  const int joint_dof = model->GetActuatorJointDOF();
  if (current_positions.empty() || joint_dof <= 0 ||
      current_positions.size() != static_cast<std::size_t>(joint_dof)) {
    std::cerr << "没有收到有效关节状态，取消运动\n";
    client.Stop();
    return 1;
  }

  int result = manager->ClearFaults();
  if (result != 0) {
    std::cerr << "清除故障失败: " << result << '\n';
    client.Stop();
    return 1;
  }

  result = manager->PowerOn();
  if (result != 0) {
    std::cerr << "机器人上电失败: " << result << '\n';
    client.Stop();
    return 1;
  }

  // 始终从当前位置生成小幅目标，不使用与机型相关的固定关节数组。
  auto target_positions = current_positions;
  target_positions[0] += 0.05;

  constexpr double kVelocity = 0.1;      // rad/s
  constexpr double kAcceleration = 0.1;  // rad/s²
  result = motion->MoveJoint(target_positions, kVelocity, kAcceleration, true);
  if (result != 0) {
    std::cerr << "MoveJoint 失败: " << result << '\n';
    const int stop_result = motion->StopMotion();
    if (stop_result != 0) {
      std::cerr << "StopMotion 失败: " << stop_result << '\n';
    }
  } else {
    std::cout << "关节运动完成\n";
  }

  // 无论运动是否成功，退出前都尝试停止并下电。
  motion->StopMotion();
  const int power_off_result = manager->PowerOff();
  if (power_off_result != 0) {
    std::cerr << "机器人下电失败: " << power_off_result << '\n';
  }

  client.Stop();
  return result == 0 && power_off_result == 0 ? 0 : 1;
}
```

使用第 4 节的命令重新编译并运行：

```bash
cmake --build build -j"$(nproc)"
./build/modi_sdk_demo
```

`MoveJoint()` 参数说明：

- `target_positions`：所有驱动关节的目标位置，顺序与
  `GetActuatorJointNames()` 一致，单位为 rad。
- `velocity`：规划速度，单位 rad/s。
- `acceleration`：规划加速度，单位 rad/s²。
- `blocked=true`：等待本次运动结束后返回。

示例使用很小的位移和速度，但并不能替代用户程序中的软限位、碰撞检测、超时监控和
急停处理。

## 6. 完整实时话题订阅示例

`Robot()->State()` 适合普通状态查询；如果需要 SDK 按固定频率执行回调，或者希望在
同一个周期内读取多个话题的最新值，可以使用 `RegisterSubscriber()`。

实时回调运行在 SDK 的 spin 线程。回调中应只复制必要数据，不要打印日志、访问文件、
调用运动服务或执行其他可能阻塞的操作。下面的程序在回调中更新快照，在普通主线程中
每 `500 ms` 打印一次关节位置。

```cpp
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"
#include "modi_type_def.h"

namespace {
std::atomic<bool> running{true};

void SignalHandler(int) {
  running.store(false, std::memory_order_relaxed);
}

struct JointSnapshot {
  std::mutex mutex;
  std::vector<double> positions;
  std::vector<double> velocities;
  bool received{false};
};
}  // namespace

int main() {
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config config;
  config.domain_id = 0;
  config.spin_frequency_hz = 1000.0;
  config.spin_cpu_id = 3;
  config.spin_priority = 90;

  modi_sdk::RobotClient client(config);
  JointSnapshot snapshot;

  const std::vector<std::string> topics = {
      "/Robot/State/JointPositions",
      "/Robot/State/JointVelocities",
  };

  // 必须在 Start() 前注册实时话题。
  const int register_result = client.RegisterSubscriber(
      topics, [&snapshot](const modi_sdk::Parser& parser) {
        const auto position_msg = parser.Value<JointPositions>(
            "/Robot/State/JointPositions");
        const auto velocity_msg = parser.Value<JointVelocities>(
            "/Robot/State/JointVelocities");

        std::lock_guard<std::mutex> lock(snapshot.mutex);
        snapshot.positions = position_msg.joint_positions();
        snapshot.velocities = velocity_msg.joint_velocities();
        snapshot.received = !snapshot.positions.empty();
      });

  if (register_result != 0) {
    std::cerr << "注册实时话题失败: " << register_result << '\n';
    return 1;
  }

  if (!client.Start()) {
    std::cerr << "启动 MODI SDK 失败\n";
    return 1;
  }

  while (running.load(std::memory_order_relaxed)) {
    std::vector<double> positions;
    std::vector<double> velocities;
    bool received = false;

    // 锁内只复制数据，打印操作放在锁外执行。
    {
      std::lock_guard<std::mutex> lock(snapshot.mutex);
      positions = snapshot.positions;
      velocities = snapshot.velocities;
      received = snapshot.received;
    }

    if (!received) {
      std::cout << "等待实时关节状态...\n";
    } else {
      std::cout << "joint positions:";
      for (double value : positions) std::cout << ' ' << value;
      std::cout << "\njoint velocities:";
      for (double value : velocities) std::cout << ' ' << value;
      std::cout << "\n---\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  client.Stop();
  return 0;
}
```

注意事项：

- 话题名称必须以单个 `/` 开头，并且存在于 SDK 话题注册表中。
- `Parser::Value<MsgT>()` 的消息类型必须与该话题实际类型一致。
- `spin_cpu_id` 必须是目标机器存在且在线的逻辑 CPU；否则需要修改配置。
- 实时线程和主线程共享数据时必须使用互斥量、无锁队列或其他线程安全机制。
- 如果只读取单个状态且不要求固定周期，优先使用 `StateApi`，代码更简单。

## 7. 完整底盘控制示例

下面的程序完成底盘故障清除、上电、低速直行、停止、状态读取和下电。第一次测试时
建议架空驱动轮，或者确保底盘前方有足够空间并安排人员随时操作急停。

```cpp
#include <chrono>
#include <iostream>
#include <thread>

#include "modi_sdk/robot_client.hpp"

int main() {
  modi_sdk::RobotClient::Config config;
  config.domain_id = 0;

  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "启动 MODI SDK 失败\n";
    return 1;
  }

  auto chassis = client.Chassis();
  auto model = chassis->Model();
  auto state = chassis->State();
  auto motion = chassis->Motion();
  auto manager = chassis->Manager();

  int result = manager->ClearFaults();
  if (result != 0) {
    std::cerr << "底盘清除故障失败: " << result << '\n';
    client.Stop();
    return 1;
  }

  result = manager->PowerOn();
  if (result != 0) {
    std::cerr << "底盘上电失败: " << result << '\n';
    client.Stop();
    return 1;
  }

  constexpr double kLinearSpeed = 0.1;  // m/s
  constexpr double kLateralSpeed = 0.0;
  constexpr auto kMotionDuration = std::chrono::seconds(1);

  // 将系统最大线速度限制到本次测试速度。
  result = model->SetMaxBaseLinearSpeed(kLinearSpeed);
  if (result != 0) {
    std::cerr << "设置底盘最大线速度失败: " << result << '\n';
    motion->StopMotion();
    manager->PowerOff();
    client.Stop();
    return 1;
  }

  result = motion->SpeedTranslate(kLinearSpeed, kLateralSpeed);
  if (result != 0) {
    std::cerr << "底盘速度指令失败: " << result << '\n';
  } else {
    std::this_thread::sleep_for(kMotionDuration);
  }

  // 不论速度指令是否成功，都主动发送停止指令。
  const int stop_result = motion->StopMotion(0.5);
  if (stop_result != 0) {
    std::cerr << "底盘停止失败: " << stop_result << '\n';
  }

  const auto velocity = state->GetBaseVelocity();
  std::cout << "base velocity: vx=" << velocity.linear_x
            << " m/s, vy=" << velocity.linear_y
            << " m/s, wz=" << velocity.angular_z << " rad/s\n";

  const auto pose = state->GetBasePose();
  if (pose.valid) {
    std::cout << "base pose: x=" << pose.x << " m, y=" << pose.y
              << " m, yaw=" << pose.yaw << " rad\n";
  } else {
    std::cout << "当前没有有效底盘位姿\n";
  }

  const int power_off_result = manager->PowerOff();
  if (power_off_result != 0) {
    std::cerr << "底盘下电失败: " << power_off_result << '\n';
  }

  client.Stop();
  return result == 0 && stop_result == 0 && power_off_result == 0 ? 0 : 1;
}
```

底盘运动接口的主要区别：

- `SpeedTranslate(vx, vy)`：控制 x、y 方向平移速度，单位 m/s。
- `SpeedRotate(vx, vz)`：控制前向速度和绕 z 轴角速度，单位 m/s、rad/s。
- `MoveBase(...)`：运动到目标二维位姿，适合有定位信息的底盘。
- `StopMotion(acceleration)`：按指定减速度停止当前运动。

速度模式下不要只依赖一次速度指令后程序自然退出。正常路径、错误路径和信号退出路径都
应显式停止底盘；生产程序还应增加指令超时、通信中断和急停处理。

## 8. 下一步

- 查看 [MODI SDK 接口文档](MODI%20SDK%20接口文档.md) 选择功能接口。
- 阅读发布包 `share/modi_sdk/example/` 中的单臂、双臂和底盘示例，获取更多完整用法。
- 部署和通信问题参考 [MODI SDK 使用说明](MODI%20SDK%20使用说明.md)。
