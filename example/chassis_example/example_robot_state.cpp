/**
 * @brief 演示通过 MODI SDK 读取移动底盘实时状态。
 *
 * 调用流程：创建并启动 modi_sdk::RobotClient，通过 client.Chassis()
 * 获取底盘接口，再通过 chassis->State() 获取状态接口，循环调用
 * GetBaseVelocity()、GetBasePose()、GetNavigationStatus()、
 * GetSteeringPositions()、GetWheelSpeeds()、GetServoState() 和故障码
 * 查询 API 读取状态。
 *
 * 如果传入 x y yaw，示例会先调用 ResetBasePose() 重置底盘位姿。
 * 按 Ctrl-C 结束状态读取，最后调用 client.Stop()。
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

std::atomic<bool> g_running{true};

// Ctrl-C 后让状态读取循环结束，主流程负责停止 SDK 客户端。
void Stop(int) { g_running.store(false); }

}  // namespace

int main(int argc, char** argv) {
  if (argc != 1 && argc != 4) {
    std::cerr << "usage: chassis_example_robot_state [x y yaw]\n";
    return 2;
  }

  // 注册信号处理，允许用户按 Ctrl-C 结束实时状态读取。
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘接口和状态接口。
  auto chassis = client.Chassis();
  auto state = chassis->State();

  if (argc == 4) {
    // 3. 如果提供目标位姿，调用 ResetBasePose() 重置底盘基座位姿。
    const modi_sdk::ChassisBasePose pose{
        std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]), true};
    state->ResetBasePose(pose);
    std::cout << "ResetBasePose() pose=(" << pose.x << ", " << pose.y << ", "
              << pose.yaw << ")\n";
  }

  // 4. 循环调用 State API 读取底盘位姿、速度、导航和关节状态。
  while (g_running.load()) {
    const auto base_velocity = state->GetBaseVelocity();
    const auto base_pose = state->GetBasePose();
    const auto navigation_status = state->GetNavigationStatus();

    std::cout << "base_velocity: " << base_velocity.linear_x << ", "
              << base_velocity.linear_y << ", " << base_velocity.angular_z
              << '\n';
    std::cout << "base_pose: " << base_pose.x << ", " << base_pose.y << ", "
              << base_pose.yaw << " valid=" << base_pose.valid << '\n';
    std::cout << "navigation_status: "
              << static_cast<int>(navigation_status) << '\n';

    // 读取转向关节位置和轮速。
    std::cout << "steering_positions:";
    for (const double value : state->GetSteeringPositions()) {
      std::cout << ' ' << value;
    }
    std::cout << '\n';

    std::cout << "wheel_speeds:";
    for (const double value : state->GetWheelSpeeds()) {
      std::cout << ' ' << value;
    }
    std::cout << '\n';

    // 读取伺服状态以及关节故障、紧急事件和警告码。
    std::cout << "servo_state:";
    for (const auto value : state->GetServoState()) {
      std::cout << ' ' << static_cast<int>(value);
    }
    std::cout << '\n';

    std::cout << "fault_code:";
    for (const auto value : state->GetJointFaultCode()) {
      std::cout << ' ' << static_cast<int>(value);
    }
    std::cout << '\n';

    std::cout << "emcy_code:";
    for (const auto value : state->GetJointEmcyCode()) {
      std::cout << ' ' << static_cast<int>(value);
    }
    std::cout << '\n';

    std::cout << "warning_code:";
    for (const auto value : state->GetJointWarningCode()) {
      std::cout << ' ' << static_cast<int>(value);
    }
    std::cout << '\n';

    std::cout << "global_path: " << state->GetGlobalPath().size()
              << " points\n---\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // 5. 状态读取结束后停止 SDK 客户端。
  client.Stop();
  return 0;
}
