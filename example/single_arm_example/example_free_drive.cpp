// SDK 拖动示教示例。
//
// 用法：
//   single_arm_example_free_drive <robot_id> [arm_name]
//
// 目标机器人必须已经处于可运行状态。示例只操作指定机械臂，不自动上电
// 或下电；收到 Ctrl-C 后先关闭拖动示教，再停止 SDK 客户端。

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;

void HandleSignal(int) { g_running = 0; }

const char* ControlModeName(modi_sdk::ControlMode mode) {
  switch (mode) {
    case modi_sdk::ControlMode::kPositionControl:
      return "position control";
    case modi_sdk::ControlMode::kCartesianAdmittance:
      return "cartesian admittance";
    case modi_sdk::ControlMode::kJointAdmittance:
      return "joint admittance";
    case modi_sdk::ControlMode::kCartesianImpedance:
      return "cartesian impedance";
    case modi_sdk::ControlMode::kJointImpedance:
      return "joint impedance";
    case modi_sdk::ControlMode::kFreeDrive:
      return "free drive";
    case modi_sdk::ControlMode::kUnknown:
    default:
      return "unknown";
  }
}

void PrintModes(const std::shared_ptr<modi_sdk::StateApi>& state,
                const std::vector<std::string>& arm_names) {
  const auto modes = state->GetControlMode();
  if (modes.size() != arm_names.size()) {
    std::cout << "control mode snapshot unavailable\n";
    return;
  }
  for (std::size_t i = 0; i < arm_names.size(); ++i) {
    std::cout << "  " << arm_names[i] << ": " << ControlModeName(modes[i])
              << " (" << static_cast<int>(modes[i]) << ")\n";
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: " << argv[0] << " <robot_id> [arm_name]\n";
    return 2;
  }

  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(config);
  if (!client->Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  const auto robot = client->Robot();
  const auto arm_names = robot->Model()->GetArmNames();
  if (arm_names.empty()) {
    std::cerr << "GetArmNames returned no arms\n";
    client->Stop();
    return 1;
  }

  const std::string arm_name = argc == 3 ? argv[2] : arm_names.front();
  if (std::find(arm_names.begin(), arm_names.end(), arm_name) ==
      arm_names.end()) {
    std::cerr << "Unknown arm_name: " << arm_name << "\nAvailable arms:\n";
    for (const auto& name : arm_names) std::cerr << "  " << name << '\n';
    client->Stop();
    return 2;
  }

  auto force = robot->Force();
  auto state = robot->State();
  auto manager = robot->Manager();
  if (manager->GetSafetyState() != modi_sdk::SafetyState::kNormal ||
      manager->GetRobotState() != modi_sdk::RobotState::kRunning) {
    std::cerr << "Free drive requires Safety=Normal and Robot=Running. "
                 "Prepare the robot before running this example.\n";
    client->Stop();
    return 1;
  }

  // HAL 测量重力已通过 sensor→base 外参旋转，仅用于观察。
  // WBC 使用 world 重力与安装位姿计算补偿，不使用这里读到的测量值。
  const auto gravity = force->GetBaseGravity(arm_name);
  if (gravity.size() == 3) {
    std::cout << "HAL base_link gravity for " << arm_name << " (m/s^2): [" << gravity[0]
              << ", " << gravity[1] << ", " << gravity[2] << "]\n";
  } else {
    std::cout << "HAL base_link gravity unavailable; WBC uses world gravity and installation pose.\n";
  }
  if (!g_running) {
    client->Stop();
    return 0;
  }

  // 启用服务成功返回时，后端已完成 CST 切换和首帧力矩下发。
  // 即使启用请求超时，也可能已在后端执行，因此仍需要尝试禁用清理。
  int exit_code = 0;
  const int enable_result = force->EnableFreeDrive(arm_name);
  if (enable_result != 0) {
    std::cerr << "EnableFreeDrive failed: " << enable_result << '\n';
    exit_code = 1;
  } else if (!force->IsFreeDriveEnabled(arm_name)) {
    std::cerr << "Free drive is not enabled or its status query failed.\n";
    exit_code = 1;
  } else {
    std::cout << arm_name
              << " is in free drive. Move the arm by hand; press Ctrl-C to "
                 "stop.\n";
    while (g_running) {
      if (!force->IsFreeDriveEnabled(arm_name)) {
        std::cerr << "Free drive stopped or its status query failed.\n";
        exit_code = 1;
        break;
      }
      PrintModes(state, arm_names);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  // 禁用服务的返回码决定清理是否成功。IsFreeDriveEnabled() 查询失败也
  // 返回 false，不能仅依据这个 bool 判断 CSP 回切已经成功。
  const int disable_result = force->DisableFreeDrive(arm_name);
  if (disable_result != 0) {
    std::cerr << "DisableFreeDrive failed: " << disable_result << '\n';
    exit_code = 1;
  } else {
    std::cout << arm_name << " free drive disabled; current position held.\n";
    PrintModes(state, arm_names);
  }

  client->Stop();
  return exit_code;
}
