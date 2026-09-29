/**
 * @brief 演示使用 MODI SDK 控制阿克曼底盘速度。
 *
 * 调用流程：创建并启动 RobotClient，获取 Chassis、Manager 和 Motion
 * 接口，调用 ClearFaults()、GetSafetyState() 和 PowerOn() 准备底盘，
 * 然后周期性调用 SpeedAckermann(vx, wz) 发送速度，最后调用
 * StopMotion()、PowerOff() 和 client.Stop()。
 *
 * 本示例发送 vx=0.1 m/s、wz=0.2 rad/s，持续约 3 秒。
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "modi_sdk/robot_client.hpp"

int main() {
  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘管理和运动控制接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();
  auto motion = chassis->Motion();

  // 3. 清除故障、确认安全状态正常并给底盘上电。
  if (manager->ClearFaults() != 0 ||
      manager->GetSafetyState() != modi_sdk::SafetyState::kNormal ||
      manager->PowerOn() != 0) {
    std::cerr << "底盘准备失败\n";
    client.Stop();
    return 1;
  }

  // 4. 周期性调用 SpeedAckermann(vx, wz) 发送阿克曼速度。
  // vx 的单位为 m/s，wz 的单位为 rad/s。
  std::cout << "SpeedAckermann(vx=0.1 m/s, wz=0.2 rad/s)\n";
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    // 速度控制期间需要持续发送目标速度。
    const int result = motion->SpeedAckermann(0.1, 0.2);
    if (result != 0) {
      std::cerr << "SpeedAckermann() 失败：" << result << '\n';
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 5. 速度测试完成后停止运动、下电并关闭 SDK 客户端。
  (void)motion->StopMotion();
  (void)manager->PowerOff();
  client.Stop();
  return 0;
}
