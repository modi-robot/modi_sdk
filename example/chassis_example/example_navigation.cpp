/**
 * @brief 演示使用 MODI SDK 执行底盘地图导航。
 *
 * 调用流程：创建并启动 RobotClient，获取 Chassis、Manager、Motion 和
 * State 接口，调用 ClearFaults()、GetSafetyState() 和 PowerOn() 准备底盘，
 * 调用 moveHolonomicInMap() 发送地图目标，再通过 GetNavigationStatus()
 * 和 GetBasePose() 查询导航状态，结束后调用 StopMotion()、PowerOff()
 * 和 client.Stop()。
 *
 * 使用前应确保服务端已加载地图，并根据实际地图修改 target。
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

  // 2. 获取底盘管理、运动和状态接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();
  auto motion = chassis->Motion();
  auto state = chassis->State();

  // 3. 导航前清除故障、确认安全状态正常并给底盘上电。
  if (manager->ClearFaults() != 0 ||
      manager->GetSafetyState() != modi_sdk::SafetyState::kNormal ||
      manager->PowerOn() != 0) {
    std::cerr << "底盘准备失败\n";
    client.Stop();
    return 1;
  }

  // 4. 设置地图目标位姿：x、y、yaw 的单位分别为 m、m、rad。
  const modi_sdk::ChassisBasePose target{0.5, 0.0, 0.0, true};
  // 调用 moveHolonomicInMap() 发送全向地图导航目标。
  const int result = motion->moveHolonomicInMap(target);
  if (result != 0) {
    std::cerr << "moveHolonomicInMap failed: " << result
              << " (verify that a map is loaded and the target is free)\n";
    (void)motion->StopMotion();
    (void)manager->PowerOff();
    client.Stop();
    return 1;
  }

  // 5. 通过 State API 轮询导航状态和当前底盘位姿。
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(30);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto status = state->GetNavigationStatus();
    const auto pose = state->GetBasePose();
    std::cout << "navigation=" << static_cast<int>(status) << " pose=("
              << pose.x << ", " << pose.y << ", " << pose.yaw << ")\n";
    if (status == modi_sdk::ChassisNavigationStatus::kFinished) {
      // 导航完成后停止运动、下电并关闭 SDK 客户端。
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 0;
    }
    if (status == modi_sdk::ChassisNavigationStatus::kError) {
      std::cerr << "Navigation failed\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // 超时或导航失败时同样执行停止、下电和客户端关闭。
  std::cerr << "Navigation timed out\n";
  (void)motion->StopMotion();
  (void)manager->PowerOff();
  client.Stop();
  return 1;
}
