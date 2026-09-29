/**
 * @brief 演示 MODI SDK 的三种底盘速度控制 API。
 *
 * 调用流程：创建并启动 modi_sdk::RobotClient，通过 client.Chassis()
 * 获取 Model、Manager 和 Motion 接口；调用 ClearFaults()、PowerOn() 准备
 * 底盘；使用 SetMaxBaseLinearSpeed()、SetMaxBaseAngularSpeed()、
 * SetMaxBaseLinearAccel() 和 SetMaxBaseAngularAccel() 设置运动限制；
 * 然后分别周期性调用 SpeedTranslate()、SpeedAckermann() 和
 * SpeedHolonomic()；每种运动结束后调用 StopMotion()，程序退出前调用
 * PowerOff() 和 client.Stop()。
 *
 * 按 Ctrl-C 可以提前结束当前测试。速度单位为 m/s，角速度单位为 rad/s。
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

std::atomic<bool> g_running{true};

void Stop(int) { g_running.store(false); }

}  // namespace

int main() {
  // 注册信号处理，允许用户按 Ctrl-C 提前结束当前速度测试。
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘管理、模型和运动控制接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();
  auto model = chassis->Model();
  auto motion = chassis->Motion();

  // 3. 运动前清除故障、确认安全状态正常并给底盘上电。
  if (manager->ClearFaults() != 0 ||
      manager->GetSafetyState() != modi_sdk::SafetyState::kNormal ||
      manager->PowerOn() != 0) {
    std::cerr << "底盘准备失败\n";
    client.Stop();
    return 1;
  }

  // 4. 设置底盘运动限制，后续 Speed*() 调用遵守这些限制。
  // 线速度单位为 m/s，角速度单位为 rad/s，加速度单位分别为 m/s^2、rad/s^2。
  std::cout << "SetMaxBaseLinearSpeed: "
            << model->SetMaxBaseLinearSpeed(0.3) << '\n';
  std::cout << "SetMaxBaseAngularSpeed: "
            << model->SetMaxBaseAngularSpeed(0.6) << '\n';
  std::cout << "SetMaxBaseLinearAccel: "
            << model->SetMaxBaseLinearAccel(0.8) << '\n';
  std::cout << "SetMaxBaseAngularAccel: "
            << model->SetMaxBaseAngularAccel(1.2) << '\n';

  constexpr auto kDuration = std::chrono::seconds(3);
  constexpr auto kCommandPeriod = std::chrono::milliseconds(100);

  // 5. 调用 SpeedTranslate(vx, vy) 控制底盘平移。
  std::cout << "=== SpeedTranslate(vx=0.2, vy=0.0) ===\n";
  auto deadline = std::chrono::steady_clock::now() + kDuration;
  while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
    // 平移速度：vx、vy 的单位都是 m/s。
    if (motion->SpeedTranslate(0.2, 0.0) != 0) {
      std::cerr << "SpeedTranslate() 失败\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(kCommandPeriod);
  }
  // 当前测试结束后停止底盘运动，再进入下一种控制模式。
  (void)motion->StopMotion();

  if (!g_running.load()) {
    (void)manager->PowerOff();
    client.Stop();
    return 0;
  }

  // 6. 调用 SpeedAckermann(vx, wz) 控制阿克曼底盘。
  std::cout << "=== SpeedAckermann(vx=0.15, wz=0.3) ===\n";
  deadline = std::chrono::steady_clock::now() + kDuration;
  while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
    // 阿克曼速度：vx 的单位是 m/s，wz 的单位是 rad/s。
    if (motion->SpeedAckermann(0.15, 0.3) != 0) {
      std::cerr << "SpeedAckermann() 失败\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(kCommandPeriod);
  }
  // 阿克曼测试结束后停止当前运动。
  (void)motion->StopMotion();

  if (!g_running.load()) {
    (void)manager->PowerOff();
    client.Stop();
    return 0;
  }

  // 7. 调用 SpeedHolonomic(vx, vy, wz) 控制全向底盘。
  std::cout << "=== SpeedHolonomic(vx=0.15, vy=0.1, wz=0.2) ===\n";
  deadline = std::chrono::steady_clock::now() + kDuration;
  while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
    // 全向速度：vx、vy 的单位是 m/s，wz 的单位是 rad/s。
    if (motion->SpeedHolonomic(0.15, 0.1, 0.2) != 0) {
      std::cerr << "SpeedHolonomic() 失败\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(kCommandPeriod);
  }

  // 8. 所有速度测试完成后停止运动、下电并关闭 SDK 客户端。
  std::cout << "StopMotion: " << motion->StopMotion() << '\n';
  std::cout << "PowerOff: " << manager->PowerOff() << '\n';
  client.Stop();
  return 0;
}
