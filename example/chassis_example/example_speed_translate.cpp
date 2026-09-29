/**
 * @brief 演示使用 MODI SDK 设置底盘速度限制并进行平移控制。
 *
 * 调用流程：创建并启动 modi_sdk::RobotClient，通过 client.Chassis()
 * 获取 Model、Manager、Motion 和 State 接口；调用 ClearFaults()、
 * GetSafetyState() 和 PowerOn() 准备底盘；调用
 * SetMaxBaseLinearSpeed() 修改本示例使用的线速度上限；周期性调用
 * SpeedTranslate(vx, vy) 发送平移速度；最后调用 StopMotion()、
 * PowerOff() 和 client.Stop()。
 *
 * 本示例发送 vx=0.2 m/s、vy=0.0 m/s，持续约 3 秒，并通过 State API
 * 读取底盘速度、伺服状态和故障码。SetMaxBaseLinearSpeed() 会修改底盘
 * 当前模型配置，实际使用时应根据设备安全范围设置。
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

std::atomic<bool> g_running{true};

// Ctrl-C 后结束速度发送循环，主流程负责停止底盘并关闭 SDK。
void Stop(int) { g_running.store(false); }

void PrintServoState(const std::vector<modi_sdk::ServoState>& states) {
  std::cout << "  servo_state:";
  for (const auto state : states) {
    std::cout << ' ' << static_cast<int>(state);
  }
  std::cout << "\n";
}

void PrintFsmStates(const std::shared_ptr<modi_sdk::ChassisManagerApi>& manager) {
  int safety_error = modi_sdk::ToInt(modi_sdk::SdkError::kOk);
  int chassis_error = modi_sdk::ToInt(modi_sdk::SdkError::kOk);
  const auto safety_state = manager->GetSafetyState(&safety_error);
  const auto chassis_state = manager->GetChassisState(&chassis_error);
  std::cout << "  safety_fsm=" << static_cast<int>(safety_state)
            << " error=" << safety_error
            << " (0=Normal,1=Estop,2=Fault,3=Recovery)\n";
  std::cout << "  chassis_fsm=" << static_cast<int>(chassis_state)
            << " error=" << chassis_error
            << " (0=Init,1=PowerOff,2=PowerOn,3=Running)\n";
}
}  // namespace

int main() {
  // 注册信号处理，允许用户提前结束速度测试。
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取模型、管理、运动和状态接口。
  auto chassis = client.Chassis();
  auto model = chassis->Model();
  auto manager = chassis->Manager();
  auto motion = chassis->Motion();
  auto state = chassis->State();

  // 3. 清除可恢复故障，并确认安全状态正常。
  const int ret = manager->ClearFaults();
  std::cout << "ClearFaults ret=" << ret << "\n";
  if (ret == modi_sdk::ToInt(modi_sdk::SdkError::kServiceTimeout)) {
    std::cerr << "Chassis service timed out. Start modi_chassis_node and "
                 "verify that the SDK and server use the same DDS domain.\n";
    client.Stop();
    return 2;
  }
  if (ret != 0) {
    std::cerr << "ClearFaults was rejected; check modi_chassis_node logs.\n";
    PrintFsmStates(manager);
    PrintServoState(state->GetServoState());
    client.Stop();
    return 1;
  }
  PrintFsmStates(manager);
  PrintServoState(state->GetServoState());
  int safety_error = modi_sdk::ToInt(modi_sdk::SdkError::kOk);
  const auto safety_state = manager->GetSafetyState(&safety_error);
  if (safety_error != modi_sdk::ToInt(modi_sdk::SdkError::kOk) ||
      safety_state != modi_sdk::SafetyState::kNormal) {
    std::cerr << "Safety is not Normal after ClearFaults (state="
              << static_cast<int>(safety_state) << ", error=" << safety_error
              << "). Refusing to power on; inspect the reported servo states.\n";
    client.Stop();
    return 1;
  }

  // 4. 调用 PowerOn() 给底盘上电，成功后才能发送运动指令。
  const int power_on_result = manager->PowerOn();
  std::cout << "PowerOn: " << power_on_result << '\n';
  if (power_on_result != 0) {
    client.Stop();
    return 1;
  }

  // 5. 准备 SpeedTranslate() 的目标速度和发送周期。
  constexpr double kVx = 0.2;  // m/s
  constexpr double kVy = 0.0;  // m/s
  constexpr auto kDuration = std::chrono::seconds(3);
  constexpr auto kCommandPeriod = std::chrono::milliseconds(100);

  // 修改底盘线速度上限，成功后 SpeedTranslate() 使用该配置。
  const int set_speed_result = model->SetMaxBaseLinearSpeed(kVx);
  std::cout << "SetMaxBaseLinearSpeed(" << kVx
            << "): " << set_speed_result << '\n';
  if (set_speed_result != 0) {
    (void)motion->StopMotion();
    (void)manager->PowerOff();
    client.Stop();
    return 1;
  }

  // 6. 周期性调用 SpeedTranslate(vx, vy) 发送平移速度。
  const auto deadline = std::chrono::steady_clock::now() + kDuration;
  while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
    // 速度控制期间需要持续发送目标速度。
    const int result = motion->SpeedTranslate(kVx, kVy);
    if (result != 0) {
      std::cerr << "SpeedTranslate() 失败：" << result << '\n';
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(kCommandPeriod);
  }

  // 7. 通过 State API 读取运动后的底盘实际速度。
  const auto base_velocity = state->GetBaseVelocity();
  std::cout << "BaseVelocity: " << base_velocity.linear_x << ", "
            << base_velocity.linear_y << ", " << base_velocity.angular_z
            << '\n';

  // 8. 停止运动、底盘下电并关闭 SDK 客户端。
  std::cout << "StopMotion: " << motion->StopMotion() << '\n';
  std::cout << "PowerOff: " << manager->PowerOff() << '\n';
  client.Stop();
  return 0;
}
