#include <atomic>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running.store(false); }

void PrintBaseVelocity(const modi_sdk::ChassisBaseVelocity& vel) {
  std::cout << "  base_vel: linear_x=" << vel.linear_x
            << " linear_y=" << vel.linear_y
            << " angular_z=" << vel.angular_z << "\n";
}

void PrintBasePose(const modi_sdk::ChassisBasePose& pose) {
  std::cout << "  base_pose: x=" << pose.x << " y=" << pose.y
            << " yaw=" << pose.yaw << " valid=" << pose.valid << "\n";
}

void PrintServoState(const std::vector<modi_sdk::ServoState>& states) {
  std::cout << "  servo_state:";
  for (const auto state : states) {
    std::cout << ' ' << static_cast<int>(state);
  }
  std::cout << "\n";
}

void PrintFsmStates(const std::shared_ptr<modi_sdk::ChassisManagerApi>& manager) {
  std::cout << "  safety_fsm=" << manager->GetSafetyState()
            << " (0=Normal,1=Estop,2=Fault,3=Recovery)\n";
  std::cout << "  chassis_fsm=" << manager->GetChassisState()
            << " (0=Init,1=PowerOff,2=PowerOn,3=Running)\n";
}
}  // namespace

int main() {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config cfg;
  auto client = std::make_shared<modi_sdk::RobotClient>(cfg);
  if (!client->Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  auto chassis = client->Chassis();
  auto model = chassis->Model();
  auto manager = chassis->Manager();
  auto motion = chassis->Motion();
  auto state = chassis->State();

  std::cout << "=== Chassis ClearFaults ===\n";
  int ret = manager->ClearFaults();
  std::cout << "ClearFaults ret=" << ret << "\n";
  PrintFsmStates(manager);
  PrintServoState(state->GetServoState());

  std::cout << "=== Chassis PowerOn ===\n";
  ret = manager->PowerOn();
  std::cout << "PowerOn ret=" << ret
            << " (0=ok, -2=timeout, -3=server rejected)\n";
  PrintFsmStates(manager);
  PrintServoState(state->GetServoState());
  if (ret != 0) {
    std::cerr << "PowerOn failed; check modi_chassis_node log "
                 "(grep 'PowerOn\\|PollFaults')\n";
    client->Stop();
    return 1;
  }

  constexpr double kVx = 0.5;  // m/s
  constexpr double kVy = 0.0;
  constexpr auto kDuration = std::chrono::seconds(3);
  constexpr auto kCmdPeriod = std::chrono::milliseconds(100);

  std::cout << "\n=== SetMaxBaseLinearSpeed " << kVx << " m/s ===\n";
  ret = model->SetMaxBaseLinearSpeed(kVx);
  std::cout << "SetMaxBaseLinearSpeed ret=" << ret
            << " current=" << model->GetMaxBaseLinearSpeed() << " m/s\n";
  if (ret != 0) {
    std::cerr << "SetMaxBaseLinearSpeed failed; stopping\n";
    client->Stop();
    return 1;
  }

  std::cout << "\n=== SpeedTranslate vx=" << kVx << " vy=" << kVy << " ===\n";
  const auto deadline = std::chrono::steady_clock::now() + kDuration;
  
  ret = motion->SpeedTranslate(kVx, kVy);
  std::cout << "SpeedTranslate ret=" << ret << "\n";
  PrintFsmStates(manager);
  PrintServoState(state->GetServoState());
  if (ret != 0) {
    std::cerr << "SpeedTranslate failed; stopping\n";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  PrintBaseVelocity(state->GetBaseVelocity());
  std::cout << "\n=== SpeedTranslate vx=0 vy=0 (stop) ===\n";
  ret = motion->SpeedTranslate(0.0, 0.0);
  std::cout << "SpeedTranslate(stop) ret=" << ret << "\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  PrintBaseVelocity(state->GetBaseVelocity());
  PrintBasePose(state->GetBasePose());

  std::cout << "\n=== Chassis PowerOff ===\n";
  ret = manager->PowerOff();
  std::cout << "PowerOff ret=" << ret << "\n";

  client->Stop();
  std::cout << "Done.\n";
  return 0;
}
