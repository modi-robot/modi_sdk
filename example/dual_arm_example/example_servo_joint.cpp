// 双臂 MoveJoint 回零后，通过 DDS 话题执行全关节 ServoJoint 往复运动。
// 周期 5 ms，每周期步进 0.01 度，范围 [-30, 30] 度，端点保持一周期。
// 按 GetActuatorJointNames() 顺序控制双臂全部被控关节。
// 运行前手动上电并退出拖动；Ctrl+C 停止运动，保持现有电源状态。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"
#include "modi_type_def.h"

namespace {

volatile std::sig_atomic_t g_running = 1;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kPeriod = 0.005;
constexpr double kStep = 0.01 * kDegToRad;
constexpr int kLimitSteps = 3000;
constexpr double kLimit = kLimitSteps * kStep;
constexpr const char* kTopic = "/Robot/Motion/ServoJoint";

void HandleSignal(int) { g_running = 0; }

bool Ready(const std::shared_ptr<modi_sdk::ManagerApi>& manager) {
  return manager->GetSafetyState() == modi_sdk::SafetyState::kNormal &&
         manager->GetRobotState() == modi_sdk::RobotState::kRunning;
}

void Check(bool success, const std::string& message) {
  if (!success) throw std::runtime_error(message);
}

template <typename Condition>
void WaitFor(const std::shared_ptr<modi_sdk::RobotApi>& robot, int seconds,
             Condition done) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(seconds);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    Check(Ready(robot->Manager()), "Robot safety/power state changed");
    const auto status = robot->Motion()->GetPlannerStatus();
    Check(status != modi_sdk::PlannerStatus::kError, "Planner status error");
    if (done(status)) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error(g_running ? "Motion state wait timed out" : "Interrupted");
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
    return 2;
  }
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  config.spin_frequency_hz = 200.0;
  auto client = std::make_shared<modi_sdk::RobotClient>(config);
  // 独立发布客户端：回零后启动，退出时先停发布，再通过服务客户端 StopMotion。
  auto servo_client = std::make_shared<modi_sdk::RobotClient>(config);
  const auto robot = client->Robot();
  std::atomic<int> publish_error{0};
  std::atomic<bool> servo_active{false};
  bool motion_requested = false;
  int exit_code = 0;
  try {
    Check(client->Start(), "RobotClient::Start failed");
    const auto names = robot->Model()->GetActuatorJointNames();
    const auto modes = robot->State()->GetControlMode();
    Check(!names.empty() && Ready(robot->Manager()) && !modes.empty() &&
          std::all_of(modes.begin(), modes.end(), [](modi_sdk::ControlMode mode) {
            return mode == modi_sdk::ControlMode::kPositionControl;
          }), "Requires Safety=Normal, Robot=Running and all arms in position control");
    std::vector<double> lower, upper;
    Check(robot->Model()->GetActuatorJointLimits(lower, upper) == 0 &&
          lower.size() == names.size() && upper.size() == names.size(),
          "Joint limits are unavailable");
    for (std::size_t i = 0; i < names.size(); ++i)
      Check(std::isfinite(lower[i]) && std::isfinite(upper[i]) &&
            lower[i] <= -kLimit && upper[i] >= kLimit,
            names[i] + " does not support the full [-30, 30] degree range");

    Check(g_running, "Interrupted");
    std::vector<double> zero(names.size(), 0.0);
    std::cout << "MoveJoint to zero: " << names.size() << " joints\n";
    motion_requested = true;
    Check(robot->Motion()->MoveJoint(zero, 0.1, 0.5) == 0, "MoveJoint failed");
    WaitFor(robot, 120, [&](modi_sdk::PlannerStatus status) {
      if (status != modi_sdk::PlannerStatus::kFinished) return false;
      const auto q = robot->State()->GetActuatorJointPositions();
      return q.size() == zero.size() && std::all_of(q.begin(), q.end(), [](double v) {
        return std::isfinite(v) && std::abs(v) <= 0.1 * kDegToRad;
      });
    });

    ServoJointCommand command;
    command.period(kPeriod);
    command.target_joints(zero);
    Check(servo_client->RegisterPublisher(
        {kTopic}, [&, command, steps = 0, direction = 1](modi_sdk::Publisher& pub) mutable {
          std::fill(command.target_joints().begin(), command.target_joints().end(), steps * kStep);
          publish_error = pub.Mutable<ServoJointCommand>(kTopic, command);
          if (!servo_active) return;  // DDS 就绪前保持零目标。
          if (steps == direction * kLimitSteps) direction = -direction;
          else steps += direction;  // 整数步进；端点保持一周期后反向。
        }) == 0 && servo_client->Start(), "Servo publisher initialization failed");
    WaitFor(robot, 10, [](modi_sdk::PlannerStatus status) {
      return status == modi_sdk::PlannerStatus::kPlanning;
    });
    servo_active = true;
    std::cout << "ServoJoint: 5 ms, 0.01 deg/step (2 deg/s), "
                 "0 -> +30 -> -30 -> +30 ...; Ctrl+C to stop\n";
    while (g_running) {
      Check(publish_error == 0 && Ready(robot->Manager()), "Servo publish or safety/power error");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& error) {
    if (g_running) {
      std::cerr << error.what() << '\n';
      exit_code = 1;
    }
  }
  servo_client->Stop();
  if (motion_requested && robot->Motion()->StopMotion() != 0) {
    std::cerr << "StopMotion failed\n";
    exit_code = 1;
  }
  client->Stop();
  return exit_code;
}
