#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"

namespace {

std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running.store(false); }

int WaitMotionFinish(const std::shared_ptr<modi_sdk::MotionApi>& motion,
                     int wait_start_timeout_ms = 10000) {
  using PlannerStatus = modi_sdk::PlannerStatus;
  if (!motion) return static_cast<int>(PlannerStatus::kError);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(wait_start_timeout_ms);
  const auto expired = [&] {
    return std::chrono::steady_clock::now() >= deadline;
  };
  const auto status = [&] {
    return motion->GetPlannerStatus();
  };
  constexpr auto kSleepPeriod = std::chrono::milliseconds(1);

  if (status() == PlannerStatus::kError) {
    return static_cast<int>(PlannerStatus::kError);
  }

  while (status() != PlannerStatus::kPlanning && !expired() &&
         g_running.load()) {
    if (status() == PlannerStatus::kFinished) {
      return static_cast<int>(PlannerStatus::kFinished);
    }
    std::this_thread::sleep_for(kSleepPeriod);
  }
  if (!g_running.load() || expired()) return -1;

  while (status() == PlannerStatus::kPlanning && g_running.load()) {
    std::this_thread::sleep_for(kSleepPeriod);
  }
  if (!g_running.load()) return -1;
  return status() == PlannerStatus::kError
             ? static_cast<int>(PlannerStatus::kError)
             : static_cast<int>(motion->GetPlannerStatus());
}

bool RunMotion(const std::shared_ptr<modi_sdk::MotionApi>& motion, int result,
               const char* name) {
  std::cout << name << " ret=" << result << std::endl;
  if (result != 0) return false;

  const int planner_status = WaitMotionFinish(motion);
  std::cout << name << " planner status=" << planner_status << std::endl;
  return planner_status == static_cast<int>(modi_sdk::PlannerStatus::kFinished);
}

}  // namespace

int main(int argc, char* argv[]) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config config;
  config.domain_id = 22;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
    return 2;
  }
  config.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(config);
  if (!client->Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  auto state = client->Robot()->State();
  auto motion = client->Robot()->Motion();

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDeg2Rad = kPi / 180.0;
  constexpr double kArmAngleAmplitude = 30.0 * kDeg2Rad;
  constexpr double kJointVelocity = 0.15;
  constexpr double kJointAcceleration = 0.2;
  constexpr double kLineVelocity = 0.02;
  constexpr double kLineAcceleration = 0.05;
  constexpr double kArmAngleVelocity = 0.2;
  constexpr double kArmAngleAcceleration = 0.4;

  const std::vector<double> initial_joint_positions = {
      0.0, -30.0 * kDeg2Rad, 0.0, -90.0 * kDeg2Rad, 0.0, 30.0 * kDeg2Rad,  0.0,
      0.0, 30.0 * kDeg2Rad,  0.0, 90.0 * kDeg2Rad,  0.0, -30.0 * kDeg2Rad, 0.0,
  };

  const auto current_joint_positions = state->GetActuatorJointPositions();
  if (current_joint_positions.size() != initial_joint_positions.size()) {
    std::cerr << "Expected " << initial_joint_positions.size()
              << " dual-arm joints, got " << current_joint_positions.size()
              << "\n";
    client->Stop();
    return 1;
  }

  std::cout << "Move both arms to the initial joint positions\n";
  if (!RunMotion(motion,
                 motion->MoveJoint(initial_joint_positions, kJointVelocity,
                                   kJointAcceleration, false),
                 "MoveJoint")) {
    client->Stop();
    return 1;
  }

  const auto center_arm_angles = state->GetCurrentArmAngles();
  if (center_arm_angles.size() != 2) {
    std::cerr << "Expected two current arm angles, got "
              << center_arm_angles.size() << "\n";
    client->Stop();
    return 1;
  }

  const auto fixed_target_poses = state->GetEndEffectorPoses();
  if (fixed_target_poses.size() != 2 || fixed_target_poses[0].size() < 6 ||
      fixed_target_poses[1].size() < 6) {
    std::cerr << "Expected two valid end-effector poses after MoveJoint\n";
    client->Stop();
    return 1;
  }

  modi_sdk::ArmAngleOptions arm_angle_options;
  arm_angle_options.arm_angle_directions = {
      modi_sdk::ArmAngleDirection::kShortest,
      modi_sdk::ArmAngleDirection::kShortest};
  arm_angle_options.arm_angle_velocities = {kArmAngleVelocity,
                                            kArmAngleVelocity};
  arm_angle_options.arm_angle_accelerations = {kArmAngleAcceleration,
                                               kArmAngleAcceleration};

  bool move_to_positive_limit = true;
  while (g_running.load()) {
    const double offset =
        move_to_positive_limit ? kArmAngleAmplitude : -kArmAngleAmplitude;
    arm_angle_options.arm_angles = {center_arm_angles[0] + offset,
                                    center_arm_angles[1] - offset};

    std::cout << "Keep both end effectors fixed and move both arm angles "
                 "30 deg from the initial measured arm angles, direction="
              << (move_to_positive_limit ? "+" : "-") << "\n";
    if (!RunMotion(
            motion,
            motion->MoveLine(fixed_target_poses, kLineVelocity,
                             kLineAcceleration, arm_angle_options, false),
            "MoveLine")) {
      if (g_running.load()) {
        std::cerr << "Arm-angle motion failed\n";
      }
      break;
    }
    move_to_positive_limit = !move_to_positive_limit;
  }

  client->Stop();
  return g_running.load() ? 1 : 0;
}
