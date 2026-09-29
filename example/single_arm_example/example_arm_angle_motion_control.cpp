#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

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

  // A short trajectory can finish before a polling cycle observes PLANNING.
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
}  // namespace

int main(int argc, char* argv[]) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config config;
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
  auto target_poses = state->GetEndEffectorPoses();
  if (target_poses.size() != 1 || target_poses[0].size() < 3) {
    std::cerr << "Expected one valid end-effector pose\n";
    client->Stop();
    return 1;
  }

  // The arm angle is an absolute target in radians. All option arrays have
  // one entry because this example controls one WBC.
  modi_sdk::ArmAngleOptions arm_angle_options;
  arm_angle_options.arm_angles = {0.5};
  arm_angle_options.arm_angle_directions = {
      modi_sdk::ArmAngleDirection::kShortest};
  arm_angle_options.arm_angle_velocities = {0.4};
  arm_angle_options.arm_angle_accelerations = {0.8};

  // The end-effector and arm-angle trajectories are planned together and
  // finish in the same control cycle.
  // target_poses[0][2] -= 0.05;
  // std::cout << "MoveLine with absolute arm angle: target_psi=0.5 rad, "
  //              "z_offset=-0.05 m\n";
  const int ret = motion->MoveLine(target_poses, 0.05, 0.1,
                                   arm_angle_options, false);
  std::cout << "MoveLine ret=" << ret << std::endl;
  if (ret == 0) {
    const int planner_status = WaitMotionFinish(motion);
    std::cout << "Planner finished, status=" << planner_status
              << " (2=finished, -1=error or wait-start-timeout)\n";
  }

  client->Stop();
  return ret == 0 ? 0 : 1;
}
