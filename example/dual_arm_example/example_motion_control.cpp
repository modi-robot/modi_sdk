// 双臂循环执行预设关节运动、镜像 Y 向直线往返和镜像 YZ 平面圆周。
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "modi_sdk/type_def.hpp"
#include "modi_sdk/robot_client.hpp"

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running.store(false); }

// Move* 之后：在 wait_start_timeout_ms 内等到状态离开入口快照并进入本轮
//（kPlanning / 直接 kFinished）；超时返回 -1（与 kError 同值）。之后在
// kPlanning 上无超时等到结束。
int WaitMotionFinish(const std::shared_ptr<modi_sdk::MotionApi>& motion,
                     int wait_start_timeout_ms = 10000) {
  using PS = modi_sdk::PlannerStatus;
  if (!motion) return static_cast<int>(PS::kError);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(wait_start_timeout_ms > 0
                                                      ? wait_start_timeout_ms
                                                      : 10000);
  auto expired = [&] { return std::chrono::steady_clock::now() >= deadline; };
  auto raw = [&] { return motion->GetPlannerStatus(); };
  auto st = [&] { return static_cast<PS>(raw()); };
  constexpr auto sl = std::chrono::milliseconds(1);

  const int s0 = raw();
  if (static_cast<PS>(s0) == PS::kError) return static_cast<int>(PS::kError);
  if (static_cast<PS>(s0) != PS::kPlanning) {
    while (g_running.load() && !expired() && raw() == s0) {
      std::this_thread::sleep_for(sl);
    }
  }
  if (!g_running.load() || expired()) return -1;

  bool started = false;
  while (g_running.load() && !expired() && !started) {
    const PS s = st();
    if (s == PS::kError) return static_cast<int>(PS::kError);
    if (s == PS::kPlanning)
      started = true;
    else if (s == PS::kFinished)
      return static_cast<int>(PS::kFinished);
    else
      std::this_thread::sleep_for(sl);
  }
  if (!started) return -1;

  while (g_running.load() && st() == PS::kPlanning) {
    std::this_thread::sleep_for(sl);
  }
  if (!g_running.load()) return -1;
  return st() == PS::kError ? static_cast<int>(PS::kError) : raw();
}

void BuildYzPlaneCircleVia(const std::vector<double>& start,
                           double y_direction, double radius_m,
                           std::vector<double>& via1,
                           std::vector<double>& via2) {
  if (start.size() < 3) {
    via1 = via2 = start;
    return;
  }
  constexpr double k120 = 2.0 * M_PI / 3.0;
  constexpr double k240 = 4.0 * M_PI / 3.0;
  const double cy = start[1] - y_direction * radius_m;
  const double cz = start[2];
  via1 = start;
  via2 = start;
  via1[1] = cy + y_direction * radius_m * std::cos(k120);
  via1[2] = cz + radius_m * std::sin(k120);
  via2[1] = cy + y_direction * radius_m * std::cos(k240);
  via2[2] = cz + radius_m * std::sin(k240);
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

  auto state = client->Robot()->State();
  auto motion = client->Robot()->Motion();

  constexpr double kDeg2Rad = M_PI / 180.0;
  const std::vector<double> preset_joints = {
      0.0 * kDeg2Rad,  -30.0 * kDeg2Rad, 0.0,
      -90.0 * kDeg2Rad, 0.0,             30.0 * kDeg2Rad,
      0.0,
      0.0 * kDeg2Rad, 30.0 * kDeg2Rad, 0.0,
      90.0 * kDeg2Rad, 0.0,            -30.0 * kDeg2Rad,
      0.0,
  };

  const auto current_joints = state->GetActuatorJointPositions();
  if (current_joints.size() != preset_joints.size()) {
    std::cerr << "Expected " << preset_joints.size()
              << " dual-arm joints, got " << current_joints.size() << "\n";
    client->Stop();
    return 1;
  }

  while (g_running.load()) {
    std::cout << "[1] MoveJoint both arms to preset pose\n";
    int ret = motion->MoveJoint(preset_joints, 0.2, 0.2);
    std::cout << "MoveJoint ret=" << ret << std::endl;
    if (ret != 0 || WaitMotionFinish(motion) < 0) break;

    auto base_poses = state->GetEndEffectorPoses();
    if (base_poses.size() < 2 || base_poses[0].size() < 3 ||
        base_poses[1].size() < 3) {
      std::cerr << "Expected two valid end-effector poses\n";
      break;
    }
    const double left_base_y = base_poses[0][1];
    const double right_base_y = base_poses[1][1];

    if (g_running.load()) {
      std::cout << "[2] MoveLine both arms Y -0.1m\n";
      auto target_poses = base_poses;
      target_poses[0][1] = left_base_y + 0.1;
      target_poses[1][1] = right_base_y - 0.1;
      ret = motion->MoveLine(target_poses, 0.05, 0.1);
      std::cout << "MoveLine ret=" << ret << std::endl;
      if (ret != 0 || WaitMotionFinish(motion) < 0) break;
    }

    if (g_running.load()) {
      std::cout << "[3] MoveLine both arms Y +0.1m back to origin\n";
      auto target_poses = state->GetEndEffectorPoses();
      if (target_poses.size() < 2 || target_poses[0].size() < 3 ||
          target_poses[1].size() < 3) {
        std::cerr << "Expected two valid end-effector poses\n";
        break;
      }
      target_poses[0][1] = left_base_y;
      target_poses[1][1] = right_base_y;
      ret = motion->MoveLine(target_poses, 0.05, 0.1);
      std::cout << "MoveLine ret=" << ret << std::endl;
      if (ret != 0 || WaitMotionFinish(motion) < 0) break;
    }

    if (g_running.load()) {
      std::cout << "[4] MoveCircle both arms in YZ plane\n";
      auto circle_poses = state->GetEndEffectorPoses();
      if (circle_poses.size() < 2 || circle_poses[0].size() < 3 ||
          circle_poses[1].size() < 3) {
        std::cerr << "Expected two valid end-effector poses\n";
        break;
      }
      std::vector<std::vector<double>> via1(2), via2(2);
      BuildYzPlaneCircleVia(circle_poses[0], 1.0, 0.05, via1[0], via2[0]);
      BuildYzPlaneCircleVia(circle_poses[1], -1.0, 0.05, via1[1], via2[1]);
      ret = motion->MoveCircle(via1, via2,
                               {2.0 * M_PI, -2.0 * M_PI}, 0.05, 0.1);
      std::cout << "MoveCircle ret=" << ret << std::endl;
      if (ret != 0 || WaitMotionFinish(motion) < 0) break;
    }
  }

  client->Stop();
  return 0;
}
