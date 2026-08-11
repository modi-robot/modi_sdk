#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "modi_sdk/type_def.hpp"
#include "modi_sdk/robot_client.hpp"

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running.store(false); }

template <typename Seq>
void PrintSeq(const std::string& label, const Seq& seq) {
  std::cout << label << ": ";
  const int n = static_cast<int>(seq.size());
  for (int i = 0; i < n && i < seq.size(); ++i) std::cout << seq[i] << " ";
  std::cout << "\n";
}

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
  if (static_cast<PS>(s0) != PS::kPlanning)
    while (!expired() && raw() == s0) std::this_thread::sleep_for(sl);
  if (expired()) return -1;

  bool started = false;
  while (!expired() && !started) {
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

  while (st() == PS::kPlanning) std::this_thread::sleep_for(sl);
  return st() == PS::kError ? static_cast<int>(PS::kError) : raw();
}

// YZ 平面整圆（法向 +X）：圆心为起点沿 -Y 偏移 radius_m，途经点为 120°/240°。
void BuildYzPlaneCircleVia(const std::vector<double>& start, double radius_m,
                           std::vector<double>& via1,
                           std::vector<double>& via2) {
  if (start.size() < 6) {
    via1 = via2 = start;
    return;
  }
  constexpr double k120 = 2.0 * M_PI / 3.0;
  constexpr double k240 = 4.0 * M_PI / 3.0;
  const double x = start[0];
  const double cy = start[1] - radius_m;
  const double cz = start[2];
  via1 = start;
  via2 = start;
  via1[1] = cy + radius_m * std::cos(k120);
  via1[2] = cz + radius_m * std::sin(k120);
  via2[1] = cy + radius_m * std::cos(k240);
  via2[2] = cz + radius_m * std::sin(k240);
  via1[0] = via2[0] = x;
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

  auto model = client->Robot()->Model();
  auto state = client->Robot()->State();
  auto motion = client->Robot()->Motion();

  // MovePose
  std::cout << "\n=== MoveJoint ===" << std::endl;
  auto target_joints = state->GetActuatorJointPositions();
  PrintSeq("target_joints", target_joints);
  target_joints[0] -= 1.57;
  int ret = motion->MoveJoint(target_joints, 0.1, 0.1);
  std::cout << "MoveJoint ret=" << ret << std::endl;
  int ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";
  target_joints[0] += 1.57;
  ret = motion->MoveJoint(target_joints, 0.1, 0.1);
  std::cout << "MoveJoint ret=" << ret << std::endl;
  ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";

  std::cout << "\n=== MoveLine ===" << std::endl;
  auto target_poses = state->GetEndEffectorPoses();
  PrintSeq("target_poses", target_poses[0]);
  target_poses[0][2] -= 0.1;
  ret = motion->MoveLine(target_poses, 0.1, 0.1);
  std::cout << "MoveLine ret=" << ret << std::endl;
  ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";
  target_poses[0][2] += 0.1;
  ret = motion->MoveLine(target_poses, 0.1, 0.1);
  std::cout << "MoveLine ret=" << ret << std::endl;
  ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";

  std::cout << "\n=== MoveCircle (YZ plane, R=0.05m, 360 deg) ===" << std::endl;
  target_poses = state->GetEndEffectorPoses();
  PrintSeq("circle start", target_poses[0]);
  constexpr double kCircleRadius = 0.05;
  constexpr double kFullTurn = 2.0 * M_PI;
  std::vector<std::vector<double>> via1, via2;
  std::vector<double> v1, v2;
  BuildYzPlaneCircleVia(target_poses[0], kCircleRadius, v1, v2);
  via1.push_back(std::move(v1));
  via2.push_back(std::move(v2));
  PrintSeq("via_point1", via1[0]);
  PrintSeq("via_point2", via2[0]);
  ret = motion->MoveCircle(via1, via2, {kFullTurn}, 0.1, 0.1);
  std::cout << "MoveCircle ret=" << ret << std::endl;
  ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";

  std::cout << "\n=== MovePose ===" << std::endl;
  target_poses = state->GetEndEffectorPoses();
  PrintSeq("target_poses", target_poses[0]);
  target_poses[0][2] -= 0.1;
  ret = motion->MovePose(target_poses, 0.1, 0.1);
  std::cout << "MovePose ret=" << ret << std::endl;
  ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";
  target_poses[0][2] += 0.1;
  ret = motion->MovePose(target_poses, 0.1, 0.1);
  std::cout << "MovePose ret=" << ret << std::endl;
  ps = WaitMotionFinish(motion);
  std::cout << "Planner finished, status=" << ps
            << " (2=finished, -1=error or wait-start-timeout)\n";

  client->Stop();
  return 0;
}
