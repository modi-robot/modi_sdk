// example_set_installation_position
// ---------------------
// 单臂场景：浮动基位姿（需已运行 modi_system 等带 WBC 的节点）。
//   StateApi::GetFbasePosition -> 改写位姿 ->
//   ModelApi::SetInstallationPosition（DDS
//   服务）。
// 单台机器人时 poses 长度为 1；若配置多台 robot_name，poses 个数须与之一致。

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

// Move* 之后：在 wait_start_timeout_ms 内等到出现 kPlanning（新轨迹已被接受）。
// 若轨迹极短，可能采不到 PLANNING 而一直为 kFinished(2)，不能用「等 raw!=s0」
//（旧、新均为 2 会死等到超时）。超时若仍为 kFinished 则视为已结束。
// 之后在 kPlanning 上无超时等到结束。
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

  if (st() == PS::kError) return static_cast<int>(PS::kError);

  if (st() != PS::kPlanning) {
    bool saw_planning = false;
    while (!expired()) {
      const PS s = st();
      if (s == PS::kError) return static_cast<int>(PS::kError);
      if (s == PS::kPlanning) {
        saw_planning = true;
        break;
      }
      std::this_thread::sleep_for(sl);
    }
    if (!saw_planning) {
      if (st() == PS::kFinished) return static_cast<int>(PS::kFinished);
      return -1;
    }
  }

  while (st() == PS::kPlanning) std::this_thread::sleep_for(sl);
  return st() == PS::kError ? static_cast<int>(PS::kError) : raw();
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
  auto init_end_effector_poses = state->GetEndEffectorPoses();
  PrintSeq("init_end_effector_poses", init_end_effector_poses[0]);
  // --- 1) 服务：初始化为绕 X 轴 90°（roll = π/2，xyz = 0）---
  auto init_poses = state->GetFbasePosition();
  init_poses[0][2] = 0.5;
  init_poses[0][3] = 90.0 * M_PI / 180.0;
  int ret = model->SetInstallationPosition(init_poses);
  if (ret != 0) {
    std::cerr << "SetInstallationPosition (service) failed, ret=" << ret
              << " (check modi_system & robot count)\n";
    client->Stop();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  for (auto& pose : init_poses) {
    PrintSeq("base pose", pose);
  }

  // MovePose
  std::cout << "\n=== MoveLine ===" << std::endl;
  while (true) {
    auto target_poses = state->GetEndEffectorPoses();
    PrintSeq("target_poses", target_poses[0]);
    target_poses[0][2] -= 0.1;
    ret = motion->MoveLine(target_poses, 0.1, 0.1);
    std::cout << "MoveLine ret=" << ret << std::endl;
    int ps = WaitMotionFinish(motion);
    std::cout << "Planner finished, status=" << ps
              << " (2=finished, -1=error or wait-start-timeout)\n";
    target_poses[0][2] += 0.1;
    ret = motion->MoveLine(target_poses, 0.1, 0.1);
    std::cout << "MoveLine ret=" << ret << std::endl;
    ps = WaitMotionFinish(motion);
    std::cout << "Planner finished, status=" << ps
              << " (2=finished, -1=error or wait-start-timeout)\n";
  }
  client->Stop();
  return 0;
}
