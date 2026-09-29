#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "modi_sdk/type_def.hpp"
#include "modi_sdk/robot_client.hpp"
#include "modi_type_def.h"

// example_rt_topic
// ----------------
// 演示 modi_sdk 的实时 topic 订阅：
//   - RegisterSubscriber 一次注册多个 topic
//   - 回调运行在 SDK spin 线程（spin_frequency_hz）
//   - 主线程低频读取最新快照并打印

namespace {

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running.store(false); }

// 回调与主线程共享的快照，互斥保护。
struct Snapshot {
  std::mutex mu;
  JointPositions jpos;
  JointVelocities jvel;
  JointTorques jtor;
  PoseArray ee_poses;
  WrenchArray ee_wrenches;
  modi_sdk::PlannerStatus planner = modi_sdk::PlannerStatus::kIdle;
};

// 通用序列打印：seq 需支持 size() 与 operator[]，元素需可流输出。
template <typename Seq>
void PrintSeq(const std::string& label, const Seq& seq) {
  std::cout << label << ": ";
  const int n = static_cast<int>(seq.size());
  for (int i = 0; i < n && i < seq.size(); ++i) std::cout << seq[i] << " ";
  std::cout << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config cfg;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
    return 2;
  }
  cfg.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(cfg);
  Snapshot snap;

  const std::vector<std::string> topics = {
      "/Robot/State/JointPositions",      "/Robot/State/JointVelocities",
      "/Robot/State/JointTorques",        "/Robot/State/EndEffectorPoses",
      "/Robot/State/EndEffectorWrenches", "/Robot/Motion/PlannerStatus",
  };

  // RT 回调：从 parser 取最新值写入快照。
  const int ret =
      client->RegisterSubscriber(topics, [&snap](const modi_sdk::Parser& p) {
        std::lock_guard<std::mutex> lk(snap.mu);
        snap.jpos = p.Value<JointPositions>("/Robot/State/JointPositions");
        snap.jvel = p.Value<JointVelocities>("/Robot/State/JointVelocities");
        snap.jtor = p.Value<JointTorques>("/Robot/State/JointTorques");
        snap.ee_poses = p.Value<PoseArray>("/Robot/State/EndEffectorPoses");
        snap.ee_wrenches = p.Value<WrenchArray>("/Robot/State/EndEffectorWrenches");
        snap.planner = static_cast<modi_sdk::PlannerStatus>(
            p.Value<PlannerStatus>("/Robot/Motion/PlannerStatus").status());
      });
  if (ret < 0) {
    std::cerr << "RegisterSubscriber failed" << std::endl;
    return 1;
  }

  if (!client->Start()) {
    std::cerr << "RobotClient Start failed" << std::endl;
    return 1;
  }

  auto motion = client->Robot()->Motion();
  // 在 IDLE / FINISHED 时来回切换的两个关节目标。
  const std::vector<std::vector<double>> targets = {
      std::vector<double>(7, 0.0),
      std::vector<double>(7, 1.0),
  };
  size_t target_idx = 0;

  auto next = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(500);
  while (g_running.load()) {
    std::this_thread::sleep_until(next);
    next += period;

    // 在锁内拷贝出本周期所需字段，避免持锁打印。
    Snapshot s;
    {
      std::lock_guard<std::mutex> lk(snap.mu);
      s.jpos = snap.jpos;
      s.jvel = snap.jvel;
      s.jtor = snap.jtor;
      s.ee_poses = snap.ee_poses;
      s.ee_wrenches = snap.ee_wrenches;
      s.planner = snap.planner;
    }

    std::cout << "---\n";
    PrintSeq("jpos", s.jpos.joint_positions());
    PrintSeq("jvel", s.jvel.joint_velocities());
    PrintSeq("jtorque", s.jtor.joint_torques());
    for (size_t i = 0; i < s.ee_poses.poses().size(); ++i) {
      PrintSeq("ee[" + std::to_string(i) + "] xyzrpy",
               s.ee_poses.poses()[i].data());
    }
    for (size_t i = 0; i < s.ee_wrenches.wrenches().size(); ++i) {
      PrintSeq("ee[" + std::to_string(i) + "] wrench",
               s.ee_wrenches.wrenches()[i].data());
    }
    std::cout << "planner_status: " << static_cast<int>(s.planner) << "\n";

    // IDLE / FINISHED：发起下一段 MoveJoint，目标在两个姿态间切换。
    if (s.planner == modi_sdk::PlannerStatus::kIdle ||
        s.planner == modi_sdk::PlannerStatus::kFinished) {
      const auto& target = targets[target_idx];
      target_idx = (target_idx + 1) % targets.size();
      motion->MoveJoint(target, 0.01, 0.1);
    }
  }

  client->Stop();
  std::cout << "Done." << std::endl;
  return 0;
}
