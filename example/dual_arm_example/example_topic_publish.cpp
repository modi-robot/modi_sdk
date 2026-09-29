#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

// DDS 生成的 Pose / PoseArray（Publisher::Mutable<PoseArray>(...)
// 在本文件实例化，需完整类型）。
#include "modi_sdk/robot_client.hpp"
#include "modi_type_def.h"

// example_topic_publish
// ----------------------
// 双臂场景：RegisterPublisher + RT 回调里 Publisher::Mutable 发布
// /Robot/Model/SetFbasePose（PoseArray）。 SetFbasePose 的 PoseArray 仍按「整机 /
// 多机」维度（每台机器人一个浮动基位姿），不是按单臂拆分。 单台时 poses 长度为
// 1；多台时须与 modi_config.yaml::robot_name 数量一致。

namespace {

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running.store(false); }

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

  std::atomic<int> seq{0};
  const int pub_ret = client->RegisterPublisher(
      {"/Robot/Model/SetFbasePose"}, [&seq](modi_sdk::Publisher& pub) {
        constexpr double kAmp = 1.57;
        constexpr int kPeriod = 5000;
        const int s = seq.fetch_add(1, std::memory_order_relaxed);
        const double t =
            static_cast<double>(s % kPeriod) / static_cast<double>(kPeriod);
        const double tri = 1.0 - std::abs(2.0 * t - 1.0);
        Pose pose0;
        pose0.data({0.0, 0.0, 0.0, kAmp * tri, 0.0, 0.0});
        Pose pose1;
        pose1.data({0.0, 0.0, 0.0, -kAmp * tri, 0.0, 0.0});
        PoseArray pose_array;
        pose_array.poses({std::move(pose0), std::move(pose1)});
        (void)pub.Mutable<PoseArray>("/Robot/Model/SetFbasePose", pose_array);
      });
  if (pub_ret != 0) {
    std::cerr
        << "RegisterPublisher failed (topic not in SDK publish registry?)\n";
    return 1;
  }

  if (!client->Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  std::cerr << "Publishing /Robot/Model/SetFbasePose (roll 0<->1.57 and 0<->-1.57). "
               "Ctrl-C to stop.\n";
  while (g_running.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  client->Stop();
  std::cerr << "Done.\n";
  return 0;
}
