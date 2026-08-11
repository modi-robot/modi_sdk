#include <atomic>
#include <chrono>
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
// 单臂场景：RegisterPublisher + RT 回调里 Publisher::Mutable 发布
// /Robot/Model/SetFbasePose（PoseArray）。 单台时 poses 长度为 1；多台时须与
// modi_config.yaml::robot_name 数量一致。

namespace {

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running.store(false); }

}  // namespace

int main() {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config cfg;
  auto client = std::make_shared<modi_sdk::RobotClient>(cfg);

  std::atomic<int> seq{0};
  const int pub_ret = client->RegisterPublisher(
      {"/Robot/Model/SetFbasePose"}, [&seq](modi_sdk::Publisher& pub) {
        Pose pose;
        const int s = seq.fetch_add(1, std::memory_order_relaxed);
        const double roll = 0.001 * static_cast<double>(s % 100000);
        pose.data({0.0, 0.0, 0.0, roll, 0.0, 0.0});
        PoseArray pose_array;
        pose_array.poses({std::move(pose)});
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

  std::cerr << "Publishing /Robot/Model/SetFbasePose via RobotClient (roll ramps). "
               "Ctrl-C to stop.\n";
  while (g_running.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  client->Stop();
  std::cerr << "Done.\n";
  return 0;
}
