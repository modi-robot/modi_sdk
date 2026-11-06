/**
 * @brief 演示通过 MODI SDK 读取底盘八路超声波雷达最新距离。
 *
 * 调用流程与其他底盘状态示例一致：创建并启动 RobotClient，通过
 * client.Chassis()->State() 获取状态接口，再调用
 * GetUltrasonicRadarData() 读取八路最新缓存。
 * 通过首个命令行参数指定 chassis_id，DDS Domain ID 使用默认值 0。
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

std::atomic<bool> g_running{true};

void Stop(int) { g_running.store(false); }

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || argv[1][0] == '\0') {
    std::cerr << "Usage: " << argv[0] << " <chassis_id>\n";
    return 2;
  }

  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  auto state = client.Chassis()->State();
  std::cout << "读取底盘超声波雷达（单位 m），按 Ctrl-C 退出\n";

  while (g_running.load()) {
    const auto data = state->GetUltrasonicRadarData();
    std::cout << std::fixed << std::setprecision(3)
              << "distance_1=" << data.distance_1
              << " distance_2=" << data.distance_2
              << " distance_3=" << data.distance_3
              << " distance_4=" << data.distance_4
              << " distance_5=" << data.distance_5
              << " distance_6=" << data.distance_6
              << " distance_7=" << data.distance_7
              << " distance_8=" << data.distance_8
              << "\n---\n"
              << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  client.Stop();
  return 0;
}
