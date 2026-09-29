/**
 * @brief 演示通过 MODI SDK 启动和停止建图或定位。
 *
 * 调用流程：创建并启动 RobotClient，通过 client.Chassis()->Slam() 获取
 * Slam 接口，根据命令行参数调用 SelectSlamMode() 进入建图或定位模式，
 * 通过 GetSlamMode() 等待运行结束，最后调用 SelectSlamMode(kIdle)
 * 切回空闲模式并调用 client.Stop()。
 *
 * 用法：chassis_example_slam <mapping|localization> <map_path>
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {
std::atomic<bool> g_running{true};

// Ctrl-C 后结束建图或定位等待循环。
void Stop(int) { g_running.store(false); }
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 ||
      (std::string(argv[1]) != "mapping" &&
       std::string(argv[1]) != "localization")) {
    std::cerr << "usage: chassis_example_slam <mapping|localization> <map_path>\n";
    return 2;
  }

  // 注册信号处理，允许用户按 Ctrl-C 停止当前 SLAM 模式。
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  // 2. 获取底盘接口，再获取 SLAM 接口。
  auto chassis = client.Chassis();
  auto slam = chassis->Slam();
  const std::string mode = argv[1];
  const std::string map_path = argv[2];

  // 3. 将命令行参数转换为 SDK 的 SLAM 模式枚举。
  const auto selected_mode = mode == "mapping"
                                 ? modi_sdk::SlamMode::kMapping
                                 : modi_sdk::SlamMode::kLocalization;
  // 4. 调用 SelectSlamMode() 启动建图或定位。
  const int start_ret = slam->SelectSlamMode(selected_mode, map_path);
  if (start_ret != 0) {
    std::cerr << "Start " << mode << " failed: " << start_ret << '\n';
    client.Stop();
    return 1;
  }

  std::cout << mode << " started. Press Ctrl-C to stop.\n";
  const auto expected_mode = mode == "mapping" ? modi_sdk::SlamMode::kMapping
                                                : modi_sdk::SlamMode::kLocalization;
  // 5. 通过 GetSlamMode() 等待服务端保持在目标模式。
  while (g_running.load() && slam->GetSlamMode() == expected_mode) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // 6. 退出前切回空闲模式，再停止 SDK 客户端。
  const int stop_ret = slam->SelectSlamMode(modi_sdk::SlamMode::kIdle);
  std::cout << "Stop ret=" << stop_ret << '\n';

  client.Stop();
  return stop_ret == 0 ? 0 : 1;
}
