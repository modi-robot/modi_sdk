/**
 * @brief 演示通过 MODI SDK 启动和停止建图或定位。
 *
 * 调用流程：创建并启动 RobotClient，通过 client.Chassis()->Slam() 获取
 * Slam 接口。建图时由 system 管理临时目录，Ctrl-C 后按名称同步保存；
 * 定位时先按名称加载地图，再显式切换到定位模式。
 *
 * 用法：chassis_example_slam <chassis_id> <mapping|localization> <map_name>
 * 通过首个命令行参数指定 chassis_id，DDS Domain ID 使用默认值 0。
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"

namespace {
std::atomic<bool> g_running{true};

// Ctrl-C 后结束建图或定位等待循环。
void Stop(int) { g_running.store(false); }
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4 || argv[1][0] == '\0' ||
      (std::string(argv[2]) != "mapping" &&
       std::string(argv[2]) != "localization")) {
    std::cerr << "usage: chassis_example_slam "
                 "<chassis_id> <mapping|localization> <map_name>\n";
    return 2;
  }

  // 注册信号处理，允许用户按 Ctrl-C 停止当前 SLAM 模式。
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  // 2. 获取底盘接口，再获取 SLAM 接口。
  auto chassis = client.Chassis();
  auto slam = chassis->Slam();
  const std::string mode = argv[2];
  const std::string map_name = argv[3];

  // 地图列表由 chassis system 扫描和校验；SDK 客户端不读取本机地图目录。
  std::vector<std::string> maps;
  const int list_ret = slam->GetMapList(&maps);
  if (list_ret == modi_sdk::ToInt(modi_sdk::SdkError::kOk)) {
    std::cout << "available maps:";
    for (const auto& name : maps) std::cout << ' ' << name;
    std::cout << '\n';
  } else {
    std::cerr << "GetMapList failed: " << list_ret << '\n';
  }

  // 3. 定位模式先让 system 按名称加载正式地图。
  if (mode == "localization") {
    const int load_ret = slam->LoadMap(map_name);
    if (load_ret != 0) {
      std::cerr << "LoadMap failed: " << load_ret << '\n';
      client.Stop();
      return 1;
    }
  }

  // 4. 将命令行参数转换为 SDK 的 SLAM 模式枚举并启动。
  const auto selected_mode = mode == "mapping"
                                 ? modi_sdk::SlamMode::kMapping
                                 : modi_sdk::SlamMode::kLocalization;
  const int start_ret = slam->SetSlamMode(selected_mode);
  if (start_ret != 0) {
    std::cerr << "Start " << mode << " failed: " << start_ret << '\n';
    client.Stop();
    return 1;
  }

  // 定位模式下，名称与原始二维栅格由同一个 system 快照返回。
  if (mode == "localization") {
    modi_sdk::GridMapInfo current_map;
    const int map_ret = slam->GetCurrentMapInfo(&current_map);
    if (map_ret == modi_sdk::ToInt(modi_sdk::SdkError::kOk)) {
      std::cout << "current map: " << current_map.name << " ("
                << current_map.width << 'x' << current_map.height << ")\n";
    } else {
      std::cerr << "GetCurrentMapInfo failed: " << map_ret << '\n';
    }
  }

  std::cout << mode << " started. Press Ctrl-C to stop.\n";
  const auto expected_mode = mode == "mapping" ? modi_sdk::SlamMode::kMapping
                                                : modi_sdk::SlamMode::kLocalization;
  // 5. 通过 GetSlamMode() 等待服务端保持在目标模式。
  while (g_running.load()) {
    modi_sdk::SlamMode current_mode = modi_sdk::SlamMode::kIdle;
    const int state_ret = slam->GetSlamMode(&current_mode);
    if (state_ret != 0 || current_mode != expected_mode) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // 6. 建图同步保存；定位则显式切回空闲模式。
  const int finish_ret = mode == "mapping"
                             ? slam->SaveMapping(map_name)
                             : slam->SetSlamMode(modi_sdk::SlamMode::kIdle);
  std::cout << (mode == "mapping" ? "SaveMapping" : "SetSlamMode(kIdle)")
            << " ret=" << finish_ret << '\n';

  client.Stop();
  return finish_ret == 0 ? 0 : 1;
}
