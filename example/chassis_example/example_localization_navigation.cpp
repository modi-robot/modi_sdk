/**
 * @brief 演示加载地图、等待定位有效后执行底盘导航。
 *
 * 调用流程：创建并启动 RobotClient，获取 Chassis、Manager、Motion、State
 * 和 Slam 接口，调用 ClearFaults()、GetSafetyState()、PowerOn() 准备底盘，
 * 调用 Slam::LoadMap() 加载地图并等待 GetBasePose() 有效，再调用
 * moveHolonomicInMap() 导航；运行期间通过 GetSlamMode()、
 * GetNavigationStatus() 和 GetBasePose() 查询状态，退出时停止运动、下电
 * 并调用 client.Stop()。
 *
 * 地图名称由第一个命令行参数传入，并从 $MODI_WS_ROOT/maps 解析。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

constexpr modi_sdk::ChassisBasePose kTarget{0.0, 0.0, 0.0, true};
std::atomic<bool> g_running{true};

// Ctrl-C 后让定位或导航循环结束，主流程负责清理 SDK 和底盘状态。
void Stop(int) { g_running.store(false); }

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || argv[1][0] == '\0') {
    std::cerr << "用法: " << argv[0] << " <地图名称>\n";
    return 2;
  }
  const std::filesystem::path map_name(argv[1]);
  if (map_name.is_absolute() || map_name.has_parent_path() ||
      map_name.string().find('\\') != std::string::npos ||
      map_name == "." || map_name == ".." ||
      map_name.filename().string().front() == '.') {
    std::cerr << "地图名称必须是 MODI_WS_ROOT/maps 下的直接子目录名\n";
    return 2;
  }
  const char* workspace_root = std::getenv("MODI_WS_ROOT");
  if (!workspace_root || workspace_root[0] == '\0') {
    std::cerr << "请设置 MODI_WS_ROOT（例如 /home/user/modi_ws）\n";
    return 2;
  }
  const auto map_directory =
      std::filesystem::path(workspace_root) / "maps" / map_name;
  // 注册信号处理，允许用户按 Ctrl-C 安全退出。
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘管理、运动、状态和 SLAM 接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();
  auto motion = chassis->Motion();
  auto state = chassis->State();
  auto slam = chassis->Slam();

  // 3. 加载地图和导航前，先清除故障、确认安全状态正常并给底盘上电。
  if (manager->ClearFaults() != 0 ||
      manager->GetSafetyState() != modi_sdk::SafetyState::kNormal ||
      manager->PowerOn() != 0) {
    std::cerr << "底盘准备失败\n";
    client.Stop();
    return 1;
  }

  // 4. 调用 LoadMap() 加载地图并切换到底盘定位模式。
  const int localization_result = slam->LoadMap(map_directory.string());
  if (localization_result != 0) {
    std::cerr << "LoadMap failed: " << localization_result << '\n';
    (void)motion->StopMotion();
    (void)manager->PowerOff();
    client.Stop();
    return 1;
  }
  std::cout << "Localization started with map: " << map_directory << '\n';
  std::cout << "Waiting for a valid localization pose...\n";

  // 5. 通过 GetSlamMode() 确认定位模式，并通过 GetBasePose() 等待有效位姿。
  bool localized = false;
  const auto localization_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(120);
  while (g_running.load() &&
         slam->GetSlamMode() == modi_sdk::SlamMode::kLocalization &&
         std::chrono::steady_clock::now() < localization_deadline) {
    const auto pose = state->GetBasePose();
    if (pose.valid && std::isfinite(pose.x) && std::isfinite(pose.y) &&
        std::isfinite(pose.yaw)) {
      std::cout << "Localization ready: pose=(" << pose.x << ", " << pose.y
                << ", " << pose.yaw << ")\n";
      localized = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (!localized) {
    std::cerr << "Localization did not become valid within 120 seconds; "
                 "verify the MID360 power and network connection.\n";
    (void)motion->StopMotion();
    (void)manager->PowerOff();
    client.Stop();
    return 1;
  }

  // 6. 定位有效后调用 moveHolonomicInMap() 发送地图导航目标。
  std::cout << "Navigating to target=(0, 0, 0)\n";
  const int navigation_result = motion->moveHolonomicInMap(kTarget);
  if (navigation_result != 0) {
    std::cerr << "moveHolonomicInMap failed: "
              << navigation_result << '\n';
    (void)motion->StopMotion();
    (void)manager->PowerOff();
    client.Stop();
    return 1;
  }

  // 7. 通过 State API 轮询导航状态和当前位姿，直到完成、失败或超时。
  const auto navigation_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(120);
  bool observed_running = false;
  while (g_running.load() &&
         slam->GetSlamMode() == modi_sdk::SlamMode::kLocalization &&
         std::chrono::steady_clock::now() < navigation_deadline) {
    const auto status = state->GetNavigationStatus();
    const auto pose = state->GetBasePose();
    std::cout << "navigation=" << static_cast<int>(status) << " pose=("
              << pose.x << ", " << pose.y << ", " << pose.yaw << ")\n";
    if (status == modi_sdk::ChassisNavigationStatus::kRunning) {
      observed_running = true;
    }
    const bool at_target =
        pose.valid && std::hypot(pose.x - kTarget.x, pose.y - kTarget.y) < 0.08 &&
        std::abs(std::remainder(pose.yaw - kTarget.yaw, 2.0 * M_PI)) < 0.08;
    if (status == modi_sdk::ChassisNavigationStatus::kFinished &&
        (observed_running || at_target)) {
      // 导航完成后停止运动、下电并关闭 SDK 客户端。
      std::cout << "Navigation finished\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 0;
    }
    if (status == modi_sdk::ChassisNavigationStatus::kError) {
      std::cerr << "Navigation failed\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // 中断、模式异常或超时时，都执行统一的停止、下电和退出流程。
  if (!g_running.load()) std::cerr << "Interrupted by user\n";
  else if (slam->GetSlamMode() != modi_sdk::SlamMode::kLocalization)
    std::cerr << "Localization process exited unexpectedly\n";
  else
    std::cerr << "Navigation timed out after 120 seconds\n";
  (void)motion->StopMotion();
  (void)manager->PowerOff();
  client.Stop();
  return 1;
}
