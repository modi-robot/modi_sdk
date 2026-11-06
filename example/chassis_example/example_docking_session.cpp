/**
 * @brief 演示通过 MODI SDK 启动、查询及结束回充会话。
 *
 * 调用流程：创建并启动 modi_sdk::RobotClient，通过 client.Chassis()
 * 获取底盘接口，再通过 chassis->Manager() 获取管理接口，
 * 调用 StartDocking、GetDockingState 和 CancelDocking。
 *
 * s 一键回充，x 结束充电，p 查询状态，q/Esc/Ctrl-C 退出。
 * 充电器已连接时 StartDocking 停稳后直接充电；未连接时需先定位并配置有效充电桩。
 * 服务端负责充电、满电待机及自动补充充电。
 * CancelDocking 返回 0 表示请求接受，active=false 才表示关闭完成。
 * 发出过启动请求后，退出前请求关闭并最多等待 10 秒确认，再调用 client.Stop()。
 * 通过首个命令行参数指定 chassis_id，DDS Domain ID 使用默认值 0。
 */

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;

// Ctrl-C 后结束键盘循环，主流程负责关闭会话并停止 SDK 客户端。
void Stop(int) { g_running = 0; }

void PrintState(const modi_sdk::ChassisDockingState& state) {
  std::cout << "state=" << state.state << " active=" << state.active
            << " charged_once=" << state.charged_once
            << " charging_enabled=" << state.charging_enabled
            << " recharge_soc_threshold=" << state.recharge_soc_threshold
            << "% task_error_code=" << state.task_error_code
            << " message=" << state.message << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || argv[1][0] == '\0') {
    std::cerr << "Usage: " << argv[0] << " <chassis_id>\n";
    return 2;
  }

  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  // 1. 创建并启动 SDK 客户端。
  if (!::isatty(STDIN_FILENO)) {
    std::cerr << "请在交互式终端中运行此示例。\n";
    return 2;
  }
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }
  // 2. 获取底盘接口和管理接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();

  // 关闭行缓冲和回显，保留 Ctrl-C 信号，按键无需回车。
  termios original{};
  if (::tcgetattr(STDIN_FILENO, &original) != 0) {
    std::cerr << "读取终端配置失败\n";
    client.Stop();
    return 2;
  }
  auto mode = original;
  mode.c_lflag &= ~(ICANON | ECHO);
  mode.c_cc[VMIN] = 1;
  mode.c_cc[VTIME] = 0;
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &mode) != 0) {
    std::cerr << "设置终端键盘输入失败\n";
    client.Stop();
    return 2;
  }
  std::cout << "s: 一键回充  x: 结束充电  p: 查询状态  q/Esc/Ctrl-C: 退出\n"
            << "回充会话包括满电待机，充满后 active 仍为 true。\n"
            << std::flush;

  // 3. 按键直接调用 SDK 接口。
  int exit_code = 0;
  bool needs_session_cleanup = false;
  while (g_running) {
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int ready = ::poll(&input, 1, 100);
    if (ready < 0) {
      if (errno == EINTR) continue;
      exit_code = 1;
      break;
    }
    if (ready == 0) continue;
    if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      exit_code = 1;
      break;
    }
    if (!(input.revents & POLLIN)) continue;
    char key = 0;
    if (::read(STDIN_FILENO, &key, 1) != 1) {
      if (errno == EINTR) continue;
      exit_code = 1;
      break;
    }
    if (!g_running) break;
    if (key == 'q' || key == 'Q' || key == '\x1b') break;
    if (key == 's' || key == 'S') {
      std::string message;
      // 启动超时也可能已被服务端接受，退出前仍需关闭并确认。
      needs_session_cleanup = true;
      const int result = manager->StartDocking(&message);
      std::cout << "StartDocking result=" << result << " " << message << '\n';
    } else if (key == 'x' || key == 'X') {
      std::string message;
      const int result = manager->CancelDocking(&message);
      std::cout << "CancelDocking result=" << result << " " << message
                << "；请查询 active 确认关闭完成。\n";
    } else if (key == 'p' || key == 'P') {
      modi_sdk::ChassisDockingState state;
      const int result = manager->GetDockingState(&state);
      if (result == 0) {
        PrintState(state);
        if (!state.active) needs_session_cleanup = false;
      } else {
        std::cout << "GetDockingState result=" << result << '\n';
      }
    }
    std::cout << std::flush;
  }

  // 4. 恢复终端，清理本示例发出的启动请求并确认，最后停止 SDK 客户端。
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &original) != 0) {
    std::cerr << "恢复终端配置失败\n";
    exit_code = 1;
  }
  if (needs_session_cleanup) {
    std::string message;
    const int result = manager->CancelDocking(&message);
    std::cout << "退出前 CancelDocking result=" << result << " " << message << '\n';
    bool closed = false;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      modi_sdk::ChassisDockingState state;
      if (manager->GetDockingStateQuiet(500, &state) == 0 && !state.active) {
        PrintState(state);
        closed = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!closed) {
      std::cerr << "尚未确认回充会话关闭，请重新查询服务端状态。\n";
      exit_code = 1;
    }
  }
  client.Stop();
  return exit_code;
}
