/**
 * @brief 使用键盘通过 MODI SDK 控制移动底盘。
 *
 * 本示例演示底盘速度控制的完整 SDK 调用流程：
 * 1. 创建并启动 modi_sdk::RobotClient。
 * 2. 通过 client.Chassis() 获取底盘接口，再获取 Manager、Model、Motion
 *    和 State 子接口。
 * 3. 调用 manager->ClearFaults() 清除可恢复故障，确认
 *    manager->GetSafetyState() == SafetyState::kNormal 后调用 PowerOn() 上电。
 * 4. 读取 model->GetMaxBaseLinearSpeed() 和
 *    model->GetMaxBaseAngularSpeed()，键盘控制默认使用速度上限的 60%。
 *    按 +/= 或 - 每次调整 10%，速度比例范围为 10% 到 100%。
 * 5. 平移模式持续调用 motion->SpeedTranslate(vx, vy)；
 *    阿克曼模式持续调用 motion->SpeedAckermann(vx, wz)。
 *    速度控制需要在运动期间周期性重复发送目标速度。
 * 6. 松开方向键或按下停止键后调用 motion->StopMotion()。
 * 7. 退出前调用 motion->StopMotion()、manager->PowerOff()，
 *    最后调用 client.Stop()。
 *
 * 键盘操作：w/s 前进后退，a/d 左右移动或转向，q/e/z/c 分别表示
 * 左前、右前、左后、右后，m 切换控制模式，+/= 提高速度，-
 * 降低速度，space/x 停止，Esc 或 Ctrl-C 退出。终端配置代码仅用于
 * 实时读取键盘输入，不属于 MODI SDK 调用流程。
 */

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include "modi_sdk/robot_client.hpp"

namespace {

// 进行键盘输入信号处理
volatile std::sig_atomic_t g_running = 1;
termios g_original_termios{};
int g_original_flags = -1;
bool g_terminal_configured = false;
void Stop(int) { g_running = 0; }
// 退出时恢复终端配置，并重新显示光标。
void RestoreTerminal() {
  if (g_terminal_configured) {
    (void)::tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
    (void)::fcntl(STDIN_FILENO, F_SETFL, g_original_flags);
  }
  // ANSI 转义序列：显示光标，并恢复终端默认显示样式。
  std::cout << "\033[?25h\033[0m" << std::endl;
}

}  // namespace

int main() {
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);

  if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
    std::cerr << "请在交互式终端中运行此示例。\n";
    return 2;
  }

  if (::tcgetattr(STDIN_FILENO, &g_original_termios) < 0) {
    std::cerr << "读取终端配置失败\n";
    return 2;
  }

  g_original_flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
  if (g_original_flags < 0) {
    std::cerr << "读取终端状态失败\n";
    return 2;
  }

  // 将终端切换为原始、非阻塞输入模式，使程序能够实时读取单个按键；
  // 如果配置失败，立即恢复终端原设置，避免影响当前终端。
  termios raw = g_original_termios;
  ::cfmakeraw(&raw);
  raw.c_oflag |= OPOST;
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0 ||
      ::fcntl(STDIN_FILENO, F_SETFL, g_original_flags | O_NONBLOCK) < 0) {
    (void)::tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
    std::cerr << "设置终端键盘输入失败\n";
    return 2;
  }

  g_terminal_configured = true;
  std::atexit(RestoreTerminal);
  // ANSI 转义序列：隐藏光标，避免刷新控制台时光标闪烁。
  std::cout << "\033[?25l";

  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘管理、模型、运动和状态接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();
  auto model = chassis->Model();
  auto motion = chassis->Motion();
  auto state = chassis->State();

  // 3. 运动前清除故障、检查安全状态并给底盘上电。
  if (manager->ClearFaults() != 0 ||
      manager->GetSafetyState() != modi_sdk::SafetyState::kNormal) {
    std::cerr << "底盘清除故障失败，或安全状态不是正常状态\n";
    client.Stop();
    return 1;
  }
  if (manager->PowerOn() != 0) {
    std::cerr << "底盘上电失败\n";
    client.Stop();
    return 1;
  }

  // 键盘控制默认使用底盘速度上限的 60%，每次调整 10%，范围为 10% 到 100%。
  constexpr double kSpeedStep = 0.1;
  double speed_scale = 0.6;
  const double max_linear_speed = model->GetMaxBaseLinearSpeed();
  const double max_angular_speed = model->GetMaxBaseAngularSpeed();
  if (max_linear_speed <= 0.0 || max_angular_speed <= 0.0) {
    std::cerr << "读取底盘速度上限失败\n";
    (void)motion->StopMotion();
    (void)manager->PowerOff();
    client.Stop();
    return 1;
  }

  bool ackermann_mode = false;
  bool stop_latched = false;
  bool was_moving = false;
  std::string status = "就绪";

  // 每次只记录一个方向键，避免通过多个按键组合产生运动方向。
  char motion_key = '\0';
  std::chrono::steady_clock::time_point motion_deadline{};

  while (g_running) {
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    const int poll_result = ::poll(&descriptor, 1, 20);
    if (poll_result < 0 && errno != EINTR) {
      std::cerr << "等待键盘输入失败：" << std::strerror(errno) << '\n';
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }

    unsigned char buffer[64] = {};
    bool input_ok = true;
    for (;;) {
      const ssize_t bytes = ::read(STDIN_FILENO, buffer, sizeof(buffer));
      if (bytes > 0) {
        for (ssize_t i = 0; i < bytes; ++i) {
          const char key = static_cast<char>(buffer[i]);
          const auto deadline = std::chrono::steady_clock::now() +
                                std::chrono::milliseconds(150);

          switch (key) {
            // Esc 的 ASCII 码为 27，用于退出键盘控制循环。
            case 27:
              g_running = 0;
              status = "退出";
              break;
            case 'm':
            case 'M':
              ackermann_mode = !ackermann_mode;
              stop_latched = true;
              motion_key = '\0';
              status =
                  ackermann_mode ? "已切换到阿克曼模式" : "已切换到平移模式";
              break;
            case '+':
            case '=':
              // 提高速度比例，最高不超过底盘速度上限的 100%。
              speed_scale += kSpeedStep;
              if (speed_scale > 1.0) speed_scale = 1.0;
              status = "速度比例提高";
              break;
            case '-':
              // 降低速度比例，最低保持在底盘速度上限的 10%。
              speed_scale -= kSpeedStep;
              if (speed_scale < 0.1) speed_scale = 0.1;
              status = "速度比例降低";
              break;
            case ' ':
            case 'x':
            case 'X':
              stop_latched = true;
              motion_key = '\0';
              status = "停止";
              break;
            case 'w':
            case 'W':
              stop_latched = false;
              motion_key = 'w';
              motion_deadline = deadline;
              break;
            case 's':
            case 'S':
              stop_latched = false;
              motion_key = 's';
              motion_deadline = deadline;
              break;
            case 'a':
            case 'A':
              stop_latched = false;
              motion_key = 'a';
              motion_deadline = deadline;
              break;
            case 'd':
            case 'D':
              stop_latched = false;
              motion_key = 'd';
              motion_deadline = deadline;
              break;
            case 'q':
            case 'Q':
              stop_latched = false;
              motion_key = 'q';
              motion_deadline = deadline;
              break;
            case 'e':
            case 'E':
              stop_latched = false;
              motion_key = 'e';
              motion_deadline = deadline;
              break;
            case 'z':
            case 'Z':
              stop_latched = false;
              motion_key = 'z';
              motion_deadline = deadline;
              break;
            case 'c':
            case 'C':
              stop_latched = false;
              motion_key = 'c';
              motion_deadline = deadline;
              break;
            default:
              break;
          }
        }
        continue;
      }

      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (bytes < 0 && errno == EINTR) continue;
      input_ok = false;
      break;
    }

    if (!input_ok) {
      std::cerr << "读取键盘输入失败\n";
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }
    if (!g_running) break;

    const auto now = std::chrono::steady_clock::now();
    const bool motion_active =
        !stop_latched && motion_deadline > now;
    int forward = 0;
    int lateral = 0;
    if (motion_active) {
      // q/e/z/c 分别对应左前、右前、左后、右后四个斜向方向。
      switch (motion_key) {
        case 'w':
          forward = 1;
          break;
        case 's':
          forward = -1;
          break;
        case 'a':
          lateral = 1;
          break;
        case 'd':
          lateral = -1;
          break;
        case 'q':
          forward = 1;
          lateral = 1;
          break;
        case 'e':
          forward = 1;
          lateral = -1;
          break;
        case 'z':
          forward = -1;
          lateral = 1;
          break;
        case 'c':
          forward = -1;
          lateral = -1;
          break;
        default:
          break;
      }
    }

    double vx = 0.0;
    double vy_or_wz = 0.0;
    if (ackermann_mode) {
      vx = max_linear_speed * speed_scale * forward;
      vy_or_wz = max_angular_speed * speed_scale * lateral;
    } else {
      // 对角移动时归一化，避免合速度超过当前速度比例。
      const double length =
          std::hypot(static_cast<double>(forward),
                     static_cast<double>(lateral));
      if (length > 0.0) {
        const double speed = max_linear_speed * speed_scale;
        vx = speed * forward / length;
        vy_or_wz = speed * lateral / length;
      }
    }

    const bool moving = std::abs(vx) > 1e-6 || std::abs(vy_or_wz) > 1e-6;
    int result = 0;
    if (moving) {
      // 速度控制期间需要持续发送指令。
      result = ackermann_mode
                   ? motion->SpeedAckermann(vx, vy_or_wz)  // m/s, rad/s
                   : motion->SpeedTranslate(vx, vy_or_wz); // m/s, m/s
      was_moving = true;
    } else if (was_moving) {
      result = motion->StopMotion();
      was_moving = false;
    }

    if (result != 0) {
      std::cerr << "底盘运动指令失败：" << result << '\n';
      (void)motion->StopMotion();
      (void)manager->PowerOff();
      client.Stop();
      return 1;
    }

    const auto base_velocity = state->GetBaseVelocity();
    const auto wheel_speeds = state->GetWheelSpeeds();
    const auto steering_positions = state->GetSteeringPositions();

    // ANSI 转义序列：清空终端屏幕，并将光标移动到左上角。
    std::cout << "\033[2J\033[H"
              << "MODI 底盘键盘控制\n"
              << "按键: w/s 前后, a/d "
              << (ackermann_mode ? "左右转向" : "左右平移")
              << ", q/e/z/c 斜向\n"
              << "控制: m 模式, +/- 调速, space/x 停止, Esc 退出\n\n"
              << std::fixed << std::setprecision(3)
              << "模式: " << (ackermann_mode ? "阿克曼" : "平移") << '\n'
              << "速度比例: " << speed_scale * 100.0 << "%\n"
              << "状态: " << status << '\n'
              << "目标: vx " << vx << " m/s, "
              << (ackermann_mode ? "wz " : "vy ") << vy_or_wz
              << (ackermann_mode ? " rad/s" : " m/s") << '\n'
              << "实际: vx " << base_velocity.linear_x << " m/s, vy "
              << base_velocity.linear_y << " m/s, wz "
              << base_velocity.angular_z << " rad/s\n"
              << "轮速:";
    for (double value : wheel_speeds) {
      std::cout << ' ' << value;
    }
    std::cout << " rad/s\n转角:";
    for (double value : steering_positions) {
      std::cout << ' ' << value;
    }
    std::cout << " rad\n" << std::flush;
  }

  // 4. 退出前停止运动、底盘下电并停止 SDK 客户端。
  (void)motion->StopMotion();
  (void)manager->PowerOff();
  client.Stop();
  return 0;
}
