// 双侧独立周期发送随机开度；每 10 次发送最大开度，最大开度须反馈到位后继续。
// 运行前手动上电。Ctrl+C 或任一侧失败时停止两侧夹爪。
#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "modi_sdk/robot_client.hpp"
#include "modi_type_def.h"

namespace {
std::atomic<bool> g_running{true};
static_assert(std::atomic<bool>::is_always_lock_free);
}  // namespace

int main(int argc, char* argv[]) {
  if ((argc == 2 && std::string(argv[1]) == "--help") ||
      (argc != 2 && argc != 4 && argc != 5 && argc != 6 && argc != 7)) {
    std::cout << "用法: " << argv[0]
              << " <robot_id> [最小开度 最大开度] [运行秒数] [domain_id] [周期_ms]\n"
              << "默认: 1~100%，持续运行，DDS 域 0，周期 100 ms；Ctrl+C 结束。\n";
    return argc == 2 ? 0 : 2;
  }
  std::array<unsigned, 5> options{1, 100, 0, 0, 100};
  for (int i = 2; i < argc; ++i) {
    const std::string text = argv[i];
    const auto result = std::from_chars(text.data(), text.data() + text.size(), options[i - 2]);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
      std::cerr << "参数必须为非负整数: " << text << '\n';
      return 2;
    }
  }
  if (options[0] < 1 || options[0] > options[1] || options[1] > 100 ||
      options[3] > 232 || options[4] == 0) {
    std::cerr << "要求 1 <= 最小开度 <= 最大开度 <= 100，domain_id <= 232，周期_ms > 0\n";
    return 2;
  }
  std::signal(SIGINT, [](int) { g_running.store(false); });
  std::signal(SIGTERM, [](int) { g_running.store(false); });

  using Clock = std::chrono::steady_clock;
  std::mutex mutex;
  ToolGripperStateArray feedback;
  std::uint64_t received_ms = 0;
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  config.domain_id = static_cast<int>(options[3]);
  modi_sdk::RobotClient client(config);
  if (client.RegisterSubscriber({"/Robot/Eef/ToolGripperState"},
      [&](const modi_sdk::Parser& parser) {
        std::lock_guard<std::mutex> lock(mutex);
        feedback = parser.Value<ToolGripperStateArray>("/Robot/Eef/ToolGripperState");
        received_ms = parser.ReceivedAtMs("/Robot/Eef/ToolGripperState");
      }) != 0 || !client.Start()) {
    std::cerr << "SDK 启动或订阅失败\n";
    return 1;
  }
  const auto arms = client.Robot()->Model()->GetArmNames();
  if (arms.size() != 2 || arms[0].empty() || arms[1].empty() || arms[0] == arms[1]) {
    std::cerr << "需要两个不同的机械臂名称\n";
    return 1;
  }
  const auto eef = client.Robot()->Eef();
  const auto period = std::chrono::milliseconds(options[4]);
  const auto start = Clock::now();
  const auto deadline = options[2] ? start + std::chrono::seconds(options[2])
                                   : Clock::time_point::max();
  std::atomic<bool> failed{false};
  std::array<std::thread, 2> workers;
  std::cout << "双侧独立发送，范围=" << options[0] << '~' << options[1]
            << "%，周期=" << options[4] << " ms；最大开度等待到位，Ctrl+C 结束。\n";
  for (std::size_t i = 0; i < workers.size(); ++i) {
    workers[i] = std::thread([&, i] {
      std::mt19937 random(std::random_device{}());
      std::uniform_int_distribution<unsigned> opening(options[0], options[1]);
      auto next = start;
      std::uint64_t calls = 0;
      std::string error;
      while (g_running.load() && Clock::now() < deadline && error.empty()) {
        // 分段等待，长周期也能及时响应 Ctrl+C。
        while (g_running.load() && Clock::now() < std::min(next, deadline)) {
          std::this_thread::sleep_until(std::min(
              {next, deadline, Clock::now() + std::chrono::milliseconds(50)}));
        }
        if (!g_running.load() || Clock::now() >= deadline) break;
        const auto target = static_cast<std::uint16_t>(
            ++calls % 10 == 0 ? options[1] : opening(random));
        const int ret = eef->SetToolGripperPosition(arms[i], target);
        if (ret != 0) {
          error = "设置开度失败，返回码=" + std::to_string(ret);
          break;
        }
        if (target == options[1]) {
          const auto sent = Clock::now();
          const auto sent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   sent.time_since_epoch()).count();
          bool arrived = false;
          while (g_running.load() && Clock::now() < deadline && error.empty() && !arrived) {
            const auto now = Clock::now();
            {
              std::lock_guard<std::mutex> lock(mutex);
              const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      Clock::now().time_since_epoch()).count();
              const auto stamp = static_cast<std::int64_t>(received_ms);
              if (stamp > sent_ms && stamp <= now_ms && now_ms - stamp <= 500) {
                const auto& names = feedback.arm_names();
                const auto arm = std::find(names.begin(), names.end(), arms[i]);
                if (std::count(names.begin(), names.end(), arms[i]) != 1 ||
                    feedback.openings().size() != names.size() ||
                    feedback.statuses().size() != names.size() ||
                    feedback.fault_codes().size() != names.size() ||
                    feedback.error_codes().size() != names.size()) {
                  error = "夹爪反馈缺失、重复或字段长度不一致";
                } else {
                  const auto index = static_cast<std::size_t>(arm - names.begin());
                  const auto status = feedback.statuses()[index];
                  if (feedback.error_codes()[index] != 0 ||
                      feedback.fault_codes()[index] != 0 || status == 0 || status > 6) {
                    error = "夹爪反馈异常，错误码=" + std::to_string(feedback.error_codes()[index]) +
                            "，故障码=" + std::to_string(feedback.fault_codes()[index]) +
                            "，状态=" + std::to_string(status);
                  } else {
                    // 反馈开度 0~1000，命令为 1~100%；允许 1% 误差且已停止运动。
                    arrived = std::abs(static_cast<int>(feedback.openings()[index]) -
                                       target * 10) <= 10 && status != 4 && status != 5;
                  }
                }
              } else if (now_ms - sent_ms > 500) {
                error = "夹爪反馈超过 500 ms 未更新";
              }
            }
            if (!arrived && now - sent >= std::chrono::seconds(10)) error = "最大开度到位超时";
            if (!arrived && error.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
          if (!arrived) break;
          next = Clock::now();
        }
        next += period;
        const auto now = Clock::now();
        if (now > next) next += period * ((now - next) / period + 1);  // 跳过过期时隙，不补发。
      }
      if (!error.empty()) {
        g_running.store(false);
        if (!failed.exchange(true)) std::cerr << arms[i] << ": " << error << '\n';
      }
    });
  }
  for (auto& worker : workers) worker.join();
  for (const auto& arm : arms) {
    const int ret = eef->SetToolGripperStop(arm);
    if (ret != 0) {
      failed.store(true);
      std::cerr << arm << ": 停止失败，返回码=" << ret << '\n';
    }
  }
  client.Stop();
  return failed.load() ? 1 : 0;
}
