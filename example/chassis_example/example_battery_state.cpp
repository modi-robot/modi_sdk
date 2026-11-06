/**
 * @brief 演示通过 MODI SDK 查询底盘电池状态。
 *
 * 调用流程：创建并启动 modi_sdk::RobotClient，通过 client.Chassis()
 * 获取底盘接口，再通过 chassis->State() 获取状态接口，
 * 分别调用电流、电压、SOC、温度、电池是否在位、充电器连接/充电/充满状态、
 * 错误/警告和充电器故障码接口，每轮查询结束后等待 1000 ms。
 * 数组顺序与服务端电池舱 battery.id 一致，只返回最新缓存值。
 *
 * CAN 接口在服务端 battery.bus 配置，客户端只通过 DDS 查询。
 * 通过首个命令行参数指定 chassis_id，DDS Domain ID 使用默认值 0。
 * 按 Ctrl-C 结束读取，最后调用 client.Stop()。
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

// Ctrl-C 后结束读取循环，主流程负责停止 SDK 客户端。
void Stop(int) { g_running.store(false); }

template <typename T>
void PrintValues(const char* name, const std::vector<T>& values,
                 const char* unit = "") {
  std::cout << name << ':';
  if (values.empty()) std::cout << " 读取失败或服务不可用";
  for (const auto value : values) std::cout << ' ' << value << unit;
  std::cout << '\n';
}

const char* ChargingStateName(modi_sdk::ChargingState state) {
  switch (state) {
    case modi_sdk::ChargingState::kNotConnected:
      return "未连接";
    case modi_sdk::ChargingState::kConnected:
      return "已连接";
    case modi_sdk::ChargingState::kCharging:
      return "充电中";
    case modi_sdk::ChargingState::kFullyCharged:
      return "已充满";
  }
  return "未知";
}

// 充电状态为枚举，按可读名称打印，顺序与 battery.id 一致。
void PrintChargingStates(
    const std::vector<modi_sdk::ChargingState>& states) {
  std::cout << "charging_state:";
  if (states.empty()) std::cout << " 读取失败或服务不可用";
  for (const auto state : states) {
    std::cout << ' ' << ChargingStateName(state);
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || argv[1][0] == '\0') {
    std::cerr << "Usage: " << argv[0] << " <chassis_id>\n";
    return 2;
  }

  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  std::cout << std::boolalpha << std::fixed << std::setprecision(2);

  // 1. 使用命令行指定的底盘 ID 创建并启动 SDK 客户端。
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘接口和状态接口。
  auto chassis = client.Chassis();
  auto state = chassis->State();

  std::cout << "目标底盘: " << config.target_robot_id
            << ", DDS Domain ID: " << config.domain_id << '\n';
  std::cout << "分项读取底盘电池舱，顺序与 battery.id 一致，按 Ctrl-C 退出\n";
  std::cout << "电池舱数据和充电器故障码超过 8 秒未收到 CAN 报文时失效。\n";
  std::cout << "charging_state 为电池舱充电状态：未连接=无电池/充电器未接，"
               "已连接=尚未检测到充电电流，充电中=检测到充电电流，"
               "已充满=BMS 0x180 报告整组充电完成，映射当前参与充电舱，"
               "不是各舱独立满电报文，也不按 SOC 或低电流推断。\n";

  // 3. 按需调用独立接口，与电机电流/电压/温度的读取方式一致。
  while (g_running.load()) {
    PrintValues("current", state->GetBatteryCurrents(), "A");
    if (!g_running.load()) break;
    PrintValues("voltage", state->GetBatteryVoltages(), "V");
    if (!g_running.load()) break;
    PrintValues("soc", state->GetBatterySoc(), "%");
    if (!g_running.load()) break;
    PrintValues("temperature", state->GetBatteryTemperatures(), "C");
    if (!g_running.load()) break;
    PrintValues("has_battery", state->HasBattery());
    if (!g_running.load()) break;
    PrintChargingStates(state->GetBatteryChargingStates());
    if (!g_running.load()) break;
    std::cout << std::hex << std::showbase;
    PrintValues("error_bits", state->GetBatteryErrorBits());
    if (!g_running.load()) break;
    PrintValues("warning_bits", state->GetBatteryWarningBits());
    if (!g_running.load()) break;
    std::cout << std::noshowbase;
    const auto charger_fault_code = state->GetChargerFaultCode();
    std::cout << "charger_fault_code: ";
    if (charger_fault_code) {
      std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(*charger_fault_code) << std::dec
                << std::setfill(' ');
    } else {
      std::cout << "无有效报文";
    }
    std::cout << '\n';
    std::cout << std::dec << std::noshowbase;
    std::cout << "---\n" << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // 4. 状态读取结束后停止 SDK 客户端。
  client.Stop();
  return 0;
}
