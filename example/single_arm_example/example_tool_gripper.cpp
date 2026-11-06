// 默认仅读取反馈；只有显式指定 position/catch/release/stop/clear 才发送控制指令。
#include <charconv>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "modi_sdk/robot_client.hpp"

namespace {

bool ParseNumber(const char* text, unsigned maximum, unsigned& value) {
  const std::string input(text);
  const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
  return result.ec == std::errc{} && result.ptr == input.data() + input.size() && value <= maximum;
}

void Usage(const char* program) {
  std::cout << "用法: " << program << " <robot_id> <arm_name> [操作]\n"
            << "  get                        分项读取开度、温度、故障和状态（默认）\n"
            << "  position <1~100>           设置目标开度（整数百分比）\n"
            << "  catch <1~100> <50~1000> [continuous]  速度(%)、力度(g)、可选连续夹取\n"
            << "  release <1~100>            按指定速度百分比张开\n"
            << "  stop                       优先停止\n"
            << "  clear                      清除可恢复故障\n"
            << "arm_name 使用 GetArmNames 返回的型号名称；固定使用第一块工具板、夹爪地址 1。\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    Usage(argv[0]);
    return 0;
  }
  if (argc < 3) {
    Usage(argv[0]);
    return 2;
  }
  unsigned first = 0, second = 0;
  const std::string action = argc == 3 ? "get" : argv[3];
  const bool continuous = argc == 7 && std::string(argv[6]) == "continuous";
  const bool valid =
      ((action == "get" || action == "stop" || action == "clear") && argc <= 4) ||
      ((action == "position" || action == "release") && argc == 5 &&
       ParseNumber(argv[4], 100, first) && first >= 1) ||
      (action == "catch" && (argc == 6 || continuous) &&
       ParseNumber(argv[4], 100, first) && first >= 1 &&
       ParseNumber(argv[5], 1000, second) && second >= 50);
  if (!valid) {
    Usage(argv[0]);
    return 2;
  }
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "SDK 启动失败\n";
    return 1;
  }
  auto eef = client.Robot()->Eef();
  const std::string arm = argv[2];
  int ret = 0;
  if (action == "get") {
    std::uint16_t opening = 0;
    std::int16_t temperature_c = 0;
    std::uint16_t fault_code = 0;
    auto status = modi_sdk::ToolGripperStatus::kUnknown;
    // 首次读取只登记轮询地址，尚未收到设备回包时可稍后重试。
    for (int attempt = 0; attempt < 20; ++attempt) {
      ret = eef->GetToolGripperOpening(arm, opening);
      if (ret == 0) ret = eef->GetToolGripperTemperature(arm, temperature_c);
      if (ret == 0) ret = eef->GetToolGripperFaultCode(arm, fault_code);
      if (ret == 0) ret = eef->GetToolGripperStatus(arm, status);
      if (ret != -402010) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (ret == 0) {
      std::cout << "opening=" << opening << " temperature_c=" << temperature_c
                << " fault=0x" << std::hex << fault_code << std::dec
                << " status=" << static_cast<unsigned>(status) << '\n';
    }
  } else if (action == "position") {
    ret = eef->SetToolGripperPosition(arm, first);
  } else if (action == "catch") {
    ret = eef->SetToolGripperCatch(arm, first, second, continuous);
  } else if (action == "release") {
    ret = eef->SetToolGripperRelease(arm, first);
  } else if (action == "stop") {
    ret = eef->SetToolGripperStop(arm);
  } else {
    ret = eef->ClearToolGripperFault(arm);
  }
  if (ret != 0) std::cerr << "调用失败，错误码=" << ret << '\n';
  else if (action != "get") std::cout << "请求已接收，实际执行结果需通过 get 确认\n";
  client.Stop();
  return ret == 0 ? 0 : 1;
}
