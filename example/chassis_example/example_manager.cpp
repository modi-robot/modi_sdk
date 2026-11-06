/**
 * @brief 演示通过 MODI SDK 管理移动底盘。
 *
 * 调用流程：
 * 1. 创建并启动 modi_sdk::RobotClient。
 * 2. 通过 client.Chassis() 获取底盘接口，再获取 Manager 子接口。
 * 3. 直接调用 Manager API 读取 Domain ID、清除故障、读取状态、
 *    上下电、释放/锁定抱闸以及触发/解除急停。
 * 4. 完成操作后调用 client.Stop() 关闭 SDK 客户端。
 *
 * 命令行参数：
 * - <chassis_id>：依次演示清故障、状态读取、上下电和抱闸控制。
 * - <chassis_id> estop-on / estop-off：触发或解除急停。
 * - <chassis_id> set-domain <id>：修改 DDS Domain ID，重启后生效。
 * 通过首个命令行参数指定 chassis_id，DDS Domain ID 使用默认值 0。
 */

#include <iostream>
#include <string>

#include "modi_sdk/robot_client.hpp"

int main(int argc, char** argv) {
  // 命令行参数指定管理操作。
  if ((argc != 2 && argc != 3 && argc != 4) || argv[1][0] == '\0') {
    std::cerr << "usage: chassis_example_manager "
                 "<chassis_id> [estop-on|estop-off|set-domain <id>]\n";
    return 2;
  }

  // 1. 创建并启动 SDK 客户端，建立与底盘服务的通信。
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) {
    std::cerr << "RobotClient::Start failed\n";
    return 1;
  }

  // 2. 通过 RobotClient 获取底盘接口，再获取底盘管理接口。
  auto chassis = client.Chassis();
  auto manager = chassis->Manager();

  // 3. 读取底盘当前配置的 DDS Domain ID。
  std::cout << "RobotDomainId: " << manager->GetRobotDomainId() << '\n';

  if (argc >= 3 && std::string(argv[2]) == "set-domain") {
    if (argc != 4) {
      std::cerr << "set-domain requires an id\n";
      client.Stop();
      return 2;
    }
    const int domain_id = std::stoi(argv[3]);
    // 修改配置文件中的 Domain ID，重启底盘服务和 SDK 后生效。
    std::cout << "SetRobotDomainId(" << domain_id << "): "
              << manager->SetRobotDomainId(domain_id) << '\n';
    client.Stop();
    return 0;
  }

  if (argc == 3 && std::string(argv[2]) == "estop-on") {
    // 触发急停：Estop(true)。
    std::cout << "Estop(true): " << manager->Estop(true) << '\n';
    client.Stop();
    return 0;
  }

  if (argc == 3 && std::string(argv[2]) == "estop-off") {
    // 解除急停：Estop(false)。
    std::cout << "Estop(false): " << manager->Estop(false) << '\n';
    client.Stop();
    return 0;
  }
  if (argc != 2) {
    std::cerr << "unknown command\n";
    client.Stop();
    return 2;
  }

  // 4. 清除可恢复故障，并读取安全状态和底盘状态。
  std::cout << "ClearFaults: " << manager->ClearFaults() << '\n';
  int safety_error = modi_sdk::ToInt(modi_sdk::SdkError::kOk);
  int chassis_error = modi_sdk::ToInt(modi_sdk::SdkError::kOk);
  const auto safety_state = manager->GetSafetyState(&safety_error);
  const auto chassis_state = manager->GetChassisState(&chassis_error);
  std::cout << "SafetyState: " << static_cast<int>(safety_state)
            << " error=" << safety_error << '\n';
  std::cout << "ChassisState: " << static_cast<int>(chassis_state)
            << " error=" << chassis_error << '\n';

  // 5. 演示底盘上电、释放抱闸、锁定抱闸和下电。
  std::cout << "PowerOn: " << manager->PowerOn() << '\n';
  std::cout << "BrakeRelease: " << manager->BrakeRelease() << '\n';
  std::cout << "BrakeLock: " << manager->BrakeLock() << '\n';
  std::cout << "PowerOff: " << manager->PowerOff() << '\n';

  // 6. 所有管理操作完成后停止 SDK 客户端。
  client.Stop();
  return 0;
}
