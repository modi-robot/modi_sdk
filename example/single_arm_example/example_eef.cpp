// example_eef — 硬件末端（灵巧手）最小读写示例
// 需 modi_system 已加载带 end_effector.master 的机型（如 rh56f1）。

#include <iostream>
#include <memory>
#include <vector>

#include "modi_sdk/robot_client.hpp"

int main(int argc, char* argv[]) {
  modi_sdk::RobotClient::Config cfg;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
    return 2;
  }
  cfg.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(cfg);
  auto eef = client->Robot()->Eef();

  if (!client->Start()) {
    std::cerr << "Start failed\n";
    return 1;
  }

  const int count = eef->GetActuatorCount();
  std::cout << "Eef actuator count: " << count << std::endl;
  if (count <= 0) {
    std::cerr << "No end-effector HAL exposed. Check robot config end_effector.master.\n";
    client->Stop();
    return 1;
  }

  const auto radians = eef->GetActuatorRadians();
  std::cout << "ActuatorRadians [" << radians.size() << "]:";
  for (double x : radians) std::cout << " " << x;
  std::cout << std::endl;

  std::vector<double> target = radians;
  if (!target.empty()) {
    // Hold current pose as a no-op command smoke test.
    const int ret = eef->SetActuatorAngle(target);
    std::cout << "SetActuatorAngle ret=" << ret << std::endl;
  }

  client->Stop();
  std::cout << "Done.\n";
  return 0;
}
