// No automatic power or motion. Explicitly choose an arm policy or clear a stop.
#include <iostream>
#include <memory>
#include <string>
#include "modi_sdk/robot_client.hpp"

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot_id> <off|low|medium|high|get|clear|status> [arm_name]\n";
    return 2;
  }
  const std::string action = argv[2];
  const std::string arm_name = argc == 4 ? argv[3] : "";
  modi_sdk::CollisionSensitivity sensitivity = modi_sdk::CollisionSensitivity::kMedium;
  if (action == "off") sensitivity = modi_sdk::CollisionSensitivity::kOff;
  else if (action == "low") sensitivity = modi_sdk::CollisionSensitivity::kLow;
  else if (action == "medium") sensitivity = modi_sdk::CollisionSensitivity::kMedium;
  else if (action == "high") sensitivity = modi_sdk::CollisionSensitivity::kHigh;
  else if (action != "clear" && action != "status" && action != "get") { std::cerr << "Unknown action\n"; return 2; }
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(config);
  if (!client->Start()) return 1;
  auto manager = client->Robot()->Manager();
  int result = 0;
  if (action == "get") {
    result = manager->GetCollisionSensitivity(arm_name, sensitivity);
    if (!result) std::cout << "collision sensitivity=" << static_cast<int>(sensitivity) << '\n';
  } else if (action == "status") {
    std::vector<bool> flags;
    result = client->Robot()->State()->CollisionDetection(flags);
    if (!result) {
      const auto names = client->Robot()->Model()->GetActuatorJointNames();
      for (std::size_t i = 0; i < flags.size(); ++i)
        std::cout << (i < names.size() ? names[i] : std::to_string(i)) << "=" << flags[i] << '\n';
    }
  } else result = action == "clear" ? manager->ClearCollision() : manager->SetCollisionSensitivity(arm_name, sensitivity);
  std::cout << "result=" << result << ", safety=" << static_cast<int>(manager->GetSafetyState()) << '\n';
  client->Stop();
  return result ? 1 : 0;
}
