// 双臂：轮询整机关节与全部末端状态
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"

static volatile bool g_running = true;
void SignalHandler(int) { g_running = false; }

void PrintVec(const std::string& label, const std::vector<double>& v,
              int max_n = 6) {
  std::cout << label << " [" << v.size() << "]: ";
  for (int i = 0; i < static_cast<int>(v.size()) && i < max_n; ++i)
    std::cout << v[i] << " ";
  if (static_cast<int>(v.size()) > max_n) std::cout << "...";
  std::cout << std::endl;
}

int main(int argc, char* argv[]) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  modi_sdk::RobotClient::Config config;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
    return 2;
  }
  config.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(config);
  auto state = client->Robot()->State();
  auto motion = client->Robot()->Motion();

  if (!client->Start()) {
    std::cerr << "Start failed" << std::endl;
    return 1;
  }

  while (g_running) {
    auto pos = state->GetActuatorJointPositions();
    auto vel = state->GetActuatorJointVelocities();
    auto torque = state->GetActuatorJointTorquesData();

    if (!pos.empty()) PrintVec("  pos", pos);
    if (!vel.empty()) PrintVec("  vel", vel);
    if (!torque.empty()) PrintVec("  torque", torque);

    auto ee_poses = state->GetEndEffectorPoses();
    if (!ee_poses.empty()) {
      for (size_t i = 0; i < ee_poses.size(); ++i) {
        PrintVec("  ee[" + std::to_string(i) + "] xyzrpy", ee_poses[i]);
      }
    }

    // End effector wrenches
    auto wrenches = state->GetEndEffectorWrenchesData();
    if (!wrenches.empty()) {
      for (size_t i = 0; i < wrenches.size(); ++i) {
        PrintVec("  wrench[" + std::to_string(i) + "]", wrenches[i]);
      }
    }

    // Planner status
    const auto status = motion->GetPlannerStatus();
    std::cout << "  PlannerStatus: " << static_cast<int>(status) << std::endl;

    std::cout << "---" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  client->Stop();
  std::cout << "Done." << std::endl;
  return 0;
}
