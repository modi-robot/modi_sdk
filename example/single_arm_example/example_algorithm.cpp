#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "modi_sdk/robot_client.hpp"

void PrintVec(const std::string& label, const std::vector<double>& v) {
  std::cout << label << " [" << v.size() << "]: ";
  for (auto x : v) std::cout << x << " ";
  std::cout << std::endl;
}

int main(int argc, char* argv[]) {
  modi_sdk::RobotClient::Config config;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
    return 2;
  }
  config.target_robot_id = argv[1];
  auto client = std::make_shared<modi_sdk::RobotClient>(config);
  auto model = client->Robot()->Model();
  auto state = client->Robot()->State();
  auto algo = client->Robot()->Algo();

  if (!client->Start()) {
    std::cerr << "Start failed" << std::endl;
    return 1;
  }

  // get current joint positions as reference
  auto ref_joints = state->GetActuatorJointPositions();
  if (ref_joints.empty()) {
    std::cerr
        << "GetActuatorJointPositions failed.\n"
        << "Make sure modi_system is running: ros2 run modi_system modi_system"
        << std::endl;
    client->Stop();
    return 1;
  }
  PrintVec("ref_joints", ref_joints);

  // FkPosSolver: joints -> end effector poses
  std::cout << "\n=== FkPosSolver ===" << std::endl;
  std::vector<std::vector<double>> fk_poses;
  int ret = algo->FkPosSolver(ref_joints, fk_poses);
  std::cout << "FkPosSolver ret=" << ret << std::endl;
  for (size_t i = 0; i < fk_poses.size(); ++i) {
    PrintVec("  ee[" + std::to_string(i) + "]", fk_poses[i]);
  }

  // SolveIkPos: end effector poses -> joints
  if (!fk_poses.empty()) {
    std::cout << "\n=== SolveIkPos ===" << std::endl;
    std::vector<double> ik_result;
    std::vector<double> psi_selected;
    modi_sdk::IkSolveOptions options;
    ret = algo->SolveIkPos(ref_joints, fk_poses, options, ik_result,
                           psi_selected);
    std::cout << "SolveIkPos ret=" << ret << std::endl;
    if (ret == 0) PrintVec("  ik_result", ik_result);
  }

  // IdSolver: inverse dynamics
  std::cout << "\n=== IdSolver ===" << std::endl;
  std::vector<double> zero_vel(ref_joints.size(), 0.0);
  std::vector<double> zero_acc(ref_joints.size(), 0.0);
  std::vector<double> torques;
  ret = algo->IdSolver(ref_joints, zero_vel, zero_acc, torques);
  std::cout << "IdSolver ret=" << ret << std::endl;
  if (ret == 0) PrintVec("  gravity_torques", torques);
  client->Stop();
  std::cout << "Done." << std::endl;
  return 0;
}
