// 双臂：打印整机关节/末端信息（末端通常为 2）
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "modi_sdk/robot_client.hpp"

void PrintVec(const std::string& label, const std::vector<double>& v) {
  std::cout << label << " [" << v.size() << "]: ";
  for (auto x : v) std::cout << x << " ";
  std::cout << std::endl;
}

int main() {
  auto client = std::make_shared<modi_sdk::RobotClient>();
  auto model = client->Robot()->Model();

  if (!client->Start()) {
    std::cerr << "Start failed" << std::endl;
    return 1;
  }

  // DOF
  std::cout << "AllJointDOF: " << model->GetAllJointDOF() << std::endl;
  std::cout << "ActuatorJointDOF: " << model->GetActuatorJointDOF()
            << std::endl;

  // Joint names
  auto all_names = model->GetAllJointNames();
  std::cout << "AllJointNames [" << all_names.size() << "]:";
  for (const auto& n : all_names) std::cout << " " << n;
  std::cout << std::endl;

  auto act_names = model->GetActuatorJointNames();
  std::cout << "ActuatorJointNames [" << act_names.size() << "]:";
  for (const auto& n : act_names) std::cout << " " << n;
  std::cout << std::endl;

  // Joint limits
  std::vector<double> lower, upper;
  if (model->GetActuatorJointLimits(lower, upper) == 0) {
    PrintVec("  lower", lower);
    PrintVec("  upper", upper);
  }

  // End effector names
  auto ee_names = model->GetEndEffectorNames();
  std::cout << "EndEffectorNames [" << ee_names.size() << "]:";
  for (const auto& n : ee_names) std::cout << " " << n;
  std::cout << std::endl;

  // TCP offset
  for (const auto& name : ee_names) {
    std::vector<double> tcp;
    if (model->GetTcpOffset(name, tcp) == 0) {
      PrintVec("  TcpOffset(" + name + ")", tcp);
    }
  }

  // Control period
  std::cout << "ControlPeriod: " << model->GetControlPeriod() << " s"
            << std::endl;

  client->Stop();
  std::cout << "Done." << std::endl;
  return 0;
}
