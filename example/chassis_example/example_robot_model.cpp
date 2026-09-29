/**
 * @brief 演示通过 MODI SDK 读取移动底盘模型信息。
 *
 * 调用流程：创建并启动 modi_sdk::RobotClient，通过 client.Chassis()
 * 获取底盘接口，再通过 chassis->Model() 获取模型接口，最后直接调用
 * GetSteeringJointNames()、GetWheelJointNames()、GetWheelCount()、
 * GetWheelRadius()、GetControlPeriod() 和各种 GetMaxBase*() API 读取模型。
 * 读取完成后调用 client.Stop() 关闭 SDK 客户端。
 */

#include <iomanip>
#include <iostream>

#include "modi_sdk/robot_client.hpp"

int main() {
  // 1. 创建并启动 SDK 客户端。
  modi_sdk::RobotClient client;
  if (!client.Start()) {
    std::cerr << "RobotClient::Start() 失败\n";
    return 1;
  }

  // 2. 获取底盘接口和模型接口。
  auto chassis = client.Chassis();
  auto model = chassis->Model();

  // 3. 读取转向关节名称，帮助用户确认底盘模型的关节拓扑。
  const auto steering_joint_names = model->GetSteeringJointNames();
  std::cout << "SteeringJointNames [" << steering_joint_names.size() << "]:";
  for (const auto& name : steering_joint_names) {
    std::cout << ' ' << name;
  }
  std::cout << '\n';

  // 4. 读取轮关节名称和轮组数量。
  const auto wheel_joint_names = model->GetWheelJointNames();
  std::cout << "WheelJointNames [" << wheel_joint_names.size() << "]:";
  for (const auto& name : wheel_joint_names) {
    std::cout << ' ' << name;
  }
  std::cout << '\n';

  // 5. 读取轮径、控制周期以及底盘速度和加速度限制。
  std::cout << "WheelCount: " << model->GetWheelCount() << '\n'
            << "WheelRadius: " << model->GetWheelRadius() << '\n'
            << "ControlPeriod: " << model->GetControlPeriod() << '\n'
            << "MaxBaseLinearSpeed: " << model->GetMaxBaseLinearSpeed()
            << '\n'
            << "MaxBaseAngularSpeed: " << model->GetMaxBaseAngularSpeed()
            << '\n'
            << "MaxBaseLinearAccel: " << model->GetMaxBaseLinearAccel() << '\n'
            << "MaxBaseAngularAccel: " << model->GetMaxBaseAngularAccel()
            << '\n';

  // 6. 读取每个转向关节的位置限制。
  const auto steering_limits = model->GetSteeringPositionLimits();
  std::cout << std::fixed << std::setprecision(4)
            << "SteeringPositionLimits:\n";
  for (std::size_t i = 0; i < steering_limits.size(); ++i) {
    std::cout << "  steering[" << i << "]: [" << steering_limits[i].first
              << ", " << steering_limits[i].second << "]\n";
  }

  // 7. 读取每个轮组相对于底盘坐标系的安装位姿。
  const auto wheel_mount_poses = model->GetWheelMountPoses();
  std::cout << "WheelMountPoses:\n";
  for (std::size_t i = 0; i < wheel_mount_poses.size(); ++i) {
    std::cout << "  wheel[" << i << "]:";
    for (const double value : wheel_mount_poses[i]) {
      std::cout << ' ' << value;
    }
    std::cout << '\n';
  }

  // 8. 模型信息读取完成后停止 SDK 客户端。
  client.Stop();
  return 0;
}
