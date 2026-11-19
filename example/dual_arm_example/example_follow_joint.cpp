// MoveJoint 到预设关节后，以该处的双臂末端位姿为固定中心。
// 每个 10 Hz 周期独立采样 xyz ±0.05 m、RPY 各 ±30°，经 IK 后下发 FollowJoint。
// 参考关节使用当前实测值；不可达目标跳过，Ctrl+C 退出循环并 StopMotion。
// 运行前手动上电并退出拖动；Ctrl+C 请求停止，不改变电源状态。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "modi_sdk/robot_client.hpp"

namespace {
volatile std::sig_atomic_t g_running = 1;

// 标准信号回调只更新标志，由主线程提交停止命令。
void HandleSignal(int) { g_running = 0; }
}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 2 && argc != 3) {
    std::cerr << "用法: " << argv[0] << " <robot_id> [domain_id]\n";
    return 2;
  }
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  std::shared_ptr<modi_sdk::RobotClient> client;
  bool motion_requested = false;
  int exit_code = 0;
  try {
    modi_sdk::RobotClient::Config config;
    config.target_robot_id = argv[1];
    if (argc == 3) {
      std::size_t consumed = 0;
      config.domain_id = std::stoi(argv[2], &consumed);
      if (consumed != std::string(argv[2]).size() || config.domain_id < 0) {
        throw std::runtime_error("domain_id 必须是非负整数");
      }
    }
    config.spin_frequency_hz = 200.0;
    client = std::make_shared<modi_sdk::RobotClient>(config);
    if (!client->Start()) throw std::runtime_error("RobotClient::Start 失败");
    const auto robot = client->Robot();
    const auto motion = robot->Motion();
    const auto state = robot->State();
    const auto algo = robot->Algo();
    if (robot->Model()->GetArmNames().size() != 2) {
      throw std::runtime_error("此示例需要双臂模型");
    }
    const auto modes = robot->State()->GetControlMode();
    if (robot->Manager()->GetSafetyState() != modi_sdk::SafetyState::kNormal ||
        robot->Manager()->GetRobotState() != modi_sdk::RobotState::kRunning ||
        modes.size() != 2 ||
        modes[0] != modi_sdk::ControlMode::kPositionControl ||
        modes[1] != modi_sdk::ControlMode::kPositionControl) {
      throw std::runtime_error("需要已上电、正常状态且处于位置控制模式");
    }
    int status_error = 0;
    const auto initial_status = motion->GetPlannerStatus(&status_error);
    if (status_error != 0 ||
        initial_status == modi_sdk::PlannerStatus::kError ||
        initial_status == modi_sdk::PlannerStatus::kPlanning) {
      throw std::runtime_error("请先结束已有运动，再运行示例");
    }
    constexpr double kDeg2Rad = M_PI / 180.0;
    const std::vector<double> preset_joints = {
        0.0 * kDeg2Rad,
        -30.0 * kDeg2Rad,
        0.0,
        -90.0 * kDeg2Rad,
        0.0,
        30.0 * kDeg2Rad,
        0.0,
        0.0 * kDeg2Rad,
        30.0 * kDeg2Rad,
        0.0,
        90.0 * kDeg2Rad,
        0.0,
        -30.0 * kDeg2Rad,
        0.0,
    };
    const auto names = robot->Model()->GetActuatorJointNames();
    const auto start = state->GetActuatorJointPositions();
    std::vector<double> lower, upper;
    if (start.size() != preset_joints.size() || start.size() != names.size() ||
        robot->Model()->GetEndEffectorNames().size() != 2 ||
        robot->Model()->GetActuatorJointLimits(lower, upper) != 0 ||
        lower.size() != start.size() || upper.size() != start.size()) {
      throw std::runtime_error(
          "需要双臂各 7 个关节、各 1 个末端及有效关节限位");
    }
    constexpr double kMargin = 3.0 * kDeg2Rad;
    for (std::size_t i = 0; i < start.size(); ++i) {
      if (!std::isfinite(start[i]) || !std::isfinite(lower[i]) ||
          !std::isfinite(upper[i]) || start[i] < lower[i] + kMargin ||
          start[i] > upper[i] - kMargin ||
          preset_joints[i] < lower[i] + kMargin ||
          preset_joints[i] > upper[i] - kMargin) {
        throw std::runtime_error(names[i] +
                                 " 的当前或预设位置不在规划安全范围内");
      }
    }
    if (g_running) {
      std::cout << "MoveJoint 到预设双臂关节位置\n";
      motion_requested = true;
      const int ret = motion->MoveJoint(preset_joints, 0.2, 0.5);
      if (ret != 0)
        throw std::runtime_error("MoveJoint 失败，错误码=" +
                                 std::to_string(ret));
    }
    bool preset_reached = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (g_running && std::chrono::steady_clock::now() < deadline) {
      const auto status = motion->GetPlannerStatus(&status_error);
      if (status_error != 0 || status == modi_sdk::PlannerStatus::kError)
        throw std::runtime_error("初始关节运动失败");
      const auto joints = state->GetActuatorJointPositions();
      bool at_target = joints.size() == preset_joints.size();
      for (std::size_t i = 0; at_target && i < joints.size(); ++i) {
        at_target = std::isfinite(joints[i]) &&
                    std::abs(joints[i] - preset_joints[i]) <= 0.5 * kDeg2Rad;
      }
      // 同时检查实测位置，避免将服务返回前的旧 Finished 快照当成到点。
      if (status == modi_sdk::PlannerStatus::kFinished && at_target) {
        preset_reached = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (g_running && !preset_reached)
      throw std::runtime_error("等待初始关节运动超时");
    if (g_running) {
      // 中心只读取一次，后续采样不累加偏移，避免目标随机游走出指定范围。
      const auto center_poses = state->GetEndEffectorPoses();
      if (center_poses.size() != 2)
        throw std::runtime_error("末端中心位姿读取失败");
      for (const auto& pose : center_poses) {
        if (pose.size() != 6 ||
            !std::all_of(pose.begin(), pose.end(),
                         [](double value) { return std::isfinite(value); }))
          throw std::runtime_error("末端中心位姿必须是有限的 xyzRPY");
      }
      std::mt19937 random(std::random_device{}());
      std::uniform_real_distribution<double> position_offset(-0.05, 0.05);
      std::uniform_real_distribution<double> angle_offset(-30.0 * kDeg2Rad,
                                                          30.0 * kDeg2Rad);
      modi_sdk::IkSolveOptions options;
      options.max_position_error = 0.0005;
      options.max_orientation_error = 0.1 * kDeg2Rad;
      // 只按 10 Hz 调度；同步 IK / DDS 超时时不补发积压周期。
      const auto period =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(1.0 / 10.0));
      auto next_update = std::chrono::steady_clock::now();
      auto report_start = next_update;
      std::size_t cycles = 0, accepted = 0, skipped = 0, overruns = 0;
      int last_ik_error = 0;
      std::cout << "10 Hz 随机 xyzRPY -> IK -> FollowJoint；Ctrl+C 停止\n";
      while (g_running) {
        auto target_poses = center_poses;
        for (auto& pose : target_poses) {
          for (std::size_t axis = 0; axis < 3; ++axis)
            pose[axis] += position_offset(random);
          for (std::size_t axis = 3; axis < 6; ++axis)
            pose[axis] += angle_offset(random);
        }
        const auto reference = state->GetActuatorJointPositions();
        if (reference.size() != preset_joints.size() ||
            !std::all_of(reference.begin(), reference.end(),
                         [](double value) { return std::isfinite(value); }))
          throw std::runtime_error("当前关节位置读取失败");
        std::vector<double> target_joints, psi_selected;
        last_ik_error = algo->SolveIkPos(reference, target_poses, options,
                                         target_joints, psi_selected);
        if (!g_running) break;
        ++cycles;
        if (last_ik_error != 0) {
          ++skipped;
        } else {
          bool valid = target_joints.size() == preset_joints.size();
          for (std::size_t i = 0; valid && i < target_joints.size(); ++i) {
            valid = std::isfinite(target_joints[i]) &&
                    target_joints[i] >= lower[i] + kMargin &&
                    target_joints[i] <= upper[i] - kMargin;
          }
          if (!valid) {
            ++skipped;
          } else {
            const int ret = motion->FollowJoint(target_joints, 0.15, 0.5);
            if (ret != 0)
              throw std::runtime_error("FollowJoint 失败，错误码=" +
                                       std::to_string(ret));
            ++accepted;
          }
        }
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - report_start).count();
        if (elapsed >= 1.0) {
          const auto status = motion->GetPlannerStatus(&status_error);
          if (status_error != 0 || status == modi_sdk::PlannerStatus::kError)
            throw std::runtime_error("跟随规划状态异常");
          std::cout << "循环=" << cycles / elapsed << " Hz，接受=" << accepted
                    << "，跳过=" << skipped << "，周期超时=" << overruns
                    << "，最近 IK 错误码=" << last_ik_error << '\n';
          cycles = accepted = skipped = overruns = 0;
          report_start = now;
        }
        next_update += period;
        if (std::chrono::steady_clock::now() > next_update) {
          ++overruns;
          next_update = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_until(next_update);
      }
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    exit_code = 1;
  }
  if (client && motion_requested) {
    const int ret = client->Robot()->Motion()->StopMotion();
    if (ret != 0) {
      std::cerr << "StopMotion 失败，错误码=" << ret << '\n';
      exit_code = 1;
    }
  }
  if (client) client->Stop();
  return exit_code;
}
