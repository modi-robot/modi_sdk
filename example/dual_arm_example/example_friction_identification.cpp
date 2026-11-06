// Usage: dual_arm_example_friction_identification <robot_id> [--write]
// Starts both model arms together; each has an independent status/result.
// Writes are explicitly enabled with --write, after both identifications
// finish.
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "modi_error_codes/name.hpp"
#include "modi_sdk/robot_client.hpp"
namespace {
volatile std::sig_atomic_t running = 1;
void Signal(int) { running = 0; }
}  // namespace
int main(int argc, char** argv) {
  if (argc < 2 || argc > 3 ||
      (argc == 3 && std::string(argv[2]) != "--write")) {
    std::cerr << "Usage: " << argv[0] << " <robot_id> [--write]\n";
    return 2;
  }
  std::signal(SIGINT, Signal);
  std::signal(SIGTERM, Signal);
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) return 1;
  auto robot = client.Robot();
  auto calibration = robot->Calibration();
  const auto arms = robot->Model()->GetArmNames();
  if (arms.size() != 2) {
    std::cerr << "Expected two arms\n";
    return 1;
  }
  std::vector<bool> active(2, false), success(2, false);
  for (std::size_t i = 0; i < arms.size(); ++i) {
    const int ret = calibration->FrictionIdentificationEnable(arms[i]);
    if (ret)
      std::cerr << arms[i] << " start failed: " << ret << '\n';
    else
      active[i] = true;
  }
  bool cancellation_requested = false;
  while (active[0] || active[1]) {
    if (!running && !cancellation_requested) {
      for (std::size_t i = 0; i < arms.size(); ++i)
        if (active[i]) calibration->CancelCalibration(arms[i]);
      cancellation_requested = true;
    }
    for (std::size_t i = 0; i < arms.size(); ++i)
      if (active[i]) {
        auto status = modi_sdk::CalibrationTaskStatus::kIdle;
        const int ret = calibration->GetCalibrationStatus(arms[i], status);
        if (ret) {
          std::cerr << arms[i] << ": " << ret << ' '
                    << modi_error_codes::RemoteErrorCodeNameOrUnknown(ret)
                    << '\n';
          if (status != modi_sdk::CalibrationTaskStatus::kFailed)
            calibration->CancelCalibration(arms[i]);
          active[i] = false;
        } else if (status != modi_sdk::CalibrationTaskStatus::kRunning) {
          active[i] = false;
          success[i] = status == modi_sdk::CalibrationTaskStatus::kSucceeded;
        }
      }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!success[0] || !success[1]) return 1;
  for (const auto& arm : arms) {
    std::vector<std::vector<double>> params;
    std::vector<double> residuals;
    if (calibration->GetFrictionIdentificationResult(arm, params, &residuals)) return 1;
    std::cout << arm << " [Fs,Fc,vs,p,B]\n";
    for (const auto& row : params) {
      for (double x : row) std::cout << x << ' ';
      std::cout << '\n';
    }
    if (argc != 3 || !running) continue;
    for (int stage = 0; stage < 2; ++stage) {
      if (!running) return 1;
      int ret = stage == 0 ? robot->Model()->SetJointFrictionParams(arm, params)
                           : robot->Model()->SetCollisionResiduals(arm, residuals);
      if (ret) return 1;
      auto status = modi_sdk::CalibrationTaskStatus::kRunning;
      bool cancel_sent = false;
      do {
        if (!running && !cancel_sent) {
          calibration->CancelCalibration(arm);
          cancel_sent = true;
        }
        ret = calibration->GetCalibrationStatus(arm, status);
        if (ret) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while (status == modi_sdk::CalibrationTaskStatus::kRunning);
      if (status != modi_sdk::CalibrationTaskStatus::kSucceeded) return 1;
    }
    // The server has already verified the write. Read and display values
    // directly without repeating the protocol scaling in the application.
    std::vector<std::vector<double>> actual;
    if (robot->Model()->GetJointFrictionParams(arm, actual) ||
        actual.size() != params.size())
      return 1;
    std::cout << arm
              << ": write verified by server. Current [Fs,Fc,vs,p,B]:\n";
    for (const auto& row : actual) {
      for (double x : row) std::cout << x << ' ';
      std::cout << '\n';
    }
  }
  return 0;
}
