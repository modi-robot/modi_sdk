// Usage: single_arm_example_friction_identification <robot_id> <arm_name>
// [--write] Requires an already powered, stationary robot in position control.
// Ctrl-C requests a controlled cancellation. --write explicitly opts into SDO
// writes.
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
int Await(const std::shared_ptr<modi_sdk::CalibrationApi>& api,
          const std::string& arm) {
  bool cancelled = false;
  while (true) {
    if (!running && !cancelled) {
      const int ret = api->CancelCalibration(arm);
      if (ret) return ret;
      cancelled = true;
    }
    auto status = modi_sdk::CalibrationTaskStatus::kIdle;
    const int ret = api->GetCalibrationStatus(arm, status);
    if (ret) {
      std::cerr << arm << ": " << ret << ' '
                << modi_error_codes::RemoteErrorCodeNameOrUnknown(ret) << '\n';
      return ret;
    }
    if (status == modi_sdk::CalibrationTaskStatus::kSucceeded) return 0;
    if (status == modi_sdk::CalibrationTaskStatus::kCancelled) return 1;
    if (status != modi_sdk::CalibrationTaskStatus::kRunning) return 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
}  // namespace
int main(int argc, char** argv) {
  if (argc < 3 || argc > 4 ||
      (argc == 4 && std::string(argv[3]) != "--write")) {
    std::cerr << "Usage: " << argv[0] << " <robot_id> <arm_name> [--write]\n";
    return 2;
  }
  std::signal(SIGINT, Signal);
  std::signal(SIGTERM, Signal);
  modi_sdk::RobotClient::Config config;
  config.target_robot_id = argv[1];
  modi_sdk::RobotClient client(config);
  if (!client.Start()) return 1;
  auto calibration = client.Robot()->Calibration();
  auto model = client.Robot()->Model();
  const std::string arm = argv[2];
  int ret = calibration->FrictionIdentificationEnable(arm);
  if (ret) {
    std::cerr << "start: " << ret << '\n';
    return 1;
  }
  if (Await(calibration, arm)) return 1;
  std::vector<std::vector<double>> params;
  std::vector<double> residuals;
  if (calibration->GetFrictionIdentificationResult(arm, params, &residuals)) return 1;
  std::cout << arm << " [Fs,Fc,vs,p,B]\n";
  for (const auto& row : params) {
    for (double x : row) std::cout << x << ' ';
    std::cout << '\n';
  }
  if (argc != 4 || !running) return 0;
  ret = model->SetJointFrictionParams(arm, params);
  if (ret || Await(calibration, arm)) return 1;
  if (!running) return 1;
  ret = model->SetCollisionResiduals(arm, residuals);
  if (ret || Await(calibration, arm)) return 1;
  // kSucceeded means the server has verified the write by reading it back.
  // SDK callers use parameter values directly; protocol scaling stays below.
  std::vector<std::vector<double>> actual;
  if (model->GetJointFrictionParams(arm, actual) ||
      actual.size() != params.size())
    return 1;
  std::cout << "Write verified by server. Current [Fs,Fc,vs,p,B]:\n";
  for (const auto& row : actual) {
    for (double x : row) std::cout << x << ' ';
    std::cout << '\n';
  }
  return 0;
}
