// Read-only commissioning: no control(), recovery, or settings writes.
#include <franka/robot.h>
#include <franka/model.h>
#include "polymetis/clients/model_validation.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

template <std::size_t N>
void array(std::ostream& out, const std::array<double, N>& values) {
  out << '[';
  for (std::size_t i = 0; i < N; ++i) out << (i ? "," : "") << values[i];
  out << ']';
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: inspect_franka ROBOT_ADDRESS NEW_OUTPUT_DIRECTORY\n"
              << "Stop the existing FCI client first. This command never enables motion.\n";
    return 2;
  }
  try {
    const auto directory = std::filesystem::path(argv[2]);
    if (std::filesystem::exists(directory)) throw std::runtime_error("Output directory already exists");
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    const auto raw_model = robot.getRobotModel();
    auto model = robot.loadModel();
    const auto state = robot.readOnce();
    std::filesystem::create_directories(directory);
    std::ofstream model_file(directory / "robot-model.raw");
    model_file.exceptions(std::ios::badbit | std::ios::failbit);
    model_file << raw_model;
    model_file.close();
    std::ofstream out(directory / "inspection.json");
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << std::setprecision(17) << "{\"schema\":\"polymetis.franka.inspection/v1\","
        << "\"fci_protocol_version\":" << robot.serverVersion()
        << ",\"robot_state\":" << state << ",\"mass_matrix\":";
    array(out, model.mass(state));
    out << ",\"coriolis\":"; array(out, model.coriolis(state));
    out << ",\"gravity\":"; array(out, model.gravity(state));
    out << ",\"O_T_flange\":"; array(out, model.pose(franka::Frame::kFlange, state));
    out << ",\"O_T_ee_model\":"; array(out, model.pose(franka::Frame::kEndEffector, state));
    out << ",\"zero_jacobian\":"; array(out, model.zeroJacobian(franka::Frame::kEndEffector, state));
    out << "}\n";
    out.close();
    polymetis_runtime::validate_ee_pose(model.pose(franka::Frame::kEndEffector, state), state.O_T_EE);
    std::cout << "Read-only model, payload, TCP and state saved to " << directory << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
