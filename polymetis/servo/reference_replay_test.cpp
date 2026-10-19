// Recorded joint targets only; no hardware, network, or robot model required.
#include "reference.hpp"
#include <fstream>
#include <iostream>
#include <vector>

using Reference = polymetis_runtime::JointReference;
using Vector = Reference::Vector;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  require(argc == 2 || argc == 3, "expected a joint-target fixture and optional lookahead");
  std::ifstream input(argv[1]);
  Vector config[7];
  for (auto& row : config) {
    for (auto& value : row) {
      require(static_cast<bool>(input >> value), "invalid fixture configuration");
      value = static_cast<float>(value);  // Match the TorchScript input boundary.
    }
  }
  std::vector<Vector> targets;
  Vector row;
  while (input >> row[0]) {
    for (int j = 1; j < 7; ++j)
      require(static_cast<bool>(input >> row[j]), "incomplete target");
    for (auto& value : row) value = static_cast<float>(value);
    targets.push_back(row);
  }
  require(!targets.empty(), "no targets");
  Reference reference(config[0], config[1], config[2], config[3], config[4],
                      config[5], config[6], .001, .15, argc==3 ? std::stod(argv[2]) : 0.0);
  for (std::int64_t i = 0; i < 60000; ++i) {
    const auto previous = reference.acceleration;
    reference.step(targets[(i / 30) % targets.size()], i / 10, i * 1000000);
    require(reference.status == 0, "recorded boundary approach terminated the session");
    for (int j = 0; j < 7; ++j) {
      require(reference.q[j] >= config[2][j] - 1e-9 &&
              reference.q[j] <= config[3][j] + 1e-9, "position bound");
      require(std::abs(reference.velocity[j]) <= config[4][j] + 1e-9, "velocity bound");
      require(std::abs(reference.acceleration[j]) <= config[5][j] + 1e-9, "acceleration bound");
      require(std::abs(reference.acceleration[j] - previous[j]) / .001 <= config[6][j] + 1e-6,
              "jerk discontinuity");
    }
  }
  require(reference.boundary_rejections > 0, "fixture missed the boundary rejection branch");
  for (std::int64_t i = 60000; i < 65000; ++i)
    reference.step(config[0], i / 10, i * 1000000);
  require(reference.status == 0, "could not retreat from the boundary");
  for (int j = 0; j < 7; ++j)
    require(std::abs(reference.q[j] - config[0][j]) < 1e-8, "retreat target not reached");
  for (std::int64_t i = 65000; i < 68000; ++i)
    reference.step(config[0], 6499, i * 1000000);
  require(reference.status == 1, "stale-input latch was bypassed");
  std::cout << "60000 replay cycles passed; boundary rejections="
            << reference.boundary_rejections << '\n';
}
