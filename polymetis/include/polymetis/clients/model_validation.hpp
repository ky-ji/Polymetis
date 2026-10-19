#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

namespace polymetis_runtime {
// Compare the client model with the independently reported controller pose.
// A model that selects an accelerometer branch must never reach control().
inline void validate_ee_pose(const std::array<double, 16>& calculated,
                             const std::array<double, 16>& reported) {
  double translation_squared = 0.0;
  for (std::size_t i = 0; i < 16; ++i) {
    if (!std::isfinite(calculated[i]) || !std::isfinite(reported[i]))
      throw std::runtime_error("Non-finite model/controller end-effector pose");
    const double error = calculated[i] - reported[i];
    if (i >= 12 && i <= 14) translation_squared += error * error;
    else if (std::abs(error) > 0.001)
      throw std::runtime_error("Model/controller end-effector rotation mismatch");
  }
  if (translation_squared > 0.001 * 0.001)
    throw std::runtime_error("Model/controller end-effector translation mismatch (>1 mm)");
}
}  // namespace polymetis_runtime
