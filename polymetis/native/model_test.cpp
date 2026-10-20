#include "polymetis/clients/model_validation.hpp"
#include <franka/model.h>
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <limits>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
template <typename F> void rejects(F fn) {
  bool caught = false;
  try { fn(); } catch (const std::exception&) { caught = true; }
  require(caught, "Invalid model pose accepted");
}

// Independent serial-chain FK: sensor branches must not change the arm flange.
Eigen::Matrix4d expected(const std::array<double, 7>& q) {
  const double half_pi = std::acos(-1.0) / 2;
  const std::array<Eigen::Vector3d, 7> xyz{{{0,0,.333}, {0,0,0}, {0,-.316,0},
      {.0825,0,0}, {-.0825,.384,0}, {0,0,0}, {.088,0,0}}};
  const std::array<double, 7> roll{{0, -half_pi, half_pi, half_pi, -half_pi, half_pi, half_pi}};
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 7; ++i) {
    transform.translate(xyz[i]);
    transform.rotate(Eigen::AngleAxisd(roll[i], Eigen::Vector3d::UnitX()));
    transform.rotate(Eigen::AngleAxisd(q[i], Eigen::Vector3d::UnitZ()));
  }
  transform.translate(Eigen::Vector3d(0, 0, .107));
  return transform.matrix();
}

int main(int argc, char** argv) {
  require(argc == 2, "model_test official-urdf");
  std::ifstream input(argv[1]);
  std::stringstream buffer; buffer << input.rdbuf();
  require(buffer.str().find("accelerometer") != std::string::npos, "Fixture lacks sensor branches");
  franka::Model model(buffer.str());
  franka::RobotState state{};
  const auto identity = Eigen::Matrix4d::Identity().eval();
  std::copy(identity.data(), identity.data()+16, state.F_T_EE.begin());
  std::copy(identity.data(), identity.data()+16, state.EE_T_K.begin());
  state.F_T_EE[14] = .1;
  for (const auto q : {std::array<double,7>{0,0,0,0,0,0,0},
                       std::array<double,7>{.2,-.3,.4,-1.8,.1,1.9,.6},
                       std::array<double,7>{-.5,.2,-.2,-2,.4,1.5,-.1}}) {
    state.q = q;
    const auto flange = model.pose(franka::Frame::kFlange, state);
    require((Eigen::Map<const Eigen::Matrix4d>(flange.data()) - expected(q)).norm() < 1e-8,
            "Flange selected wrong URDF branch");
    const auto ee = model.pose(franka::Frame::kEndEffector, state);
    const Eigen::Matrix4d target = expected(q) * Eigen::Map<const Eigen::Matrix4d>(state.F_T_EE.data());
    std::copy(target.data(), target.data()+16, state.O_T_EE.begin());
    polymetis_runtime::validate_ee_pose(ee, state.O_T_EE);
    const auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
    for (int joint = 0; joint < 7; ++joint) {
      auto perturbed = q; perturbed[joint] += 1e-6;
      const Eigen::Matrix4d moved = expected(perturbed) * Eigen::Map<const Eigen::Matrix4d>(state.F_T_EE.data());
      const Eigen::Vector3d derivative = (moved.block<3,1>(0,3) - target.block<3,1>(0,3)) / 1e-6;
      for (int row = 0; row < 3; ++row)
        require(std::abs(derivative[row] - jacobian[joint*6+row]) < 1e-6, "Wrong EE Jacobian");
    }
    auto wrong = ee; wrong[12] += .01;
    rejects([&] { polymetis_runtime::validate_ee_pose(wrong, state.O_T_EE); });
    wrong = ee; wrong[0] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { polymetis_runtime::validate_ee_pose(wrong, state.O_T_EE); });
  }
}
