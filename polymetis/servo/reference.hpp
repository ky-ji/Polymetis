#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <ruckig/ruckig.hpp>

namespace polymetis_runtime {

// Position commands arrive asynchronously; only this 1 kHz reference advances
// the impedance setpoint. A fault latches until a new control session is created.
class JointReference {
 public:
  using Vector = std::array<double, 7>;
  JointReference(Vector q, Vector velocity, Vector lower, Vector upper,
                 Vector max_velocity, Vector max_acceleration, Vector max_jerk,
                 double dt, double timeout)
      : q(q), velocity(velocity), lower_(lower), upper_(upper),
        generator_(dt), dt_(dt), timeout_ns_(0) {
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.01 ||
        !std::isfinite(timeout) || timeout < dt || timeout > 1.0)
      throw std::invalid_argument("invalid reference period or timeout");
    timeout_ns_ = static_cast<std::int64_t>(timeout * 1e9);
    for (std::size_t i = 0; i < 7; ++i) {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i] ||
          !std::isfinite(q[i]) || q[i] < lower[i] || q[i] > upper[i] ||
          !std::isfinite(velocity[i]) ||
          !std::isfinite(max_velocity[i]) || max_velocity[i] <= 0 ||
          !std::isfinite(max_acceleration[i]) || max_acceleration[i] <= 0 ||
          !std::isfinite(max_jerk[i]) || max_jerk[i] <= 0 ||
          std::abs(velocity[i]) > max_velocity[i])
        throw std::invalid_argument("invalid initial state or motion limits");
    }
    input_.max_velocity = max_velocity;
    input_.max_acceleration = max_acceleration;
    input_.max_jerk = max_jerk;
    // Teleoperation updates independent joints continuously. Avoid forcing all
    // seven axes to arrive simultaneously at every transient operator sample.
    input_.synchronization = ruckig::Synchronization::None;
    if (!plan(q, false)) throw std::invalid_argument("initial state cannot stop within bounds");
  }

  void step(const Vector& target, std::int64_t sequence, std::int64_t now_ns, bool paused = false) {
    if (last_time_ >= 0 && now_ns < last_time_) latch(5);
    last_time_ = now_ns;
    if (status == 0 && sequence != sequence_) {
      if (sequence < 0 || (sequence_ >= 0 && sequence < sequence_)) {
        latch(2);
      } else {
        sequence_ = sequence;
        received_at_ = now_ns;
        bool valid = true;
        for (std::size_t i = 0; i < 7; ++i)
          valid = valid && std::isfinite(target[i]) && target[i] >= lower_[i] && target[i] <= upper_[i];
        if (!valid) latch(2);
        else if (!plan(target, false)) latch(3);
      }
    }
    if (paused) received_at_ = now_ns;
    if (status == 0 && !paused && received_at_ >= 0 && now_ns - received_at_ >= timeout_ns_) latch(1);
    elapsed_ = std::min(elapsed_ + dt_, trajectory_.get_duration());
    trajectory_.at_time(elapsed_, q, velocity, acceleration);
  }

  Vector q{}, velocity{}, acceleration{};
  // 0 running; 1 stale; 2 invalid/reordered input; 3 infeasible trajectory;
  // 5 clock regression. A fault never automatically re-enables input.
  int status = 0;
  int last_plan_error = 0;

 private:
  bool plan(const Vector& target, bool stop) {
    auto candidate_input = input_;
    candidate_input.current_position = q;
    candidate_input.current_velocity = velocity;
    candidate_input.current_acceleration = acceleration;
    candidate_input.target_position = target;
    candidate_input.target_velocity.fill(0.0);
    candidate_input.target_acceleration.fill(0.0);
    candidate_input.control_interface = stop ? ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
    ruckig::Trajectory<7> candidate;
    const auto result = generator_.calculate(candidate_input, candidate);
    if (result < 0) { last_plan_error = result; return false; }
    // Community Ruckig does not enforce positional limits. Validate the ENTIRE
    // trajectory, including overshoot during reversals, before accepting it.
    const auto extrema = candidate.get_position_extrema();
    for (std::size_t i = 0; i < 7; ++i)
      if (extrema[i].min < lower_[i] - 1e-10 || extrema[i].max > upper_[i] + 1e-10) { last_plan_error = -1024; return false; }
    trajectory_ = candidate;
    elapsed_ = 0;
    return true;
  }

  void latch(int reason) {
    if (status != 0) return;
    status = reason;
    // Prefer a jerk-limited stop. If that stop would cross a bound, finish the
    // previous validated trajectory, whose terminal velocity/acceleration are zero.
    plan(q, true);
  }

  Vector lower_, upper_;
  ruckig::Ruckig<7> generator_;
  ruckig::InputParameter<7> input_;
  ruckig::Trajectory<7> trajectory_;
  double dt_, elapsed_ = 0;
  std::int64_t timeout_ns_, sequence_ = -1, received_at_ = -1, last_time_ = -1;
};
}  // namespace polymetis_runtime
