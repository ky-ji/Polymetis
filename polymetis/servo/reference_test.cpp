#include "reference.hpp"
#include <iostream>
#include <limits>

using polymetis_runtime::JointReference;
using V = JointReference::Vector;
V filled(double value) { V out; out.fill(value); return out; }
void require(bool ok, const char* what) { if (!ok) throw std::runtime_error(what); }
JointReference make() {
  return JointReference(filled(0), filled(0), filled(-1), filled(1), filled(1), filled(3), filled(20), .001, .1);
}
int main() {
  for (double timeout : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(), -1.0, 2.0}) {
    bool rejected = false;
    try {
      JointReference invalid_timeout(filled(0), filled(0), filled(-1), filled(1),
                                     filled(1), filled(3), filled(20), .001, timeout);
    } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid timeout accepted");
  }
  auto servo = make();
  auto previous_acceleration = servo.acceleration;
  double max_jerk = 0;
  for (std::int64_t i = 0; i < 20000; ++i) {
    previous_acceleration = servo.acceleration;
    // Repeated discontinuous operator reversals, changing every 10 ms.
    const double target = i < 1000 ? .5 : .7 * std::sin(i * .001 * 2.0);
    servo.step(filled(target), i / 10, i * 1000000);
    if (servo.status != 0) std::cerr << "i=" << i << " status=" << servo.status << " error=" << servo.last_plan_error << " q=" << servo.q[0] << " v=" << servo.velocity[0] << " a=" << servo.acceleration[0] << std::endl;
    require(servo.status == 0, "unexpected latch during tracking");
    for (std::size_t j = 0; j < 7; ++j) {
      require(std::abs(servo.q[j]) <= 1 + 1e-9, "position bound");
      require(std::abs(servo.velocity[j]) <= 1 + 1e-9, "velocity bound");
      require(std::abs(servo.acceleration[j]) <= 3 + 1e-9, "acceleration bound");
      const double jerk = std::abs(servo.acceleration[j] - previous_acceleration[j]) / .001;
      max_jerk = std::max(max_jerk, jerk);
      require(jerk <= 20.000001, "jerk discontinuity");
    }
  }
  // A producer that disappears must stop and cannot resume by sending again.
  for (std::int64_t i = 20000; i < 25000; ++i) servo.step(filled(.8), 1999, i * 1000000);
  require(servo.status == 1, "missing stale latch");
  require(std::abs(servo.velocity[0]) < 1e-8, "stale input did not stop");
  const auto held = servo.q;
  servo.step(filled(-.8), 3000, 25000000000LL);
  require(servo.status == 1 && servo.q == held, "fault re-enabled without a new session");
  auto invalid = make();
  invalid.step(filled(std::numeric_limits<double>::quiet_NaN()), 0, 0);
  require(invalid.status == 2, "NaN accepted");
  auto out_of_bounds = make();
  out_of_bounds.step(filled(1.1), 0, 0);
  require(out_of_bounds.status == 2, "out-of-range target accepted");
  auto replay = make();
  replay.step(filled(.5), 2, 0);
  replay.step(filled(.5), 1, 1000000);
  require(replay.status == 2, "reordered command accepted");
  // Repeatedly approach both position boundaries and reverse while moving.
  // Reject infeasible replacements without abandoning the last safe plan.
  JointReference bounded(filled(0), filled(0), filled(-.2), filled(.2),
                         filled(.25), filled(.9), filled(6), .001, .1);
  for (std::int64_t i = 0; i < 60000; ++i) {
    const auto previous = bounded.acceleration;
    const double target = std::clamp(.35 * std::sin(i * .001 * 3.7), -.2, .2);
    bounded.step(filled(target), i / 10, i * 1000000);
    require(bounded.status == 0, "boundary approach ended the session");
    for (std::size_t j = 0; j < 7; ++j) {
      require(std::abs(bounded.q[j]) <= .2 + 1e-9, "boundary position escaped");
      require(std::abs(bounded.velocity[j]) <= .25 + 1e-9, "boundary velocity escaped");
      require(std::abs(bounded.acceleration[j]) <= .9 + 1e-9, "boundary acceleration escaped");
      require(std::abs(bounded.acceleration[j] - previous[j]) / .001 <= 6.000001,
              "boundary replan introduced a jerk discontinuity");
    }
  }
  for (std::int64_t i = 60000; i < 65000; ++i)
    bounded.step(filled(0), i / 10, i * 1000000);
  require(bounded.status == 0 && std::abs(bounded.q[0]) < 1e-8,
          "could not retreat after a boundary approach");
  for (std::int64_t i = 65000; i < 68000; ++i)
    bounded.step(filled(0), 6499, i * 1000000);
  require(bounded.status == 1, "boundary handling disabled stale-input protection");
  std::cout << "boundary rejections=" << bounded.boundary_rejections << '\n';
  std::cout << "reference checks passed; max jerk=" << max_jerk << " rad/s^3\n";
}
