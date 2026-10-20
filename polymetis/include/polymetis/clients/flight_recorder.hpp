#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <pthread.h>
#include <stdexcept>
#include <thread>
#include <vector>
#include <franka/exception.h>
#include <franka/robot_state.h>

namespace polymetis_runtime {

inline std::int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct ServoReference {
  std::array<double, 7> q{}, dq{}, ddq{};
  int status = -1;
  bool valid = false;
};

// No allocation, formatting, disk access or locking occurs in record().
// dump() runs only after Robot::control has returned or thrown; the callback
// cannot run while its buffer is being serialized by the SCHED_OTHER writer.
class FlightRecorder {
 public:
  struct Sample {
    franka::RobotState state;
    ServoReference reference;
    std::array<double, 7> commanded{};
    std::array<double, 7> applied{};
    std::int64_t host_monotonic_ns = 0;
    double callback_ms = 0;
    double rpc_ms = 0;
    double period_ms = 0;
    bool complete = false;
  };

  FlightRecorder(std::string directory, std::size_t capacity)
      : directory_(std::move(directory)), samples_(validated_capacity(capacity)) {
    if (!directory_.empty()) std::filesystem::create_directories(directory_);
  }

  void record(const franka::RobotState& state,
              const std::array<double, 7>& commanded,
              const std::array<double, 7>& applied, std::int64_t started,
              double rpc_ms, double period_ms, bool complete, const ServoReference& reference = {}) {
    if (directory_.empty()) return;
    auto& row = samples_[next_];
    row.state = state;
    row.reference = reference;
    row.commanded = commanded;
    row.applied = applied;
    row.host_monotonic_ns = started;
    row.callback_ms = (monotonic_ns() - started) / 1e6;
    row.rpc_ms = rpc_ms;
    row.period_ms = period_ms;
    row.complete = complete;
    next_ = (next_ + 1) % samples_.size();
    count_ = std::min(count_ + 1, samples_.size());
  }

  void dump(const std::string& reason, const std::vector<franka::Record>& vendor_log = {}) {
    if (directory_.empty()) return;
    std::exception_ptr failure;
    std::thread writer([&] {
      try {
        sched_param parameters{};
        if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &parameters) != 0)
          throw std::runtime_error("could not lower flight recorder writer priority");
        const auto stem = directory_ + "/fault-" + std::to_string(monotonic_ns());
        const auto partial = stem + ".partial.jsonl";
        std::ofstream out(partial);
        out.exceptions(std::ios::badbit | std::ios::failbit);
        out << std::setprecision(17)
            << "{\"schema\":\"polymetis.franka.flight/v1\",\"reason\":";
        write_string(out, reason);
        out << ",\"samples\":" << count_ << ",\"clock\":\"host_monotonic_ns\"}\n";
        for (std::size_t n = 0; n < count_; ++n) {
          const auto& row = samples_[(next_ + samples_.size() - count_ + n) % samples_.size()];
          out << "{\"host_monotonic_ns\":" << row.host_monotonic_ns
              << ",\"callback_ms\":" << row.callback_ms << ",\"rpc_ms\":" << row.rpc_ms
              << ",\"period_ms\":" << row.period_ms << ",\"complete\":"
              << (row.complete ? "true" : "false") << ",\"torque_computed\":";
          write_array(out, row.commanded);
          out << ",\"torque_applied\":";
          write_array(out, row.applied);
          if (row.reference.valid) {
            out << ",\"joint_reference\":"; write_array(out, row.reference.q);
            out << ",\"joint_velocity_reference\":"; write_array(out, row.reference.dq);
            out << ",\"joint_acceleration_reference\":"; write_array(out, row.reference.ddq);
            out << ",\"servo_status\":" << row.reference.status;
          }
          out << ",\"robot_state\":" << row.state << "}\n";
        }
        // libfranka's final received state can contain the error that prevented
        // the next callback. Preserve it separately from the callback history.
        for (const auto& row : vendor_log)
          out << "{\"vendor_exception_state\":" << row.state << "}\n";
        out.flush();
        out.close();
        std::filesystem::rename(partial, stem + ".jsonl");
      } catch (...) { failure = std::current_exception(); }
    });
    writer.join();
    if (failure) std::rethrow_exception(failure);
  }

 private:
  static std::size_t validated_capacity(std::size_t capacity) {
    if (capacity == 0 || capacity > 20000)
      throw std::invalid_argument("flight recorder capacity must be in [1, 20000]");
    return capacity;
  }
  template <std::size_t N>
  static void write_array(std::ostream& out, const std::array<double, N>& values) {
    out << '[';
    for (std::size_t i = 0; i < N; ++i) {
      out << (i ? "," : "");
      if (std::isfinite(values[i])) out << values[i];
      else out << "null";
    }
    out << ']';
  }
  static void write_string(std::ostream& out, const std::string& value) {
    out << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << c;
      else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                           << static_cast<unsigned>(c) << std::dec;
      else out << c;
    }
    out << '"';
  }
  std::string directory_;
  std::vector<Sample> samples_;
  std::size_t next_ = 0, count_ = 0;
};
}  // namespace polymetis_runtime
