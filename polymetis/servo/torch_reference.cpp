#include "reference.hpp"
#include <chrono>
#include <torch/custom_class.h>
#include <torch/script.h>

namespace {
class TorchJointReference : public torch::CustomClassHolder {
 public:
  explicit TorchJointReference(torch::Tensor config)
      : config_(validate(config)), engine_(vector(0), vector(7), vector(14), vector(21),
                                          vector(28), vector(35), vector(42),
                                          config_[49].item<double>(), config_[50].item<double>()),
        output_(torch::zeros({4, 7}, torch::kFloat32)) {}

  torch::Tensor step(torch::Tensor target, std::int64_t sequence, bool paused) {
    TORCH_CHECK(target.device().is_cpu() && target.scalar_type() == torch::kFloat32 &&
                target.dim() == 1 && target.size(0) == 7 && target.is_contiguous(),
                "joint target must be a contiguous CPU float32 vector of length seven");
    polymetis_runtime::JointReference::Vector q;
    const auto* data = target.data_ptr<float>();
    std::copy(data, data + 7, q.begin());
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    engine_.step(q, sequence, now, paused);
    auto* result = output_.data_ptr<float>();
    for (std::size_t i = 0; i < 7; ++i) {
      result[i] = engine_.q[i];
      result[7 + i] = engine_.velocity[i];
      result[14 + i] = engine_.acceleration[i];
      result[21 + i] = engine_.status;
    }
    started_ = true;
    return output_;
  }

  torch::Tensor initial_state() const {
    // Upload an unstarted controller. Serializing a running OTG as a fresh one
    // would reset its acceleration and watchdog state, so explicitly reject it.
    TORCH_CHECK(!started_, "cannot serialize a running joint reference; create a new controller");
    return config_.clone();
  }

  bool terminated() const {
    // Keep safety transitions free of first-use Tensor kernels in the 1 kHz
    // callback. This method is exercised on every normal and faulted cycle.
    return engine_.status != 0 && std::all_of(engine_.velocity.begin(), engine_.velocity.end(),
        [](double velocity) { return std::abs(velocity) < 1e-6; });
  }

 private:
  static torch::Tensor validate(torch::Tensor config) {
    TORCH_CHECK(config.device().is_cpu() && config.dim() == 1 && config.size(0) == 51,
                "joint reference configuration must contain 51 CPU scalars");
    return config.to(torch::kFloat64).contiguous().clone();
  }
  polymetis_runtime::JointReference::Vector vector(std::size_t offset) {
    polymetis_runtime::JointReference::Vector out;
    const auto* data = config_.data_ptr<double>() + offset;
    std::copy(data, data + 7, out.begin());
    return out;
  }
  torch::Tensor config_;
  polymetis_runtime::JointReference engine_;
  torch::Tensor output_;
  bool started_ = false;
};
}  // namespace

TORCH_LIBRARY(polymetis_servo, module) {
  module.class_<TorchJointReference>("JointReference")
      .def(torch::init<torch::Tensor>())
      .def("step", &TorchJointReference::step)
      .def("terminated", &TorchJointReference::terminated)
      .def_pickle(
          [](const c10::intrusive_ptr<TorchJointReference>& self) { return self->initial_state(); },
          [](torch::Tensor config) { return c10::make_intrusive<TorchJointReference>(config); });
}
