// Copyright (c) Facebook, Inc. and its affiliates.

// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "torch_server_ops.hpp"
#include <istream>
#include <streambuf>
#include <torch/jit.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <torch/csrc/jit/runtime/graph_executor.h>
#include <vector>

extern "C" {

/*
A preallocated chunk of memory, required to convert a char array to an istream.
*/
class membuf : public std::basic_streambuf<char> {
public:
  membuf(const char *p, size_t l) {
    this->setg((char *)p, (char *)p, (char *)p + l);
  }

  pos_type seekoff(off_type off, std::ios_base::seekdir dir,
                   std::ios_base::openmode which = std::ios_base::in) override {
    if (dir == std::ios_base::cur)
      gbump(off);
    else if (dir == std::ios_base::end)
      setg(eback(), egptr() + off, egptr());
    else if (dir == std::ios_base::beg)
      setg(eback(), eback() + off, egptr());
    return gptr() - eback();
  }

  pos_type seekpos(pos_type sp, std::ios_base::openmode which) override {
    return seekoff(sp - pos_type(off_type(0)), std::ios_base::beg, which);
  }
};

/**
TODO
*/
class memstream : public std::istream {
public:
  memstream(const char *p, size_t l) : std::istream(&_buffer), _buffer(p, l) {
    rdbuf(&_buffer);
  }

private:
  membuf _buffer;
};

struct TorchTensor {
  torch::Tensor data;
};
struct TorchScriptModule {
  torch::jit::script::Module data;
};
struct TorchInput {
  std::vector<torch::jit::IValue> data;
};

struct StateDict {
  c10::Dict<std::string, torch::Tensor> data;
};

TorchRobotState::TorchRobotState(int num_dofs) {
  num_dofs_ = num_dofs;

  // Create initial state dictionary
  rs_timestamp_ = new TorchTensor{torch::zeros(2).to(torch::kInt32)};
  rs_joint_positions_ = new TorchTensor{torch::zeros(num_dofs)};
  rs_joint_velocities_ = new TorchTensor{torch::zeros(num_dofs)};
  rs_motor_torques_measured_ = new TorchTensor{torch::zeros(num_dofs)};
  rs_motor_torques_external_ = new TorchTensor{torch::zeros(num_dofs)};

  state_dict_ = new StateDict{c10::Dict<std::string, torch::Tensor>()};

  state_dict_->data.insert("timestamp", rs_timestamp_->data);
  state_dict_->data.insert("joint_positions", rs_joint_positions_->data);
  state_dict_->data.insert("joint_velocities", rs_joint_velocities_->data);
  state_dict_->data.insert("motor_torques_measured",
                           rs_motor_torques_measured_->data);
  state_dict_->data.insert("motor_torques_external",
                           rs_motor_torques_external_->data);

  input_ = new TorchInput{std::vector<torch::jit::IValue>()};
  input_->data.push_back(state_dict_->data);
}

TorchRobotState::~TorchRobotState() {
  delete rs_timestamp_;
  delete rs_joint_positions_;
  delete rs_joint_velocities_;
  delete rs_motor_torques_measured_;
  delete rs_motor_torques_external_;
  delete state_dict_;
  delete input_;
}

void TorchRobotState::update_state(int timestamp_s, int timestamp_ns,
                                   std::vector<float> joint_positions,
                                   std::vector<float> joint_velocities,
                                   std::vector<float> motor_torques_measured,
                                   std::vector<float> motor_torques_external) {
  rs_timestamp_->data[0] = timestamp_s;
  rs_timestamp_->data[1] = timestamp_ns;
  for (int i = 0; i < joint_positions.size(); i++) {
    rs_joint_positions_->data[i] = joint_positions[i];
    rs_joint_velocities_->data[i] = joint_velocities[i];
    rs_motor_torques_measured_->data[i] = motor_torques_measured[i];
    rs_motor_torques_external_->data[i] = motor_torques_external[i];
  }
}

TorchScriptedController::TorchScriptedController(
    char *data, size_t size, TorchRobotState &init_robot_state) {
  memstream stream(data, size);
  module_ = new TorchScriptModule{torch::jit::load(stream)};

  param_dict_input_ = new TorchInput{std::vector<torch::jit::IValue>()};
  empty_input_ = new TorchInput{std::vector<torch::jit::IValue>()};

  // Warm up controller (TorchScript models take time to compile during first 2
  // queries)
  this->warmup_controller(WARM_UP_ITERS, init_robot_state);
}

TorchScriptedController::~TorchScriptedController() {
  delete module_;
  delete param_dict_input_;
  delete empty_input_;
}

std::vector<float> TorchScriptedController::forward(TorchRobotState &input) {
  torch::NoGradGuard no_grad;
  // An unseen safety branch must not trigger JIT profiling or specialization
  // inside a realtime callback. Warmup cannot exercise every runtime branch.
  torch::jit::GraphOptimizerEnabledGuard no_optimization(false);
  // Step controller & generate torque command response
  c10::Dict<torch::jit::IValue, torch::jit::IValue> controller_state_dict =
      module_->data.forward(input.input_->data).toGenericDict();

  torch::jit::IValue key = torch::jit::IValue("joint_torques");
  torch::Tensor desired_torque = controller_state_dict.at(key).toTensor();

  has_servo_reference_ = controller_state_dict.contains("joint_reference");
  if (has_servo_reference_) {
    const char* keys[] = {"joint_reference", "joint_velocity_reference", "joint_acceleration_reference"};
    for (int part = 0; part < 3; ++part) {
      auto values = controller_state_dict.at(keys[part]).toTensor();
      TORCH_CHECK(values.numel() == 7, "servo reference must have seven entries");
      for (int i = 0; i < 7; ++i) servo_reference_[part * 7 + i] = values[i].item<float>();
    }
    servo_reference_[21] = controller_state_dict.at("servo_status").toTensor()[0].item<float>();
  }

  std::vector<float> result;
  for (int i = 0; i < input.num_dofs_; i++) {
    result.push_back(desired_torque[i].item<float>());
  }
  return result;
}

void TorchScriptedController::warmup_controller(
    int warmup_iters, TorchRobotState &init_robot_state) {
  // Backup
  auto tmp_module_data = module_->data.deepcopy();

  // Warmup
  for (int i = 0; i < warmup_iters; i++) {
    this->forward(init_robot_state);
  }

  // Reload
  module_->data = tmp_module_data;
}

bool TorchScriptedController::is_terminated() {
  torch::jit::GraphOptimizerEnabledGuard no_optimization(false);
  return module_->data.get_method("is_terminated")(empty_input_->data).toBool();
}

void TorchScriptedController::reset() {
  torch::jit::GraphOptimizerEnabledGuard no_optimization(false);
  module_->data.get_method("reset")(empty_input_->data);
}

bool TorchScriptedController::param_dict_load(char *data, size_t size) {
  torch::NoGradGuard no_grad;
  torch::jit::GraphOptimizerEnabledGuard no_optimization(false);
  memstream model_stream(data, size);

  torch::jit::script::Module param_dict_container;
  try {
    param_dict_container = torch::jit::load(model_stream);
  } catch (const c10::Error &e) {
    std::cerr << "error loading the param container:\n";
    std::cerr << e.msg() << std::endl;
    return false;
  }

  // Create controller update input dict
  param_dict_input_->data.clear();
  param_dict_input_->data.push_back(
      param_dict_container.forward(empty_input_->data));

  return true;
}

void TorchScriptedController::param_dict_update_module() {
  // This method executes with the control mutex held. A first-use JIT
  // specialization here blocks the 1 kHz control callback.
  torch::NoGradGuard no_grad;
  torch::jit::GraphOptimizerEnabledGuard no_optimization(false);
  module_->data.get_method("update")(param_dict_input_->data);
}

void TorchScriptedController::update_joint_target(const std::array<float, 7>& target,
                                                  int64_t sequence, bool paused) {
  torch::NoGradGuard no_grad;
  auto position = module_->data.attr("joint_pos_desired").toTensor();
  TORCH_CHECK(position.numel() == 7 && position.scalar_type() == torch::kFloat32 && position.is_contiguous(),
              "joint servo requires seven contiguous float32 targets");
  std::copy(target.begin(), target.end(), position.data_ptr<float>());
  module_->data.attr("command_sequence").toTensor().fill_(sequence);
  module_->data.attr("command_paused").toTensor().fill_(paused ? 1.0 : 0.0);
}

} /* extern "C" */
