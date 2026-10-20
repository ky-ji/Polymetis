#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

#include "polymetis/utils.h"
#include <grpcpp/grpcpp.h>
#include <future>
#include <cstdint>

#include <franka/gripper.h>
#include <franka/gripper_state.h>

#define GRIPPER_HZ 30

// Define tolerances to be able to grasp any object without specifying width
#define EPSILON_INNER 0.2
#define EPSILON_OUTER 0.2

class FrankaHandClient {
private:
  void getGripperState(void);
  bool applyGripperCommand(GripperCommand command);

  // gRPC
  std::unique_ptr<GripperServer::Stub> stub_;
  grpc::Status status_;

  GripperState gripper_state_;
  GripperCommand gripper_cmd_;
  std::int64_t prev_cmd_timestamp_ns_ = 0;
  bool prev_cmd_successful_ = true;
  bool readonly_mode_ = false;

  // Franka
  std::shared_ptr<franka::Gripper> gripper_;
  bool is_moving_ = false;
  // Destroy the future first so its worker completes before gripper_ is freed.
  std::future<bool> command_future_;

public:
  FrankaHandClient(std::shared_ptr<grpc::Channel> channel, YAML::Node config);
  void run(void);
};
