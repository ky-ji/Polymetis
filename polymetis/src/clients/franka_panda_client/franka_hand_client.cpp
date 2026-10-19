#include "polymetis/clients/franka_hand_client.hpp"

#include "spdlog/spdlog.h"
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <time.h>

#include <grpc/grpc.h>

#include "polymetis.grpc.pb.h"
#include "polymetis/utils.h"

using grpc::ClientContext;

FrankaHandClient::FrankaHandClient(std::shared_ptr<grpc::Channel> channel,
                                   YAML::Node config)
    : stub_(GripperServer::NewStub(channel)) {
  readonly_mode_ = config["readonly"].as<bool>(false);
  const bool home_on_start = config["home_on_start"].as<bool>(false);
  if (readonly_mode_ && home_on_start)
    throw std::invalid_argument("Read-only gripper cannot home on startup");
  // Connect to gripper
  std::string robot_ip = config["robot_ip"].as<std::string>();
  spdlog::info("Connecting to robot_ip {}", robot_ip);
  gripper_.reset(new franka::Gripper(robot_ip));

  // Homing moves the fingers and must be an explicit commissioning choice.
  if (home_on_start) gripper_->homing();
  is_moving_ = false;

  // Initialize server connection
  franka::GripperState franka_gripper_state = gripper_->readOnce();

  GripperMetadata metadata;
  metadata.set_max_width(franka_gripper_state.max_width);
  metadata.set_hz(GRIPPER_HZ);

  ClientContext context;
  context.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(2));
  Empty empty;
  const auto status = stub_->InitRobotClient(&context, metadata, &empty);
  if (!status.ok()) throw std::runtime_error("Gripper InitRobotClient failed: " + status.error_message());

  spdlog::info("Connected.", robot_ip);
}

void FrankaHandClient::getGripperState(void) {
  franka::GripperState franka_gripper_state = gripper_->readOnce();

  gripper_state_.set_width(franka_gripper_state.width);
  gripper_state_.set_is_grasped(franka_gripper_state.is_grasped);
  gripper_state_.set_is_moving(is_moving_);
  gripper_state_.set_prev_command_successful(prev_cmd_successful_);

  // gripper_state.time();  // Use current timestamp instead!
  setTimestampToNow(gripper_state_.mutable_timestamp());
}

bool FrankaHandClient::applyGripperCommand(GripperCommand command) {
  if (readonly_mode_) throw std::runtime_error("Read-only gripper cannot execute a command");

  if (command.grasp()) {
    spdlog::info("Grasping at width {} at speed={}", command.width(),
                 command.speed());
    double eps_inner = (command.epsilon_inner() < 0)
                           ? EPSILON_INNER
                           : command.epsilon_inner();
    double eps_outer = (command.epsilon_outer() < 0)
                           ? EPSILON_OUTER
                           : command.epsilon_outer();
    return gripper_->grasp(command.width(), command.speed(),
                           command.force(), eps_inner, eps_outer);

  } else {
    spdlog::info("Moving to width {} at speed={}", command.width(),
                 command.speed());
    return gripper_->move(command.width(), command.speed());
  }

}

void FrankaHandClient::run(void) {
  const auto period = std::chrono::nanoseconds(1000000000LL / GRIPPER_HZ);
  auto next = std::chrono::steady_clock::now();
  while (true) {
    if (command_future_.valid() &&
        command_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      prev_cmd_successful_ = command_future_.get();
      is_moving_ = false;
    }
    // Run control step
    getGripperState();

    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(100));
    status_ = stub_->ControlUpdate(&context, gripper_state_, &gripper_cmd_);
    if (!status_.ok()) throw std::runtime_error("Gripper ControlUpdate failed: " + status_.error_message());

    if (!readonly_mode_ && !is_moving_) {
      // Skip if command not updated
      const std::int64_t timestamp_ns = gripper_cmd_.timestamp().seconds() * 1000000000LL +
                                        gripper_cmd_.timestamp().nanos();
      if (timestamp_ns > prev_cmd_timestamp_ns_) {
        is_moving_ = true;
        // Copy the command; the feedback loop may replace its protobuf cache.
        command_future_ = std::async(std::launch::async, &FrankaHandClient::applyGripperCommand,
                                    this, gripper_cmd_);
        prev_cmd_timestamp_ns_ = timestamp_ns;
      }
    }

    // Spin once
    next += period;
    const auto now = std::chrono::steady_clock::now();
    if (next < now) next = now + period;
    std::this_thread::sleep_until(next);
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    spdlog::error("Usage: franka_hand_client /path/to/cfg.yaml");
    return 1;
  }
  try {
    YAML::Node config = YAML::LoadFile(argv[1]);

    // Launch client
    std::string control_address = config["control_ip"].as<std::string>() + ":" +
                                  config["control_port"].as<std::string>();
    FrankaHandClient franka_hand_client(
        grpc::CreateChannel(control_address, grpc::InsecureChannelCredentials()),
        config);
    franka_hand_client.run();

    // Termination
    spdlog::info("Wait for shutdown; press CTRL+C to close.");

    return 0;
  } catch (const std::exception& ex) {
    spdlog::error("Gripper client stopped: {}", ex.what());
    return 1;
  }
}
