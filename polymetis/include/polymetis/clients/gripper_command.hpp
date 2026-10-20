#pragma once

#include <chrono>
#include <future>

#include <franka/exception.h>
#include <spdlog/spdlog.h>

// A rejected motion command is distinct from a failed feedback connection.
// Consume it once, report failure, and leave subsequent commands to the operator.
inline void pollGripperCommand(std::future<bool>& command, bool& successful,
                               bool& moving) {
  if (!command.valid() ||
      command.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }
  try {
    successful = command.get();
    if (!successful) {
      spdlog::warn("Gripper command did not complete successfully; feedback continues");
    }
  } catch (const franka::CommandException& ex) {
    successful = false;
    spdlog::warn("Gripper command failed; feedback continues: {}", ex.what());
  }
  moving = false;
}
