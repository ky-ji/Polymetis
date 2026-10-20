#include "polymetis/clients/gripper_command.hpp"

#include <iostream>
#include <stdexcept>

void require(bool condition) {
  if (!condition) throw std::runtime_error("Gripper command regression failed");
}

int main() {
  bool successful = true;
  bool moving = true;
  std::promise<bool> rejected;
  auto command = rejected.get_future();
  pollGripperCommand(command, successful, moving);
  require(command.valid() && successful && moving);
  rejected.set_exception(std::make_exception_ptr(franka::CommandException("injected rejection")));
  pollGripperCommand(command, successful, moving);
  require(!command.valid() && !successful && !moving);
  pollGripperCommand(command, successful, moving);
  require(!successful && !moving);

  std::promise<bool> next;
  command = next.get_future();
  moving = true;
  next.set_value(true);
  pollGripperCommand(command, successful, moving);
  require(successful && !moving && !command.valid());

  std::promise<bool> unsuccessful;
  command = unsuccessful.get_future();
  moving = true;
  unsuccessful.set_value(false);
  pollGripperCommand(command, successful, moving);
  require(!successful && !moving);

  std::promise<bool> disconnected;
  command = disconnected.get_future();
  disconnected.set_exception(std::make_exception_ptr(franka::NetworkException("injected disconnect")));
  bool propagated = false;
  try {
    pollGripperCommand(command, successful, moving);
  } catch (const franka::NetworkException&) {
    propagated = true;
  }
  require(propagated);
  std::cout << "PASS: pending, rejection, next command, unsuccessful, network failure\n";
}
