#include "robot_hardware/mock_backend.hpp"

namespace robot_hardware {

std::string MockBackend::Name() const { return "mock"; }

bool MockBackend::Open(std::string*) {
  open_ = true;
  return true;
}

void MockBackend::Close() { open_ = false; }

bool MockBackend::IsOpen() const { return open_; }

bool MockBackend::WriteCommand(const ChassisCommand& command, std::string*) {
  last_command_ = command;
  return open_;
}

std::optional<std::string> MockBackend::Read(std::string*) { return std::nullopt; }

ChassisCommand MockBackend::LastCommand() const { return last_command_; }

}  // namespace robot_hardware

