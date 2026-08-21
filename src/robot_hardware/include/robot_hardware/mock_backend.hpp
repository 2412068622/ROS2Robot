#pragma once

#include "robot_hardware/chassis_backend.hpp"

namespace robot_hardware {

class MockBackend final : public ChassisBackend {
 public:
  MockBackend() = default;

  std::string Name() const override;
  bool Open(std::string* error) override;
  void Close() override;
  bool IsOpen() const override;
  bool WriteCommand(const ChassisCommand& command, std::string* error) override;
  std::optional<std::string> Read(std::string* error) override;
  ChassisCommand LastCommand() const;

 private:
  bool open_ = false;
  ChassisCommand last_command_;
};

}  // namespace robot_hardware

