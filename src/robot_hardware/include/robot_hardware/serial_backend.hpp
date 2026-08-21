#pragma once

#include <string>

#include "robot_hardware/chassis_backend.hpp"

namespace robot_hardware {

class SerialBackend final : public ChassisBackend {
 public:
  explicit SerialBackend(ChassisBackendConfig config);
  ~SerialBackend() override;

  std::string Name() const override;
  bool Open(std::string* error) override;
  void Close() override;
  bool IsOpen() const override;
  bool WriteCommand(const ChassisCommand& command, std::string* error) override;
  std::optional<std::string> Read(std::string* error) override;

 private:
  ChassisBackendConfig config_;
  int fd_ = -1;
};

}  // namespace robot_hardware

