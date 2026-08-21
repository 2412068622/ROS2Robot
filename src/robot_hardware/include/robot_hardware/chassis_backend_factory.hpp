#pragma once

#include <memory>
#include <string>

#include "robot_hardware/chassis_backend.hpp"

namespace robot_hardware {

std::unique_ptr<ChassisBackend> CreateChassisBackend(
    const std::string& backend, const ChassisBackendConfig& config);

}  // namespace robot_hardware
