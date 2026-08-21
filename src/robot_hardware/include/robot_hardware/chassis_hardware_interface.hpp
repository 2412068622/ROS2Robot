#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_hardware/chassis_backend.hpp"
#include "robot_hardware/chassis_system_adapter.hpp"

namespace robot_hardware {

class ChassisHardwareInterface final : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  bool LoadParameters();
  bool ValidateJoints() const;
  ChassisCommand CommandFromWheelVelocityCommands() const;
  void UpdateWheelStates(const ChassisSystemState& state, double period_seconds);

  ChassisBackendConfig backend_config_;
  ChassisSystemAdapterConfig adapter_config_;
  std::string calibration_profile_ = "nominal_reference";
  std::string backend_name_ = "mock";
  std::array<std::string, 2> joint_names_{"left_wheel_joint", "right_wheel_joint"};
  std::array<double, 2> wheel_position_rad_{0.0, 0.0};
  std::array<double, 2> wheel_velocity_radps_{0.0, 0.0};
  std::array<double, 2> wheel_command_radps_{0.0, 0.0};
  std::unique_ptr<ChassisSystemAdapter> adapter_;
};

}  // namespace robot_hardware
