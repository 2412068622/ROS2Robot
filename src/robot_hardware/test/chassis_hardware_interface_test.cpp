#include <optional>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/chassis_hardware_interface.hpp"

namespace robot_hardware {
namespace {

hardware_interface::InterfaceInfo MakeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface_info{};
  interface_info.name = name;
  return interface_info;
}

hardware_interface::ComponentInfo MakeWheelJoint(const std::string& name) {
  hardware_interface::ComponentInfo joint{};
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces.push_back(MakeInterface("velocity"));
  joint.state_interfaces.push_back(MakeInterface("position"));
  joint.state_interfaces.push_back(MakeInterface("velocity"));
  return joint;
}

hardware_interface::HardwareComponentInterfaceParams MakeParams(
    const std::optional<std::string>& chassis_type = std::nullopt) {
  hardware_interface::HardwareComponentInterfaceParams params;
  auto& info = params.hardware_info;
  info.name = "MobileBaseSystem";
  info.type = "system";
  info.hardware_plugin_name = "robot_hardware/ChassisHardwareInterface";
  info.rw_rate = 50;
  info.is_async = false;
  info.thread_priority = 0;
  info.hardware_parameters["backend"] = "mock";
  info.hardware_parameters["wheel_diameter_m"] = "0.15";
  info.hardware_parameters["wheel_base_m"] = "0.42";
  info.hardware_parameters["track_width_m"] = "0.43";
  info.hardware_parameters["calibration_profile"] = "nominal_reference";
  info.hardware_parameters["left_encoder_scale"] = "1.0";
  info.hardware_parameters["right_encoder_scale"] = "1.0";
  info.hardware_parameters["left_direction_sign"] = "1";
  info.hardware_parameters["right_direction_sign"] = "1";
  if (chassis_type.has_value()) {
    info.hardware_parameters["chassis_type"] = *chassis_type;
  }
  info.joints.push_back(MakeWheelJoint("left_wheel_joint"));
  info.joints.push_back(MakeWheelJoint("right_wheel_joint"));
  return params;
}

class ChassisHardwareInterfaceTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ChassisHardwareInterfaceTest, OnInit_MissingChassisType_UsesDiffDriveDefault) {
  ChassisHardwareInterface hardware;

  EXPECT_EQ(hardware.on_init(MakeParams()), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(ChassisHardwareInterfaceTest, OnInit_ExplicitDiffDrive_Succeeds) {
  ChassisHardwareInterface hardware;

  EXPECT_EQ(
      hardware.on_init(MakeParams("diff_drive")), hardware_interface::CallbackReturn::SUCCESS);
}

class InvalidHardwareParameterTest
    : public ChassisHardwareInterfaceTest,
      public ::testing::WithParamInterface<std::pair<std::string, std::string>> {};

TEST_P(InvalidHardwareParameterTest, OnInit_InvalidParameter_ReturnsError) {
  ChassisHardwareInterface hardware;
  auto params = MakeParams("diff_drive");
  params.hardware_info.hardware_parameters[GetParam().first] = GetParam().second;

  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(
    InvalidParameters, InvalidHardwareParameterTest,
    ::testing::Values(
        std::pair<std::string, std::string>{"serial_baud", "115200junk"},
        std::pair<std::string, std::string>{"udp_port", "9000.5"},
        std::pair<std::string, std::string>{"wheel_diameter_m", "0"},
        std::pair<std::string, std::string>{"wheel_base_m", "-0.42"},
        std::pair<std::string, std::string>{"track_width_m", "nan"},
        std::pair<std::string, std::string>{"left_encoder_scale", "1.0junk"},
        std::pair<std::string, std::string>{"right_encoder_scale", "-1"},
        std::pair<std::string, std::string>{"left_direction_sign", "0"},
        std::pair<std::string, std::string>{"left_direction_sign", "not-an-int"},
        std::pair<std::string, std::string>{"right_direction_sign", "2"},
        std::pair<std::string, std::string>{"left_direction_sign", "-1"},
        std::pair<std::string, std::string>{"calibration_profile", ""}));

class UnsupportedChassisTypeTest
    : public ChassisHardwareInterfaceTest,
      public ::testing::WithParamInterface<std::string> {};

TEST_P(UnsupportedChassisTypeTest, OnInit_UnsupportedType_ReturnsError) {
  ChassisHardwareInterface hardware;

  EXPECT_EQ(hardware.on_init(MakeParams(GetParam())), hardware_interface::CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(
    UnsupportedTypes, UnsupportedChassisTypeTest,
    ::testing::Values(
        "mecanum", "omni", "ackermann", "acker", "four_ws4wd", "4ws4wd",
        "four_wheel_steer", "unsupported", "Diff_Drive", " diff_drive", "diff_drive ", ""));

TEST_F(ChassisHardwareInterfaceTest, OnInit_MissingWheelJoint_ReturnsError) {
  ChassisHardwareInterface hardware;
  auto params = MakeParams("diff_drive");
  params.hardware_info.joints.pop_back();

  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ChassisHardwareInterfaceTest, OnInit_ReversedWheelJointOrder_ReturnsError) {
  ChassisHardwareInterface hardware;
  auto params = MakeParams("diff_drive");
  std::swap(params.hardware_info.joints[0], params.hardware_info.joints[1]);

  EXPECT_EQ(hardware.on_init(params), hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace robot_hardware
