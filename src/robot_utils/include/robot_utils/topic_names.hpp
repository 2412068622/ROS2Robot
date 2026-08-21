#pragma once

namespace robot_utils::topics {

constexpr const char* kScan = "/scan";
constexpr const char* kImu = "/imu/data";
constexpr const char* kOdom = "/odom";
constexpr const char* kWheelOdom = "/wheel/odom";
constexpr const char* kFilteredOdom = "/odometry/filtered";
constexpr const char* kCmdVel = "/cmd_vel";
constexpr const char* kNav2CmdVel = "/nav2_cmd_vel";
constexpr const char* kTrackingCmdVel = "/tracking_cmd_vel";
constexpr const char* kTeleopCmdVel = "/teleop_cmd_vel";
constexpr const char* kReferencePath = "/reference_path";
constexpr const char* kTrackingError = "/tracking_error";

}  // namespace robot_utils::topics
