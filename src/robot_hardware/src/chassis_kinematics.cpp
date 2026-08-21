#include "robot_hardware/chassis_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace robot_hardware {
namespace {

constexpr double kPi = 3.14159265358979323846;

double WheelRpmFromLinear(const double speed_mps, const double wheel_diameter_m) {
  if (wheel_diameter_m <= 0.0) {
    return 0.0;
  }
  return speed_mps / (wheel_diameter_m * kPi) * 60.0;
}

double WheelLinearFromRpm(const double rpm, const double wheel_diameter_m) {
  if (wheel_diameter_m <= 0.0) {
    return 0.0;
  }
  return rpm / 60.0 * wheel_diameter_m * kPi;
}

}  // namespace

std::string NormalizeKinematicsModel(const std::string& model) {
  if (model == "mecanum" || model == "omni") {
    return "mecanum";
  }
  if (model == "ackermann" || model == "acker" || model == "four_ws4wd" ||
      model == "4ws4wd" || model == "four_wheel_steer") {
    return "diff_drive";
  }
  return "diff_drive";
}

PlanarPose IntegratePlanarMotion(
    const PlanarPose& pose, const ChassisCommand& command, const double dt_seconds) {
  const double dt = std::max(0.0, dt_seconds);
  PlanarPose next = pose;
  const double cos_yaw = std::cos(pose.yaw_rad);
  const double sin_yaw = std::sin(pose.yaw_rad);
  next.x_m += (command.linear_x_mps * cos_yaw - command.linear_y_mps * sin_yaw) * dt;
  next.y_m += (command.linear_x_mps * sin_yaw + command.linear_y_mps * cos_yaw) * dt;
  next.yaw_rad += command.angular_z_radps * dt;
  next.yaw_rad = std::atan2(std::sin(next.yaw_rad), std::cos(next.yaw_rad));
  return next;
}

WheelSpeeds EstimateWheelSpeeds(
    const ChassisCommand& command, const std::string& model, const double wheel_diameter_m,
    const double wheel_base_m, const double track_width_m) {
  WheelSpeeds speeds;
  const std::string normalized = NormalizeKinematicsModel(model);
  const double half_track = track_width_m * 0.5;
  const double rotation_radius = (wheel_base_m + track_width_m) * 0.5;

  if (normalized == "mecanum") {
    const double fl = command.linear_x_mps - command.linear_y_mps -
                      rotation_radius * command.angular_z_radps;
    const double fr = command.linear_x_mps + command.linear_y_mps +
                      rotation_radius * command.angular_z_radps;
    const double rl = command.linear_x_mps + command.linear_y_mps -
                      rotation_radius * command.angular_z_radps;
    const double rr = command.linear_x_mps - command.linear_y_mps +
                      rotation_radius * command.angular_z_radps;
    speeds.rpm = {
        WheelRpmFromLinear(fl, wheel_diameter_m), WheelRpmFromLinear(fr, wheel_diameter_m),
        WheelRpmFromLinear(rl, wheel_diameter_m), WheelRpmFromLinear(rr, wheel_diameter_m)};
    return speeds;
  }

  const double left = command.linear_x_mps - half_track * command.angular_z_radps;
  const double right = command.linear_x_mps + half_track * command.angular_z_radps;
  speeds.rpm = {
      WheelRpmFromLinear(left, wheel_diameter_m), WheelRpmFromLinear(right, wheel_diameter_m),
      WheelRpmFromLinear(left, wheel_diameter_m), WheelRpmFromLinear(right, wheel_diameter_m)};
  return speeds;
}

ChassisCommand EstimateCommandFromWheelSpeeds(
    const WheelSpeeds& speeds, const std::string& model, const double wheel_diameter_m,
    const double wheel_base_m, const double track_width_m) {
  ChassisCommand command;
  const std::string normalized = NormalizeKinematicsModel(model);
  const double fl = WheelLinearFromRpm(speeds.rpm[0], wheel_diameter_m);
  const double fr = WheelLinearFromRpm(speeds.rpm[1], wheel_diameter_m);
  const double rl = WheelLinearFromRpm(speeds.rpm[2], wheel_diameter_m);
  const double rr = WheelLinearFromRpm(speeds.rpm[3], wheel_diameter_m);

  if (normalized == "mecanum") {
    const double rotation_radius = (wheel_base_m + track_width_m) * 0.5;
    command.linear_x_mps = (fl + fr + rl + rr) * 0.25;
    command.linear_y_mps = (-fl + fr + rl - rr) * 0.25;
    if (rotation_radius > 0.0) {
      command.angular_z_radps = (-fl + fr - rl + rr) / (4.0 * rotation_radius);
    }
    return command;
  }

  const double left = (fl + rl) * 0.5;
  const double right = (fr + rr) * 0.5;
  command.linear_x_mps = (left + right) * 0.5;
  if (track_width_m > 0.0) {
    command.angular_z_radps = (right - left) / track_width_m;
  }
  return command;
}

}  // namespace robot_hardware
