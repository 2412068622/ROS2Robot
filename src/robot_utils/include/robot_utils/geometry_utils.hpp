#pragma once

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace robot_utils {

inline double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace robot_utils
