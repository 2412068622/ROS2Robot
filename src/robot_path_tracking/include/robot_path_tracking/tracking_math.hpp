#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace robot_path_tracking {

inline double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double SignedLateralError(const double path_x, const double path_y,
                                 const double path_yaw, const double robot_x,
                                 const double robot_y) {
  const double dx = path_x - robot_x;
  const double dy = path_y - robot_y;
  return std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;
}

inline double ForwardProjection(const double robot_yaw, const double dx, const double dy) {
  return std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
}

inline bool IsFinalWaypointReached(
    const int nearest_index, const std::size_t path_size, const double distance_to_goal,
    const double goal_tolerance) {
  return path_size > 0 && nearest_index == static_cast<int>(path_size - 1) &&
         distance_to_goal <= std::max(0.0, goal_tolerance);
}

}  // namespace robot_path_tracking
