#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace robot_utils {

inline double SecondsBetween(const rclcpp::Time& newer, const rclcpp::Time& older) {
  return (newer - older).seconds();
}

inline std::chrono::milliseconds SecondsToMilliseconds(const double seconds) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(seconds));
}

}  // namespace robot_utils
