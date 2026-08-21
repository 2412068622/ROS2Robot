#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"

namespace robot_simulation {

nav_msgs::msg::Odometry ProjectOdometryToPlanarFootprint(
    const nav_msgs::msg::Odometry& input, const std::string& odom_frame,
    const std::string& base_frame);

}  // namespace robot_simulation
