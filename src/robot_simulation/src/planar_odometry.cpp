#include "robot_simulation/planar_odometry.hpp"

#include <cmath>

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace robot_simulation {
namespace {

geometry_msgs::msg::Quaternion PlanarQuaternion(
    const geometry_msgs::msg::Quaternion& orientation) {
  tf2::Quaternion input(
      orientation.x, orientation.y, orientation.z, orientation.w);
  const double norm_squared = input.length2();
  if (!std::isfinite(norm_squared) || norm_squared <= 1e-12) {
    geometry_msgs::msg::Quaternion identity;
    identity.w = 1.0;
    return identity;
  }

  input.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(input).getRPY(roll, pitch, yaw);

  tf2::Quaternion planar;
  planar.setRPY(0.0, 0.0, yaw);
  planar.normalize();
  geometry_msgs::msg::Quaternion output;
  output.x = planar.x();
  output.y = planar.y();
  output.z = planar.z();
  output.w = planar.w();
  return output;
}

}  // namespace

nav_msgs::msg::Odometry ProjectOdometryToPlanarFootprint(
    const nav_msgs::msg::Odometry& input, const std::string& odom_frame,
    const std::string& base_frame) {
  nav_msgs::msg::Odometry output = input;
  output.header.frame_id = odom_frame;
  output.child_frame_id = base_frame;
  output.pose.pose.position.z = 0.0;
  output.pose.pose.orientation = PlanarQuaternion(input.pose.pose.orientation);
  return output;
}

}  // namespace robot_simulation
