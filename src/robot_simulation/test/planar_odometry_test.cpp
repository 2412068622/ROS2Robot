#include "robot_simulation/planar_odometry.hpp"

#include <cmath>

#include <gtest/gtest.h>
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace robot_simulation {
namespace {

geometry_msgs::msg::Quaternion QuaternionFromRpy(
    const double roll, const double pitch, const double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion output;
  output.x = quaternion.x();
  output.y = quaternion.y();
  output.z = quaternion.z();
  output.w = quaternion.w();
  return output;
}

TEST(PlanarOdometryTest, ProjectOdometryToPlanarFootprint_ResetsHeightRollAndPitch) {
  nav_msgs::msg::Odometry input;
  input.header.frame_id = "world";
  input.child_frame_id = "base_link";
  input.pose.pose.position.x = 1.2;
  input.pose.pose.position.y = -0.4;
  input.pose.pose.position.z = 0.075;
  input.pose.pose.orientation = QuaternionFromRpy(0.3, -0.2, 1.1);
  input.pose.covariance[0] = 0.25;
  input.twist.twist.linear.x = 0.6;

  const auto output = ProjectOdometryToPlanarFootprint(input, "odom", "base_footprint");

  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.child_frame_id, "base_footprint");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, input.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, input.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], input.pose.covariance[0]);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, input.twist.twist.linear.x);

  tf2::Quaternion projected(
      output.pose.pose.orientation.x, output.pose.pose.orientation.y,
      output.pose.pose.orientation.z, output.pose.pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(projected).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 1e-12);
  EXPECT_NEAR(pitch, 0.0, 1e-12);
  EXPECT_NEAR(yaw, 1.1, 1e-12);
}

TEST(PlanarOdometryTest, ProjectOdometryToPlanarFootprint_InvalidQuaternionUsesIdentity) {
  nav_msgs::msg::Odometry input;

  const auto output = ProjectOdometryToPlanarFootprint(input, "odom", "base_footprint");

  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.w, 1.0);
}

}  // namespace
}  // namespace robot_simulation
