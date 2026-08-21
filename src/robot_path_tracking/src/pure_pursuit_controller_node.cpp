#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/tracking_error.hpp"
#include "robot_path_tracking/tracking_math.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PurePursuitControllerNode final : public rclcpp::Node {
 public:
  PurePursuitControllerNode() : Node("pure_pursuit_controller_node") {
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.8);
    target_speed_ = declare_parameter<double>("target_speed", 0.35);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.2);
    goal_tolerance_ = std::max(0.0, declare_parameter<double>("goal_tolerance", 0.10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/reference_path", 1, [this](nav_msgs::msg::Path::SharedPtr msg) { path_ = msg; });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/tracking_cmd_vel", 10);
    error_pub_ = create_publisher<robot_interfaces::msg::TrackingError>("/tracking_error", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { Tick(); });
  }

 private:
  void Tick() {
    if (!odom_ || !path_ || path_->poses.empty()) {
      return;
    }

    const auto& pose = odom_->pose.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - pose.position.x;
      const double dy = path_->poses[i].pose.position.y - pose.position.y;
      const double distance = std::hypot(dx, dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = static_cast<int>(i);
      }
    }

    const auto& nearest_pose = path_->poses[nearest_index].pose;
    const double path_yaw = tf2::getYaw(nearest_pose.orientation);
    const double lateral_error =
        robot_path_tracking::SignedLateralError(nearest_pose.position.x, nearest_pose.position.y,
                                                path_yaw, pose.position.x, pose.position.y);
    const auto& goal = path_->poses.back().pose.position;
    const double distance_to_goal =
        std::hypot(goal.x - pose.position.x, goal.y - pose.position.y);
    if (robot_path_tracking::IsFinalWaypointReached(
            nearest_index, path_->poses.size(), distance_to_goal, goal_tolerance_)) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      PublishError(nearest_index, lateral_error, 0.0, 0.0);
      return;
    }

    const auto target_index =
        SelectTargetIndex(nearest_index, pose.position.x, pose.position.y, yaw);
    const auto& target = path_->poses[target_index].pose.position;
    const double dx = target.x - pose.position.x;
    const double dy = target.y - pose.position.y;
    const double heading_error = robot_path_tracking::NormalizeAngle(std::atan2(dy, dx) - yaw);
    const double curvature = 2.0 * std::sin(heading_error) / std::max(lookahead_distance_, 0.05);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_speed_;
    cmd.angular.z = std::clamp(target_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
    cmd_pub_->publish(cmd);

    PublishError(nearest_index, lateral_error, heading_error, cmd.linear.x);
  }

  size_t SelectTargetIndex(const int nearest_index, const double x, const double y,
                           const double yaw) const {
    size_t first_forward_index = path_->poses.size();
    for (size_t i = static_cast<size_t>(nearest_index); i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - x;
      const double dy = path_->poses[i].pose.position.y - y;
      if (robot_path_tracking::ForwardProjection(yaw, dx, dy) <= 0.0) {
        continue;
      }
      if (first_forward_index == path_->poses.size()) {
        first_forward_index = i;
      }
      if (std::hypot(dx, dy) >= lookahead_distance_) {
        return i;
      }
    }
    if (first_forward_index != path_->poses.size()) {
      return first_forward_index;
    }
    return path_->poses.size() - 1;
  }

  void PublishError(const int nearest_index, const double lateral_error,
                    const double heading_error, const double speed) {
    robot_interfaces::msg::TrackingError error;
    error.header.stamp = now();
    error.header.frame_id = path_->header.frame_id;
    error.lateral_error = lateral_error;
    error.heading_error = heading_error;
    error.target_speed = target_speed_;
    error.actual_speed = speed;
    error.nearest_path_index = nearest_index;
    error.controller_name = "pure_pursuit";
    error_pub_->publish(error);
  }

  double lookahead_distance_;
  double target_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  nav_msgs::msg::Path::SharedPtr path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<robot_interfaces::msg::TrackingError>::SharedPtr error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitControllerNode>());
  rclcpp::shutdown();
  return 0;
}
