#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_simulation/planar_odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

class GazeboOdomTfNode final : public rclcpp::Node {
 public:
  GazeboOdomTfNode() : Node("gazebo_odom_tf_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/model/mobile_robot/odometry");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { Publish(*msg); });
  }

 private:
  void Publish(const nav_msgs::msg::Odometry& input) {
    const auto odom = robot_simulation::ProjectOdometryToPlanarFootprint(
        input, odom_frame_, base_frame_);
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
