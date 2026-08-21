#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class OdomPathRecorderNode final : public rclcpp::Node {
 public:
  OdomPathRecorderNode() : Node("odom_path_recorder_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom/path");
    max_poses_ = declare_parameter<int>("max_poses", 2000);
    min_distance_m_ = declare_parameter<double>("min_distance_m", 0.02);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, 50, [this](nav_msgs::msg::Odometry::SharedPtr msg) { AddOdom(*msg); });
    pub_ = create_publisher<nav_msgs::msg::Path>(output_topic_, 10);
    reset_srv_ = create_service<std_srvs::srv::Empty>(
        "/reset_odom_path",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {
          poses_.clear();
          have_last_pose_ = false;
          PublishPath();
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(publish_period_ms_), [this]() { PublishPath(); });
  }

 private:
  void AddOdom(const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;

    if (have_last_pose_) {
      const double dx = pose.pose.position.x - last_pose_.pose.position.x;
      const double dy = pose.pose.position.y - last_pose_.pose.position.y;
      if (std::hypot(dx, dy) < min_distance_m_) {
        return;
      }
    }
    have_last_pose_ = true;
    last_pose_ = pose;
    poses_.push_back(pose);
    while (static_cast<int>(poses_.size()) > max_poses_) {
      poses_.pop_front();
    }
  }

  void PublishPath() {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = poses_.empty() ? "odom" : poses_.back().header.frame_id;
    path.poses.assign(poses_.begin(), poses_.end());
    pub_->publish(path);
  }

  std::string input_topic_;
  std::string output_topic_;
  int max_poses_ = 2000;
  double min_distance_m_ = 0.02;
  int publish_period_ms_ = 200;
  bool have_last_pose_ = false;
  geometry_msgs::msg::PoseStamped last_pose_;
  std::deque<geometry_msgs::msg::PoseStamped> poses_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPathRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
