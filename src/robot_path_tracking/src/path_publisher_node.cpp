#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class PathPublisherNode final : public rclcpp::Node {
 public:
  PathPublisherNode() : Node("path_publisher_node") {
    path_file_ = declare_parameter<std::string>("path_file", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    LoadPath();
    publisher_ = create_publisher<nav_msgs::msg::Path>("/reference_path", 1);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               [this]() { Publish(); });
  }

 private:
  void LoadPath() {
    path_.header.frame_id = frame_id_;
    if (path_file_.empty()) {
      AddPose(0.0, 0.0, 0.0);
      AddPose(1.0, 0.0, 0.0);
      AddPose(2.0, 0.5, 0.2);
      AddPose(3.0, 1.0, 0.2);
      return;
    }

    std::ifstream file(path_file_);
    if (!file) {
      throw std::runtime_error("failed to open path file: " + path_file_);
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line.front() == '#') {
        continue;
      }
      std::istringstream stream(line);
      std::vector<double> values;
      double value = 0.0;
      while (stream >> value) {
        values.push_back(value);
      }
      if (values.size() == 3) {
        AddPose(values[0], values[1], values[2]);
      } else if (values.size() >= 4) {
        AddPose(values[0], values[1], values[3]);
      }
    }
  }

  void AddPose(const double x, const double y, const double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = QuaternionFromYaw(yaw);
    path_.poses.push_back(pose);
  }

  void Publish() {
    path_.header.stamp = now();
    for (auto& pose : path_.poses) {
      pose.header.stamp = path_.header.stamp;
    }
    publisher_->publish(path_);
  }

  std::string path_file_;
  std::string frame_id_;
  double publish_rate_hz_;
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
