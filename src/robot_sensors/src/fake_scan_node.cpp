#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class FakeScanNode final : public rclcpp::Node {
 public:
  FakeScanNode() : Node("fake_scan_node") {
    topic_ = declare_parameter<std::string>("topic", "/scan");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_link");
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(topic_, 10);
    timer_ = create_wall_timer(100ms, [this]() { Publish(); });
  }

 private:
  void Publish() {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = frame_id_;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = 2.0 * M_PI / 360.0;
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;
    scan.range_min = 0.08;
    scan.range_max = 10.0;
    scan.ranges.assign(360, 5.0F);
    for (size_t i = 80; i < 110; ++i) {
      scan.ranges[i] = 1.5F;
    }
    publisher_->publish(scan);
  }

  std::string topic_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeScanNode>());
  rclcpp::shutdown();
  return 0;
}
