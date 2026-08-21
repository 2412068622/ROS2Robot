#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserFilterNode final : public rclcpp::Node {
 public:
  LaserFilterNode() : Node("laser_filter_node"), diagnostics_(this) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sim/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_link");
    range_min_ = declare_parameter<double>("range_min", 0.08);
    range_max_ = declare_parameter<double>("range_max", 12.0);
    angle_min_ = declare_parameter<double>("angle_min", -3.141592653589793);
    angle_max_ = declare_parameter<double>("angle_max", 3.141592653589793);
    clockwise_ = declare_parameter<bool>("clockwise", false);
    invalid_range_as_inf_ = declare_parameter<bool>("invalid_range_as_inf", true);
    enable_builtin_filter_ = declare_parameter<bool>("enable_builtin_filter", true);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic_, 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { Filter(*msg); });
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    diagnostics_.setHardwareID(frame_id_);
    diagnostics_.add("laser_filter", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "scan normalized");
      stat.add("input_topic", input_topic_);
      stat.add("output_topic", output_topic_);
      stat.add("invalid_ranges", last_invalid_count_);
      stat.add("builtin_filter_enabled", enable_builtin_filter_ ? "true" : "false");
    });
  }

 private:
  void Filter(sensor_msgs::msg::LaserScan scan) {
    scan.header.frame_id = frame_id_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    if (clockwise_) {
      std::reverse(scan.ranges.begin(), scan.ranges.end());
      std::reverse(scan.intensities.begin(), scan.intensities.end());
    }
    int invalid_count = 0;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      auto& range = scan.ranges[i];
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const bool angle_rejected = angle < angle_min_ || angle > angle_max_;
      const bool range_rejected = !std::isfinite(range) || range < range_min_ || range > range_max_;
      if (enable_builtin_filter_ && (range_rejected || angle_rejected)) {
        range = invalid_range_as_inf_ ? std::numeric_limits<float>::infinity() : 0.0F;
        if (i < scan.intensities.size()) {
          scan.intensities[i] = 0.0F;
        }
        ++invalid_count;
      }
    }
    pub_->publish(scan);
    last_invalid_count_ = invalid_count;
    diagnostics_.force_update();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  bool clockwise_;
  bool invalid_range_as_inf_;
  bool enable_builtin_filter_;
  int last_invalid_count_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  diagnostic_updater::Updater diagnostics_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserFilterNode>());
  rclcpp::shutdown();
  return 0;
}
