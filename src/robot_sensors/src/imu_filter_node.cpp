#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuFilterNode final : public rclcpp::Node {
 public:
  ImuFilterNode() : Node("imu_filter_node"), diagnostics_(this) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sim/imu");
    output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_topic_, 10, [this](sensor_msgs::msg::Imu::SharedPtr msg) { Filter(*msg); });
    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);
    diagnostics_.setHardwareID(frame_id_);
    diagnostics_.add("imu_filter", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "imu frame normalized");
      stat.add("input_topic", input_topic_);
      stat.add("output_topic", output_topic_);
      stat.add("frame_id", frame_id_);
    });
  }

 private:
  void Filter(sensor_msgs::msg::Imu imu) {
    imu.header.frame_id = frame_id_;
    pub_->publish(imu);
    diagnostics_.force_update();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  diagnostic_updater::Updater diagnostics_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFilterNode>());
  rclcpp::shutdown();
  return 0;
}
