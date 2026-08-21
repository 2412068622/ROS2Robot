#include <chrono>
#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class VirtualRcNode final : public rclcpp::Node {
 public:
  VirtualRcNode() : Node("virtual_rc_node") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    max_linear_mps_ = declare_parameter<double>("max_linear_mps", 0.5);
    max_angular_radps_ = declare_parameter<double>("max_angular_radps", 1.2);
    manual_takeover_ = declare_parameter<bool>("manual_takeover", false);

    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/rc/joy", 10);
    takeover_pub_ = create_publisher<std_msgs::msg::Bool>("/manual_takeover/state", 10);
    teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/virtual_rc/cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { virtual_cmd_ = *msg; });
    set_takeover_srv_ = create_service<std_srvs::srv::SetBool>(
        "/set_manual_takeover",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          manual_takeover_ = request->data;
          response->success = true;
          response->message = manual_takeover_ ? "manual takeover active" : "manual takeover cleared";
        });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period), [this]() { Tick(); });
  }

 private:
  void Tick() {
    std_msgs::msg::Bool takeover;
    takeover.data = manual_takeover_;
    takeover_pub_->publish(takeover);

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.header.frame_id = "virtual_rc";
    joy.axes = {
        ClampAxis(virtual_cmd_.linear.y / max_linear_mps_),
        ClampAxis(virtual_cmd_.linear.x / max_linear_mps_),
        ClampAxis(virtual_cmd_.angular.z / max_angular_radps_)};
    joy.buttons = {manual_takeover_ ? 1 : 0};
    joy_pub_->publish(joy);

    if (manual_takeover_) {
      geometry_msgs::msg::Twist limited = virtual_cmd_;
      limited.linear.x = Clamp(virtual_cmd_.linear.x, -max_linear_mps_, max_linear_mps_);
      limited.linear.y = Clamp(virtual_cmd_.linear.y, -max_linear_mps_, max_linear_mps_);
      limited.angular.z = Clamp(virtual_cmd_.angular.z, -max_angular_radps_, max_angular_radps_);
      teleop_pub_->publish(limited);
    }
  }

  static double Clamp(const double value, const double low, const double high) {
    return std::min(std::max(value, low), high);
  }

  static float ClampAxis(const double value) {
    return static_cast<float>(Clamp(value, -1.0, 1.0));
  }

  double publish_rate_hz_ = 20.0;
  double max_linear_mps_ = 0.5;
  double max_angular_radps_ = 1.2;
  bool manual_takeover_ = false;
  geometry_msgs::msg::Twist virtual_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_takeover_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualRcNode>());
  rclcpp::shutdown();
  return 0;
}
