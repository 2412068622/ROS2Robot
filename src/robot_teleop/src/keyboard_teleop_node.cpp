#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {

bool g_shutdown_requested = false;

class TerminalRawMode {
 public:
  TerminalRawMode() {
    enabled_ = isatty(STDIN_FILENO) == 1 && tcgetattr(STDIN_FILENO, &original_) == 0;
    if (!enabled_) {
      return;
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);
  }

  ~TerminalRawMode() {
    if (!enabled_) {
      return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    fcntl(STDIN_FILENO, F_SETFL, original_flags_);
  }

 private:
  bool enabled_ = false;
  termios original_{};
  int original_flags_ = 0;
};

double Clamp(const double value, const double low, const double high) {
  return std::min(std::max(value, low), high);
}

}  // namespace

class KeyboardTeleopNode final : public rclcpp::Node {
 public:
  KeyboardTeleopNode() : Node("keyboard_teleop_node") {
    topic_ = declare_parameter<std::string>("topic", "/virtual_rc/cmd_vel");
    max_linear_mps_ = declare_parameter<double>("max_linear_mps", 0.6);
    max_angular_radps_ = declare_parameter<double>("max_angular_radps", 1.4);
    linear_step_mps_ = declare_parameter<double>("linear_step_mps", 0.1);
    angular_step_radps_ = declare_parameter<double>("angular_step_radps", 0.2);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period), [this]() { Tick(); });

    PrintHelp();
  }

  void PublishStop() {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

 private:
  void Tick() {
    ReadKeys();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_mps_;
    cmd.angular.z = angular_radps_;
    cmd_pub_->publish(cmd);
  }

  void ReadKeys() {
    char key = 0;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      switch (key) {
        case 'w':
        case 'W':
          linear_mps_ = Clamp(linear_mps_ + linear_step_mps_, -max_linear_mps_, max_linear_mps_);
          break;
        case 's':
        case 'S':
          linear_mps_ = Clamp(linear_mps_ - linear_step_mps_, -max_linear_mps_, max_linear_mps_);
          break;
        case 'a':
        case 'A':
          angular_radps_ =
              Clamp(angular_radps_ + angular_step_radps_, -max_angular_radps_, max_angular_radps_);
          break;
        case 'd':
        case 'D':
          angular_radps_ =
              Clamp(angular_radps_ - angular_step_radps_, -max_angular_radps_, max_angular_radps_);
          break;
        case 'x':
        case 'X':
        case ' ':
          linear_mps_ = 0.0;
          angular_radps_ = 0.0;
          break;
        case 'q':
        case 'Q':
          linear_mps_ = 0.0;
          angular_radps_ = 0.0;
          g_shutdown_requested = 1;
          break;
        case 'h':
        case 'H':
        case '?':
          PrintHelp();
          break;
        default:
          break;
      }
    }
  }

  void PrintHelp() const {
    std::cout << "\nKeyboard teleop publishing to " << topic_ << "\n"
              << "  W/S: increase/decrease forward speed\n"
              << "  A/D: turn left/right\n"
              << "  X or Space: stop\n"
              << "  H: help\n"
              << "  Q: stop and quit\n"
              << "Current limits: linear " << max_linear_mps_ << " m/s, angular "
              << max_angular_radps_ << " rad/s\n"
              << std::flush;
  }

  std::string topic_;
  double max_linear_mps_ = 0.6;
  double max_angular_radps_ = 1.4;
  double linear_step_mps_ = 0.1;
  double angular_step_radps_ = 0.2;
  double publish_rate_hz_ = 20.0;
  double linear_mps_ = 0.0;
  double angular_radps_ = 0.0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  TerminalRawMode terminal;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleopNode>();
  rclcpp::WallRate rate(100);
  while (rclcpp::ok() && g_shutdown_requested == 0) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  if (rclcpp::ok()) {
    node->PublishStop();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
  }
  return 0;
}
