#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_core/srv/set_control_mode.hpp"
#include "robot_teleop/cmd_vel_safety.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class CmdVelSafetyGateNode final : public rclcpp::Node {
 public:
  CmdVelSafetyGateNode() : Node("cmd_vel_safety_gate_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/muxed_cmd_vel");
    active_source_hint_ = declare_parameter<std::string>("default_source", "twist_mux");
    watchdog_timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 500);
    dynamic_angular_limit_radps_ =
        declare_parameter<double>("dynamic_angular_limit_radps", 0.6);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    active_pub_ = create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);
    estop_state_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop/state", 10);
    takeover_state_pub_ = create_publisher<std_msgs::msg::Bool>("/cmd_vel_mux/manual_takeover", 10);
    takeover_lock_pub_ =
        create_publisher<std_msgs::msg::Bool>("/cmd_vel_mux/manual_takeover_lock", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          last_twist_ = *msg;
          last_stamp_ = now();
          received_ = true;
        });
    takeover_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/manual_takeover/state", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) { manual_takeover_ = msg->data; });
    safety_sub_ = create_subscription<robot_interfaces::msg::SafetyState>(
        "/safety_state", 10, [this](robot_interfaces::msg::SafetyState::SharedPtr msg) {
          runtime_safety_stop_ = msg->safety_stop;
          dynamic_speed_limit_mps_ = msg->effective_speed_limit_mps;
        });

    v2_set_source_srv_ = create_service<robot_interfaces_core::srv::SetControlMode>(
        "/v2/set_cmd_source",
        [this](const std::shared_ptr<robot_interfaces_core::srv::SetControlMode::Request> request,
               std::shared_ptr<robot_interfaces_core::srv::SetControlMode::Response> response) {
          SetSourceHint(request, response);
        });
    enable_estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/enable_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          SetEmergencyStop(request, response, true);
        });
    clear_estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/clear_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          SetEmergencyStop(request, response, false);
        });

    last_stamp_ = now();
    timer_ = create_wall_timer(50ms, [this]() { Tick(); });
  }

 private:
  template <typename RequestT, typename ResponseT>
  void SetSourceHint(const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    if (request->mode != "nav2" && request->mode != "tracking" && request->mode != "teleop" &&
        request->mode != "twist_mux") {
      response->success = false;
      response->message = "unknown cmd source: " + request->mode;
      return;
    }
    active_source_hint_ = request->mode;
    response->success = true;
    response->message =
        "cmd source hint set to " + active_source_hint_ + "; twist_mux still arbitrates by priority";
  }

  void SetEmergencyStop(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response, const bool target_state) {
    if (target_state) {
      emergency_stop_ = request->data;
    } else if (request->data) {
      emergency_stop_ = false;
    }
    response->success = true;
    response->message = emergency_stop_ ? "emergency stop active" : "emergency stop cleared";
  }

  bool WatchdogExpired() const {
    if (!received_) {
      return true;
    }
    const auto age_ms = (now() - last_stamp_).nanoseconds() / 1000000;
    return age_ms > watchdog_timeout_ms_;
  }

  void PublishState() {
    std_msgs::msg::String active_msg;
    const std::string state_hint =
        active_source_hint_ == "twist_mux" ? "twist_mux" : "twist_mux:" + active_source_hint_;
    active_msg.data = robot_teleop::DescribeCommandState(
        emergency_stop_, runtime_safety_stop_, manual_takeover_, state_hint);
    active_pub_->publish(active_msg);

    std_msgs::msg::Bool estop_msg;
    estop_msg.data = emergency_stop_;
    estop_state_pub_->publish(estop_msg);

    std_msgs::msg::Bool takeover_msg;
    takeover_msg.data = manual_takeover_;
    takeover_state_pub_->publish(takeover_msg);
    takeover_lock_pub_->publish(takeover_msg);
  }

  void Tick() {
    PublishState();

    robot_teleop::CmdVelSafetyGateRequest gate_request;
    gate_request.input_twist = received_ ? last_twist_ : robot_teleop::MakeZeroTwist();
    gate_request.emergency_stop = emergency_stop_;
    gate_request.runtime_safety_stop = runtime_safety_stop_;
    gate_request.input_available = received_;
    gate_request.input_fresh = !WatchdogExpired();
    gate_request.dynamic_speed_limit_mps = dynamic_speed_limit_mps_;
    gate_request.dynamic_angular_limit_radps = dynamic_angular_limit_radps_;
    const auto gate_decision = robot_teleop::EvaluateCmdVelSafetyGate(gate_request);

    if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputUnavailable) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no muxed cmd_vel received on '%s'; publishing zero twist",
                           input_topic_.c_str());
    } else if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputStale) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "muxed cmd_vel on '%s' watchdog expired; publishing zero twist",
                           input_topic_.c_str());
    }
    cmd_pub_->publish(gate_decision.output_twist);
  }

  std::string input_topic_;
  std::string active_source_hint_;
  int watchdog_timeout_ms_;
  bool emergency_stop_ = false;
  bool manual_takeover_ = false;
  bool runtime_safety_stop_ = false;
  bool received_ = false;
  double dynamic_speed_limit_mps_ = 0.0;
  double dynamic_angular_limit_radps_ = 0.6;
  rclcpp::Time last_stamp_;
  geometry_msgs::msg::Twist last_twist_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeover_sub_;
  rclcpp::Subscription<robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_lock_pub_;
  rclcpp::Service<robot_interfaces_core::srv::SetControlMode>::SharedPtr v2_set_source_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_estop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
