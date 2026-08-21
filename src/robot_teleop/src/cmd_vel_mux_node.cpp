#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_core/srv/set_control_mode.hpp"
#include "robot_teleop/cmd_vel_safety.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::Twist ZeroTwist() {
  return robot_teleop::MakeZeroTwist();
}

}  // namespace

class CmdVelMuxNode final : public rclcpp::Node {
 public:
  CmdVelMuxNode() : Node("cmd_vel_mux_node") {
    active_source_ = declare_parameter<std::string>("default_source", "teleop");
    watchdog_timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 500);
    dynamic_angular_limit_radps_ = declare_parameter<double>("dynamic_angular_limit_radps", 0.6);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    active_pub_ = create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);
    estop_state_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop/state", 10);
    takeover_state_pub_ = create_publisher<std_msgs::msg::Bool>("/cmd_vel_mux/manual_takeover", 10);

    AddInput("nav2", "/nav2_cmd_vel");
    AddInput("tracking", "/tracking_cmd_vel");
    AddInput("teleop", "/teleop_cmd_vel");
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
          SetSource(request, response);
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

    timer_ = create_wall_timer(50ms, [this]() { Tick(); });
  }

 private:
  struct InputState {
    geometry_msgs::msg::Twist last_twist;
    rclcpp::Time last_stamp;
    bool received = false;
  };

  void AddInput(const std::string& name, const std::string& topic) {
    inputs_[name] = InputState{ZeroTwist(), now(), false};
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(
        topic, 10, [this, name](geometry_msgs::msg::Twist::SharedPtr msg) {
          inputs_[name].last_twist = *msg;
          inputs_[name].last_stamp = now();
          inputs_[name].received = true;
        }));
  }

  template <typename RequestT, typename ResponseT>
  void SetSource(const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    if (inputs_.find(request->mode) == inputs_.end()) {
      response->success = false;
      response->message = "unknown cmd source: " + request->mode;
      return;
    }
    active_source_ = request->mode;
    response->success = true;
    response->message = "active cmd source set to " + active_source_;
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

  bool WatchdogExpired(const InputState& input) const {
    if (!input.received) {
      return true;
    }
    const auto age_ms = (now() - input.last_stamp).nanoseconds() / 1000000;
    return age_ms > watchdog_timeout_ms_;
  }

  void Tick() {
    std_msgs::msg::String active_msg;
    active_msg.data = robot_teleop::DescribeCommandState(
        emergency_stop_, runtime_safety_stop_, manual_takeover_, active_source_);
    active_pub_->publish(active_msg);
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = emergency_stop_;
    estop_state_pub_->publish(estop_msg);
    std_msgs::msg::Bool takeover_msg;
    takeover_msg.data = manual_takeover_;
    takeover_state_pub_->publish(takeover_msg);

    const std::string selected_source =
        robot_teleop::SelectCommandSource(manual_takeover_, active_source_);
    const auto input_it = inputs_.find(selected_source);
    const bool input_available = input_it != inputs_.end();
    const bool input_fresh = input_available && !WatchdogExpired(input_it->second);

    robot_teleop::CmdVelSafetyGateRequest gate_request;
    gate_request.input_twist = input_available ? input_it->second.last_twist : ZeroTwist();
    gate_request.emergency_stop = emergency_stop_;
    gate_request.runtime_safety_stop = runtime_safety_stop_;
    gate_request.input_available = input_available;
    gate_request.input_fresh = input_fresh;
    gate_request.dynamic_speed_limit_mps = dynamic_speed_limit_mps_;
    gate_request.dynamic_angular_limit_radps = dynamic_angular_limit_radps_;
    const auto gate_decision = robot_teleop::EvaluateCmdVelSafetyGate(gate_request);

    if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputUnavailable) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cmd_vel source '%s' is not registered; publishing zero twist",
                           selected_source.c_str());
    } else if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputStale) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cmd_vel source '%s' watchdog expired; publishing zero twist",
                           selected_source.c_str());
    }
    cmd_pub_->publish(gate_decision.output_twist);
  }

  std::string active_source_;
  int watchdog_timeout_ms_;
  bool emergency_stop_ = false;
  bool manual_takeover_ = false;
  bool runtime_safety_stop_ = false;
  double dynamic_speed_limit_mps_ = 0.0;
  double dynamic_angular_limit_radps_ = 0.6;
  std::unordered_map<std::string, InputState> inputs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subscriptions_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeover_sub_;
  rclcpp::Subscription<robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_state_pub_;
  rclcpp::Service<robot_interfaces_core::srv::SetControlMode>::SharedPtr v2_set_source_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_estop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMuxNode>());
  rclcpp::shutdown();
  return 0;
}
