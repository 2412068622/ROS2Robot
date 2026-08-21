#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class FaultSupervisorNode final : public rclcpp::Node {
 public:
  FaultSupervisorNode() : Node("fault_supervisor_node") {
    auto_clear_ = declare_parameter<bool>("auto_clear", true);
    command_cooldown_ms_ = declare_parameter<int>("command_cooldown_ms", 500);
    startup_grace_ms_ = declare_parameter<int>("startup_grace_ms", 2000);
    started_stamp_ = now();

    state_pub_ = create_publisher<robot_interfaces::msg::RobotState>("/fault_supervisor/state", 10);
    health_sub_ = create_subscription<robot_interfaces::msg::RobotState>(
        "/system_health", 10, [this](robot_interfaces::msg::RobotState::SharedPtr msg) {
          last_health_ = *msg;
          health_seen_ = true;
        });
    enable_estop_client_ =
        create_client<std_srvs::srv::SetBool>("/enable_emergency_stop");
    clear_estop_client_ =
        create_client<std_srvs::srv::SetBool>("/clear_emergency_stop");
    timer_ = create_wall_timer(100ms, [this]() { Tick(); });
  }

 private:
  bool CooldownReady() const {
    if (last_command_stamp_.nanoseconds() == 0) {
      return true;
    }
    return ((now() - last_command_stamp_).nanoseconds() / 1000000) >= command_cooldown_ms_;
  }

  void Tick() {
    if (!health_seen_) {
      SetState("WAITING", "waiting for /system_health", true);
      return;
    }
    if (InStartupGrace() && last_health_.state == "ERROR") {
      SetState("ARMING", "startup grace active: " + last_health_.message, true);
      return;
    }
    if (last_health_.state == "ERROR") {
      fault_active_ = true;
      if (!emergency_stop_requested_ && CooldownReady()) {
        RequestEmergencyStop();
      }
      SetState("FAULT", "emergency stop requested: " + last_health_.message, false);
      return;
    }
    if (fault_active_ && auto_clear_ && HealthAllowsClear()) {
      if (emergency_stop_requested_ && CooldownReady()) {
        RequestEmergencyClear();
      }
      SetState("RECOVERING", "system health allows emergency stop clear", true);
      return;
    }
    SetState("CLEAR", "system health " + last_health_.state, true);
  }

  void RequestEmergencyStop() {
    if (!enable_estop_client_->service_is_ready()) {
      SetState("WAITING", "waiting for /enable_emergency_stop", true);
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    enable_estop_client_->async_send_request(request);
    emergency_stop_requested_ = true;
    last_command_stamp_ = now();
  }

  void RequestEmergencyClear() {
    if (!clear_estop_client_->service_is_ready()) {
      SetState("WAITING", "waiting for /clear_emergency_stop", true);
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    clear_estop_client_->async_send_request(request);
    emergency_stop_requested_ = false;
    fault_active_ = false;
    last_command_stamp_ = now();
  }

  bool InStartupGrace() const {
    return ((now() - started_stamp_).nanoseconds() / 1000000) < startup_grace_ms_;
  }

  bool HealthAllowsClear() const {
    if (last_health_.state == "OK") {
      return true;
    }
    return last_health_.state == "WARN" &&
           last_health_.message.find("emergency stop active") != std::string::npos;
  }

  void SetState(const std::string& state, const std::string& message, const bool recoverable) {
    current_state_ = state;
    current_message_ = message;
    recoverable_ = recoverable;
    PublishState();
  }

  void PublishState() {
    robot_interfaces::msg::RobotState state;
    state.header.stamp = now();
    state.state = current_state_;
    state.previous_state = previous_state_;
    state.active_goal_id = "";
    state.message = current_message_;
    state.recoverable = recoverable_;
    state_pub_->publish(state);
    previous_state_ = current_state_;
  }

  bool auto_clear_ = true;
  int command_cooldown_ms_ = 500;
  int startup_grace_ms_ = 2000;
  bool health_seen_ = false;
  bool fault_active_ = false;
  bool emergency_stop_requested_ = false;
  bool recoverable_ = true;
  std::string current_state_ = "WAITING";
  std::string previous_state_ = "UNKNOWN";
  std::string current_message_ = "waiting for /system_health";
  robot_interfaces::msg::RobotState last_health_;
  rclcpp::Time started_stamp_;
  rclcpp::Time last_command_stamp_;
  rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Subscription<robot_interfaces::msg::RobotState>::SharedPtr health_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_estop_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr clear_estop_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaultSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
