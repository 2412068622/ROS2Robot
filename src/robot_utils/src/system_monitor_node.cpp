#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/chassis_state.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace {

diagnostic_msgs::msg::KeyValue MakeKeyValue(const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string BoolString(const bool value) { return value ? "true" : "false"; }

}  // namespace

class SystemMonitorNode final : public rclcpp::Node {
 public:
  SystemMonitorNode() : Node("system_monitor_node"), diagnostics_(this) {
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 1000);
    low_battery_voltage_ = declare_parameter<double>("low_battery_voltage", 22.0);
    critical_battery_voltage_ = declare_parameter<double>("critical_battery_voltage", 20.0);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);

    diagnostics_.setHardwareID("robot");
    diagnostics_.setPeriod(static_cast<double>(std::max(50, publish_period_ms_)) / 1000.0);
    diagnostics_.add("system/chassis", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckChassis(), stat);
    });
    diagnostics_.add("system/battery", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckBattery(), stat);
    });
    diagnostics_.add(
        "system/emergency_stop", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
          FillDiagnosticStatus(CheckEmergencyStop(), stat);
        });
    diagnostics_.add("system/scan", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckTopicFreshness("system/scan", scan_seen_, scan_stamp_), stat);
    });
    diagnostics_.add("system/imu", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckTopicFreshness("system/imu", imu_seen_, imu_stamp_), stat);
    });
    diagnostics_.add("system/odom", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckTopicFreshness("system/odom", odom_seen_, odom_stamp_), stat);
    });
    health_pub_ = create_publisher<robot_interfaces::msg::RobotState>("/system_health", 10);

    chassis_sub_ = create_subscription<robot_interfaces::msg::ChassisState>(
        "/chassis/state", 10, [this](robot_interfaces::msg::ChassisState::SharedPtr msg) {
          chassis_state_ = *msg;
          chassis_seen_ = true;
          chassis_stamp_ = now();
        });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop/state", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
          emergency_stop_ = msg->data;
          emergency_seen_ = true;
          emergency_stamp_ = now();
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr) {
          scan_seen_ = true;
          scan_stamp_ = now();
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, [this](sensor_msgs::msg::Imu::SharedPtr) {
          imu_seen_ = true;
          imu_stamp_ = now();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr) {
          odom_seen_ = true;
          odom_stamp_ = now();
        });

    const auto period = std::chrono::milliseconds(std::max(50, publish_period_ms_));
    timer_ = create_wall_timer(period, [this]() { PublishHealth(); });
  }

 private:
  bool IsFresh(const bool seen, const rclcpp::Time& stamp) const {
    if (!seen) {
      return false;
    }
    return ((now() - stamp).nanoseconds() / 1000000) <= stale_timeout_ms_;
  }

  diagnostic_msgs::msg::DiagnosticStatus MakeStatus(
      const std::string& name, const uint8_t level, const std::string& message,
      const std::vector<diagnostic_msgs::msg::KeyValue>& values = {}) const {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.hardware_id = "robot";
    status.level = level;
    status.message = message;
    status.values = values;
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus CheckChassis() const {
    if (!IsFresh(chassis_seen_, chassis_stamp_)) {
      return MakeStatus(
          "system/chassis", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "chassis state timeout");
    }
    const auto level = chassis_state_.connected ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                                : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    return MakeStatus(
        "system/chassis", level, chassis_state_.connected ? "connected" : "disconnected",
        {
            MakeKeyValue("backend", chassis_state_.backend),
            MakeKeyValue("connected", BoolString(chassis_state_.connected)),
            MakeKeyValue("status", chassis_state_.status),
        });
  }

  diagnostic_msgs::msg::DiagnosticStatus CheckBattery() const {
    if (!IsFresh(chassis_seen_, chassis_stamp_)) {
      return MakeStatus(
          "system/battery", diagnostic_msgs::msg::DiagnosticStatus::STALE,
          "battery unavailable");
    }
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "battery normal";
    if (chassis_state_.battery_voltage <= critical_battery_voltage_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "battery critical";
    } else if (chassis_state_.battery_voltage <= low_battery_voltage_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "battery low";
    }
    return MakeStatus(
        "system/battery", level, message,
        {MakeKeyValue("voltage", std::to_string(chassis_state_.battery_voltage))});
  }

  diagnostic_msgs::msg::DiagnosticStatus CheckEmergencyStop() const {
    if (!IsFresh(emergency_seen_, emergency_stamp_)) {
      return MakeStatus(
          "system/emergency_stop", diagnostic_msgs::msg::DiagnosticStatus::STALE,
          "emergency stop state timeout");
    }
    const auto level = emergency_stop_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                       : diagnostic_msgs::msg::DiagnosticStatus::OK;
    return MakeStatus(
        "system/emergency_stop", level, emergency_stop_ ? "emergency stop active" : "clear",
        {MakeKeyValue("active", BoolString(emergency_stop_))});
  }

  diagnostic_msgs::msg::DiagnosticStatus CheckTopicFreshness(
      const std::string& name, const bool seen, const rclcpp::Time& stamp) const {
    const bool fresh = IsFresh(seen, stamp);
    return MakeStatus(
        name, fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK
                    : diagnostic_msgs::msg::DiagnosticStatus::STALE,
        fresh ? "fresh" : "stale");
  }

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> BuildStatuses() const {
    return {
        CheckChassis(),
        CheckBattery(),
        CheckEmergencyStop(),
        CheckTopicFreshness("system/scan", scan_seen_, scan_stamp_),
        CheckTopicFreshness("system/imu", imu_seen_, imu_stamp_),
        CheckTopicFreshness("system/odom", odom_seen_, odom_stamp_),
    };
  }

  void FillDiagnosticStatus(
      const diagnostic_msgs::msg::DiagnosticStatus& status,
      diagnostic_updater::DiagnosticStatusWrapper& wrapper) const {
    wrapper.summary(status.level, status.message);
    for (const auto& value : status.values) {
      wrapper.add(value.key, value.value);
    }
  }

  void PublishHealth() {
    const auto statuses = BuildStatuses();

    uint8_t worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> issues;
    for (const auto& status : statuses) {
      worst_level = std::max(worst_level, status.level);
      if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        issues.push_back(status.name + "=" + status.message);
      }
    }

    robot_interfaces::msg::RobotState health;
    health.header.stamp = now();
    health.previous_state = previous_state_;
    health.active_goal_id = "";
    health.recoverable = worst_level != diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    if (worst_level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      health.state = "OK";
      health.message = "all monitored systems nominal";
    } else if (worst_level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      health.state = "WARN";
      health.message = JoinIssues(issues);
    } else {
      health.state = "ERROR";
      health.message = JoinIssues(issues);
    }
    previous_state_ = health.state;

    health_pub_->publish(health);
    diagnostics_.force_update();
  }

  std::string JoinIssues(const std::vector<std::string>& issues) const {
    std::ostringstream out;
    for (std::size_t i = 0; i < issues.size(); ++i) {
      if (i > 0) {
        out << "; ";
      }
      out << issues[i];
    }
    return out.str();
  }

  int stale_timeout_ms_ = 1000;
  double low_battery_voltage_ = 22.0;
  double critical_battery_voltage_ = 20.0;
  int publish_period_ms_ = 200;
  bool chassis_seen_ = false;
  bool emergency_seen_ = false;
  bool scan_seen_ = false;
  bool imu_seen_ = false;
  bool odom_seen_ = false;
  bool emergency_stop_ = false;
  std::string previous_state_ = "UNKNOWN";
  robot_interfaces::msg::ChassisState chassis_state_;
  rclcpp::Time chassis_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time emergency_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  diagnostic_updater::Updater diagnostics_;
  rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr health_pub_;
  rclcpp::Subscription<robot_interfaces::msg::ChassisState>::SharedPtr chassis_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
