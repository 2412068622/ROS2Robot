#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_fleet/srv/list_fleet_robots.hpp"
#include "robot_interfaces_mission/srv/list_stations.hpp"
#include "robot_interfaces_fleet/srv/update_fleet_robot_state.hpp"

using namespace std::chrono_literals;

namespace {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double NormalizeAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace

class Robot2FleetDemoDriverNode final : public rclcpp::Node {
 public:
  Robot2FleetDemoDriverNode() : Node("robot2_fleet_demo_driver_node") {
    robot_id_ = declare_parameter<std::string>("robot_id", "robot_2");
    initial_station_id_ = declare_parameter<std::string>("initial_station_id", "dock");
    stations_service_ = declare_parameter<std::string>("stations_service", "/v2/list_stations");
    fleet_service_ = declare_parameter<std::string>("fleet_service", "/v2/list_fleet_robots");
    update_fleet_service_ =
        declare_parameter<std::string>("update_fleet_service", "/v2/update_fleet_robot_state");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/robot_2/odom");
    main_odom_topic_ = declare_parameter<std::string>("main_odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/robot_2/cmd_vel");
    max_linear_mps_ = declare_parameter<double>("max_linear_mps", 0.72);
    max_angular_radps_ = declare_parameter<double>("max_angular_radps", 1.25);
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.22);
    slow_radius_m_ = declare_parameter<double>("slow_radius_m", 1.0);
    min_separation_m_ = declare_parameter<double>("min_separation_m", 1.25);
    control_period_s_ = declare_parameter<double>("control_period_s", 0.1);
    linear_heading_tolerance_rad_ = declare_parameter<double>("linear_heading_tolerance_rad", 0.18);
    linear_command_sign_ = declare_parameter<double>("linear_command_sign", 1.0);
    odom_yaw_sign_ = declare_parameter<double>("odom_yaw_sign", -1.0);
    angular_command_sign_ = declare_parameter<double>("angular_command_sign", -1.0);

    stations_client_ = create_client<robot_interfaces_mission::srv::ListStations>(stations_service_);
    fleet_client_ = create_client<robot_interfaces_fleet::srv::ListFleetRobots>(fleet_service_);
    update_fleet_client_ =
        create_client<robot_interfaces_fleet::srv::UpdateFleetRobotState>(update_fleet_service_);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          pose_.x = msg->pose.pose.position.x;
          pose_.y = msg->pose.pose.position.y;
          pose_.yaw = YawFromQuaternion(msg->pose.pose.orientation);
          have_pose_ = true;
        });
    main_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        main_odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          main_pose_.x = msg->pose.pose.position.x;
          main_pose_.y = msg->pose.pose.position.y;
          main_pose_.yaw = YawFromQuaternion(msg->pose.pose.orientation);
          have_main_pose_ = true;
        });

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_s_)),
        [this]() { Tick(); });
  }

 private:
  void Tick() {
    PollStations();
    PollFleet();
    UpdateWorldOffset();

    if (!have_pose_ || target_station_id_.empty() ||
        stations_.find(target_station_id_) == stations_.end() || !have_world_offset_) {
      PublishStop();
      return;
    }

    const auto target = stations_.at(target_station_id_);
    const auto world_pose = CurrentWorldPose();
    const double dx = target.x - world_pose.x;
    const double dy = target.y - world_pose.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= goal_tolerance_m_) {
      PublishStop();
      ReportArrival();
      return;
    }

    if (TooCloseToMainRobot()) {
      PublishStop();
      return;
    }

    const double target_heading = std::atan2(dy, dx);
    const double heading_error = NormalizeAngle(target_heading - world_pose.yaw);
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z =
        angular_command_sign_ *
        std::clamp(1.6 * heading_error, -max_angular_radps_, max_angular_radps_);
    if (std::abs(heading_error) < linear_heading_tolerance_rad_) {
      const double distance_scale = std::clamp(distance / slow_radius_m_, 0.25, 1.0);
      cmd.linear.x = linear_command_sign_ * max_linear_mps_ * distance_scale;
    }
    cmd_pub_->publish(cmd);
  }

  void PollStations() {
    if (stations_request_pending_ || !ShouldPoll(last_station_poll_, 2s) ||
        !stations_client_->service_is_ready()) {
      return;
    }
    stations_request_pending_ = true;
    auto request = std::make_shared<robot_interfaces_mission::srv::ListStations::Request>();
    stations_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_mission::srv::ListStations>::SharedFuture future) {
          stations_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          std::map<std::string, Pose2D> stations;
          for (std::size_t i = 0; i < response->station_ids.size(); ++i) {
            if (i >= response->x.size() || i >= response->y.size() || i >= response->yaw.size()) {
              continue;
            }
            stations[response->station_ids[i]] = {response->x[i], response->y[i], response->yaw[i]};
          }
          if (!stations.empty()) {
            stations_ = std::move(stations);
          }
        });
  }

  void PollFleet() {
    if (fleet_request_pending_ || !ShouldPoll(last_fleet_poll_, 750ms) ||
        !fleet_client_->service_is_ready()) {
      return;
    }
    fleet_request_pending_ = true;
    auto request = std::make_shared<robot_interfaces_fleet::srv::ListFleetRobots::Request>();
    request->include_disabled = true;
    fleet_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_fleet::srv::ListFleetRobots>::SharedFuture future) {
          fleet_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          for (std::size_t i = 0; i < response->robot_ids.size(); ++i) {
            if (response->robot_ids[i] != robot_id_) {
              continue;
            }
            if (i < response->current_station_ids.size()) {
              SetTargetStation(response->current_station_ids[i]);
            }
            return;
          }
        });
  }

  void SetTargetStation(const std::string& station_id) {
    if (station_id.empty() || station_id == target_station_id_) {
      return;
    }
    target_station_id_ = station_id;
    arrival_reported_ = false;
  }

  bool ShouldPoll(
      std::chrono::steady_clock::time_point& last_poll,
      const std::chrono::steady_clock::duration period) {
    const auto now = std::chrono::steady_clock::now();
    if (last_poll.time_since_epoch().count() != 0 && now - last_poll < period) {
      return false;
    }
    last_poll = now;
    return true;
  }

  bool TooCloseToMainRobot() const {
    if (!have_main_pose_ || min_separation_m_ <= 0.0) {
      return false;
    }
    const auto world_pose = CurrentWorldPose();
    return std::hypot(world_pose.x - main_pose_.x, world_pose.y - main_pose_.y) < min_separation_m_;
  }

  void UpdateWorldOffset() {
    if (have_world_offset_ || !have_pose_ || stations_.empty()) {
      return;
    }
    auto station = stations_.find(initial_station_id_);
    if (station == stations_.end() && !target_station_id_.empty()) {
      station = stations_.find(target_station_id_);
    }
    if (station == stations_.end()) {
      return;
    }
    world_offset_x_ = station->second.x - pose_.x;
    world_offset_y_ = station->second.y - pose_.y;
    have_world_offset_ = true;
  }

  Pose2D CurrentWorldPose() const {
    return {pose_.x + world_offset_x_, pose_.y + world_offset_y_, odom_yaw_sign_ * pose_.yaw};
  }

  void ReportArrival() {
    if (arrival_reported_ || arrival_report_pending_ || !update_fleet_client_->service_is_ready()) {
      return;
    }
    arrival_report_pending_ = true;
    auto request = std::make_shared<robot_interfaces_fleet::srv::UpdateFleetRobotState::Request>();
    request->robot_id = robot_id_;
    request->state = "available";
    request->current_station_id = target_station_id_;
    request->update_state = true;
    request->update_current_station_id = true;
    update_fleet_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_fleet::srv::UpdateFleetRobotState>::SharedFuture future) {
          arrival_report_pending_ = false;
          arrival_reported_ = future.get()->success;
        });
  }

  void PublishStop() {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  std::string robot_id_;
  std::string initial_station_id_;
  std::string stations_service_;
  std::string fleet_service_;
  std::string update_fleet_service_;
  std::string odom_topic_;
  std::string main_odom_topic_;
  std::string cmd_vel_topic_;
  double max_linear_mps_ = 0.72;
  double max_angular_radps_ = 1.25;
  double goal_tolerance_m_ = 0.22;
  double slow_radius_m_ = 1.0;
  double min_separation_m_ = 1.25;
  double control_period_s_ = 0.1;
  double linear_heading_tolerance_rad_ = 0.18;
  double linear_command_sign_ = 1.0;
  double odom_yaw_sign_ = -1.0;
  double angular_command_sign_ = -1.0;
  bool have_pose_ = false;
  bool have_main_pose_ = false;
  bool have_world_offset_ = false;
  bool stations_request_pending_ = false;
  bool fleet_request_pending_ = false;
  bool arrival_report_pending_ = false;
  bool arrival_reported_ = false;
  std::string target_station_id_;
  Pose2D pose_;
  Pose2D main_pose_;
  double world_offset_x_ = 0.0;
  double world_offset_y_ = 0.0;
  std::map<std::string, Pose2D> stations_;
  std::chrono::steady_clock::time_point last_station_poll_;
  std::chrono::steady_clock::time_point last_fleet_poll_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr main_odom_sub_;
  rclcpp::Client<robot_interfaces_mission::srv::ListStations>::SharedPtr stations_client_;
  rclcpp::Client<robot_interfaces_fleet::srv::ListFleetRobots>::SharedPtr fleet_client_;
  rclcpp::Client<robot_interfaces_fleet::srv::UpdateFleetRobotState>::SharedPtr update_fleet_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot2FleetDemoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
