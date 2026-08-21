#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_interfaces/msg/payload_state.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_mission/srv/detect_traffic_deadlock.hpp"
#include "robot_interfaces_facility/srv/get_door_state.hpp"
#include "robot_interfaces_mission/srv/get_dock_state.hpp"
#include "robot_interfaces_facility/srv/get_lift_state.hpp"
#include "robot_interfaces_facility/srv/get_pending_confirmations.hpp"
#include "robot_interfaces_fleet/srv/list_fleet_robots.hpp"
#include "robot_interfaces_mission/srv/list_mission_events.hpp"
#include "robot_interfaces_mission/srv/list_traffic_reservations.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct DisplayPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct LabelPose {
  std::string model_name;
  std::string text;
  double x = 0.0;
  double y = 0.0;
  double z = 1.0;
  double yaw = 0.0;
};

geometry_msgs::msg::Pose Pose(const double x, const double y, const double z, const double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

geometry_msgs::msg::Pose HiddenPose() { return Pose(0.0, 0.0, -2.0, 0.0); }

double Lerp(const double start, const double finish, const double ratio) {
  return start + (finish - start) * std::clamp(ratio, 0.0, 1.0);
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool ActiveStatus(const std::string& status) {
  return status.find("open") != std::string::npos ||
         status.find("occupied") != std::string::npos ||
         status.find("reserved") != std::string::npos ||
         status.find("session") != std::string::npos;
}

constexpr double kShowcaseReviewHoldSeconds = 600.0;

}  // namespace

class AmrSimGazeboEntityBridgeNode final : public rclcpp::Node {
 public:
  AmrSimGazeboEntityBridgeNode() : Node("amr_sim_gazebo_entity_bridge_node") {
    set_pose_service_ =
        declare_parameter<std::string>("set_pose_service", "/world/indoor_room/set_pose");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/model/mobile_robot/odometry");
    robot2_odom_topic_ = declare_parameter<std::string>("robot2_odom_topic", "/robot_2/odom");
    robot2_pose_bridge_enabled_ = declare_parameter<bool>("robot2_pose_bridge_enabled", false);
    update_period_s_ = declare_parameter<double>("update_period_s", 0.5);
    robot2_speed_mps_ = declare_parameter<double>("robot2_speed_mps", 0.45);
    robot2_min_separation_m_ = declare_parameter<double>("robot2_min_separation_m", 1.35);
    labels_enabled_ = declare_parameter<bool>("labels_enabled", true);
    // 默认朝向对准 SDF 中默认相机机位（场景 -X-Y 斜上方），
    // 这样开启 labels_enabled 做纯 Gazebo 演示时文字正面朝向观察者，不会侧/反。
    label_yaw_rad_ = declare_parameter<double>("label_yaw_rad", -2.356);
    declare_parameter<double>("label_scale", 0.55);

    InitStationLayout();
    set_pose_client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(set_pose_service_);
    door_client_ = create_client<robot_interfaces_facility::srv::GetDoorState>("/v2/get_door_state");
    dock_client_ = create_client<robot_interfaces_mission::srv::GetDockState>("/v2/get_dock_state");
    lift_client_ = create_client<robot_interfaces_facility::srv::GetLiftState>("/v2/get_lift_state");
    pending_client_ =
        create_client<robot_interfaces_facility::srv::GetPendingConfirmations>("/v2/get_pending_confirmations");
    fleet_client_ = create_client<robot_interfaces_fleet::srv::ListFleetRobots>("/v2/list_fleet_robots");
    traffic_client_ = create_client<robot_interfaces_mission::srv::ListTrafficReservations>(
        "/v2/list_traffic_reservations");
    events_client_ =
        create_client<robot_interfaces_mission::srv::ListMissionEvents>("/v2/list_mission_events");
    deadlock_client_ =
        create_client<robot_interfaces_mission::srv::DetectTrafficDeadlock>("/v2/detect_traffic_deadlock");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          robot_pose_.x = msg->pose.pose.position.x;
          robot_pose_.y = msg->pose.pose.position.y;
          robot_pose_.yaw = YawFromQuaternion(msg->pose.pose.orientation);
          have_robot_pose_ = true;
        });
    robot2_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        robot2_odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          if (robot2_pose_bridge_enabled_) {
            return;
          }
          robot2_pose_.x = msg->pose.pose.position.x;
          robot2_pose_.y = msg->pose.pose.position.y;
          robot2_pose_.yaw = YawFromQuaternion(msg->pose.pose.orientation);
        });
    payload_sub_ = create_subscription<robot_interfaces::msg::PayloadState>(
        "/payload/state", 10, [this](robot_interfaces::msg::PayloadState::SharedPtr msg) {
          const bool was_loaded = payload_loaded_;
          const std::string previous_action = payload_last_action_;
          payload_loaded_ = msg->loaded;
          payload_last_action_ = msg->last_action;
          payload_top_module_ = msg->top_module_type;
          if (payload_loaded_ && !was_loaded) {
            payload_load_started_s_ = now().seconds();
            payload_delivered_visible_ = false;
          } else if (was_loaded && msg->last_action == "confirm_unload") {
            payload_unload_started_s_ = now().seconds();
            payload_delivered_visible_ = true;
            payload_delivered_pose_ =
                have_robot_pose_ ? robot_pose_ : stations_.at("storage_b");
          }
          if (!payload_last_action_.empty() && payload_last_action_ != "none" &&
              payload_last_action_ != previous_action) {
            top_module_action_started_s_ = now().seconds();
          }
        });
    localization_sub_ = create_subscription<robot_interfaces::msg::LocalizationHealth>(
        "/localization_health", 10,
        [this](robot_interfaces::msg::LocalizationHealth::SharedPtr msg) {
          localization_state_ = msg->state;
          localization_localized_ = msg->localized;
        });
    safety_sub_ = create_subscription<robot_interfaces::msg::SafetyState>(
        "/safety_state", 10, [this](robot_interfaces::msg::SafetyState::SharedPtr msg) {
          obstacle_blocked_ = msg->obstacle_blocked || msg->safety_stop;
          if (obstacle_blocked_) {
            obstacle_block_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
          }
          const bool was_speed_limited = speed_limited_;
          const bool current_speed_limited =
              msg->runtime_speed_limit_mps > 0.0 ||
              (msg->effective_speed_limit_mps > 0.0 &&
               msg->effective_speed_limit_mps < msg->zone_speed_limit_mps);
          speed_limited_ = current_speed_limited;
          if (speed_limited_) {
            speed_limit_seen_ = true;
            speed_limit_visible_until_s_ = now().seconds() + 240.0;
          }
          if ((was_speed_limited || speed_limit_seen_) && !speed_limited_) {
            speed_restore_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
          }
        });
    manual_takeover_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/manual_takeover/state", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) { manual_takeover_active_ = msg->data; });

    robot2_pose_ = {-3.9, -3.8, 0.0};
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(update_period_s_)),
        [this]() { Tick(); });
  }

 private:
  void InitStationLayout() {
    stations_ = {
        {"dock", {-3.9, -3.1, 0.0}},
        {"receiving", {-3.7, 0.65, 0.0}},
        {"storage_a", {1.5, 2.0, 1.57}},
        {"storage_b", {-2.0, 2.55, 1.57}},
        {"packing", {2.0, -1.35, -1.57}},
        {"traffic_wait", {-1.0, -4.2, 0.0}},
        {"lift_lobby", {3.65, 2.2, 1.57}},
        {"floor_2_dropoff", {3.65, 3.35, 1.57}},
    };
  }

  void Tick() {
    if (!set_pose_client_->service_is_ready()) {
      return;
    }
    PollDoor();
    PollDock();
    PollLift();
    PollPendingConfirmation();
    PollFleet();
    PollTraffic();
    PollMissionEvents();
    PollDeadlock();
    UpdatePayloadPose();
    UpdateTopModulePose();
    UpdateDoorPose();
    UpdateDockPose();
    UpdateLiftPose();
    UpdatePendingConfirmationPose();
    UpdateLocalizationPose();
    UpdateTrafficPose();
    UpdateSafetyPose();
    UpdateManualTakeoverPose();
    UpdateRobot2Pose();
    UpdateObjectLabels();
  }

  void PollDoor() {
    if (door_request_pending_ || !door_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::GetDoorState::Request>();
    request->door_id = "receiving_door";
    door_request_pending_ = true;
    door_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_facility::srv::GetDoorState>::SharedFuture future) {
          door_request_pending_ = false;
          const auto response = future.get();
          if (response->success) {
            door_status_ = response->status;
            door_available_ = response->available;
          }
        });
  }

  void PollDock() {
    if (dock_request_pending_ || !dock_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::GetDockState::Request>();
    request->dock_id = "main_dock";
    dock_request_pending_ = true;
    dock_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_mission::srv::GetDockState>::SharedFuture future) {
          dock_request_pending_ = false;
          const auto response = future.get();
          if (response->success) {
            dock_state_ = response->state;
          }
        });
  }

  void PollLift() {
    if (lift_request_pending_ || !lift_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::GetLiftState::Request>();
    request->lift_id = "freight_elevator";
    lift_request_pending_ = true;
    lift_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_facility::srv::GetLiftState>::SharedFuture future) {
          lift_request_pending_ = false;
          const auto response = future.get();
          if (response->success) {
            lift_status_ = response->status;
            lift_available_ = response->available;
          }
        });
  }

  void PollFleet() {
    if (fleet_request_pending_ || !fleet_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_fleet::srv::ListFleetRobots::Request>();
    request->include_disabled = true;
    fleet_request_pending_ = true;
    fleet_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_fleet::srv::ListFleetRobots>::SharedFuture future) {
          fleet_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          for (std::size_t i = 0; i < response->robot_ids.size(); ++i) {
            if (response->robot_ids[i] != "robot_2") {
              continue;
            }
            if (i < response->current_station_ids.size() &&
                stations_.find(response->current_station_ids[i]) != stations_.end()) {
              robot2_target_station_ = response->current_station_ids[i];
            }
            break;
          }
        });
  }

  void PollPendingConfirmation() {
    if (pending_request_pending_ || !pending_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::GetPendingConfirmations::Request>();
    pending_request_pending_ = true;
    pending_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_facility::srv::GetPendingConfirmations>::SharedFuture future) {
          pending_request_pending_ = false;
          const auto response = future.get();
          pending_confirmation_active_ = false;
          if (!response->success) {
            return;
          }
          for (std::size_t i = 0; i < response->station_ids.size(); ++i) {
            if (i < response->states.size() && response->states[i] != "PENDING") {
              continue;
            }
            if (stations_.find(response->station_ids[i]) != stations_.end()) {
              pending_confirmation_active_ = true;
              pending_confirmation_station_ = response->station_ids[i];
              return;
            }
          }
        });
  }

  void PollTraffic() {
    if (traffic_request_pending_ || !traffic_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::ListTrafficReservations::Request>();
    traffic_request_pending_ = true;
    traffic_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_mission::srv::ListTrafficReservations>::SharedFuture future) {
          traffic_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          const bool had_block = traffic_block_active_;
          const bool had_visible_block = now().seconds() < traffic_block_visible_until_s_;
          const std::string previous_from = traffic_block_from_;
          const std::string previous_to = traffic_block_to_;
          traffic_block_active_ = false;
          std::string selected_from;
          std::string selected_to;
          std::string selected_owner;
          std::string selected_type;
          bool selected_is_mission = false;
          bool selected_is_showcase_traffic = false;
          for (std::size_t i = 0; i < response->resource_ids.size(); ++i) {
            const auto route = ParseRouteEdge(response->resource_ids[i]);
            if (!route.first.empty() && stations_.find(route.first) != stations_.end() &&
                stations_.find(route.second) != stations_.end()) {
              const std::string owner =
                  i < response->owner_ids.size() ? response->owner_ids[i] : "";
              const std::string type =
                  i < response->reservation_types.size() ? response->reservation_types[i] : "";
              const bool is_mission = type == "mission" || owner.rfind("station_order_", 0) == 0;
              const bool is_showcase_traffic =
                  owner.find("traffic_alpha") != std::string::npos ||
                  owner.find("traffic_beta") != std::string::npos;
              if (selected_from.empty() ||
                  (is_mission && !selected_is_mission) ||
                  (is_showcase_traffic && !selected_is_showcase_traffic)) {
                selected_from = route.first;
                selected_to = route.second;
                selected_owner = owner;
                selected_type = type;
                selected_is_mission = is_mission;
                selected_is_showcase_traffic = is_showcase_traffic;
              }
            }
          }
          if (!selected_from.empty()) {
            traffic_block_active_ = true;
            traffic_block_from_ = selected_from;
            traffic_block_to_ = selected_to;
            traffic_owner_id_ = selected_owner;
            traffic_reservation_type_ = selected_type;
            traffic_block_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
            if (selected_owner.find("traffic_alpha") != std::string::npos) {
              traffic_wait_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
            }
            if (selected_owner.find("traffic_beta_after_release") != std::string::npos) {
              traffic_go_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
            }
            return;
          }
          if ((had_block || had_visible_block) && stations_.find(previous_from) != stations_.end() &&
              stations_.find(previous_to) != stations_.end()) {
            traffic_reopen_from_ = previous_from;
            traffic_reopen_to_ = previous_to;
            traffic_reopen_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
          }
        });
  }

  void PollMissionEvents() {
    if (events_request_pending_ || !events_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::ListMissionEvents::Request>();
    request->limit = 30;
    events_request_pending_ = true;
    events_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedFuture future) {
          events_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          for (std::size_t i = 0; i < response->mission_ids.size(); ++i) {
            if (i < response->states.size() && response->states[i] == "DOCKING_CONTACT_FAILED") {
              docking_retry_visible_until_s_ = now().seconds() + 600.0;
            }
            if (response->mission_ids[i] == "safety_sim_auto_obstacle" &&
                i < response->states.size() && response->states[i] == "OBSTACLE_CLEARED") {
              obstacle_clear_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
              // Keep the red obstacle visible as review evidence after clear; safety state is unchanged.
              obstacle_block_visible_until_s_ = std::max(
                  obstacle_block_visible_until_s_, now().seconds() + kShowcaseReviewHoldSeconds);
            }
            if (response->mission_ids[i] == "charging_request_sim_auto_opportunity") {
              opportunity_charge_visible_until_s_ = now().seconds() + 240.0;
            }
            if (response->mission_ids[i].find("charging_request_") == 0) {
              charging_visible_until_s_ = now().seconds() + 240.0;
            }
            if (i < response->states.size() && response->states[i] == "DYNAMIC_SPEED_LIMIT") {
              speed_limit_seen_ = true;
              speed_limit_visible_until_s_ = now().seconds() + 240.0;
              if (!speed_limited_) {
                speed_restore_visible_until_s_ = now().seconds() + kShowcaseReviewHoldSeconds;
              }
            }
          }
        });
  }


  void PollDeadlock() {
    if (deadlock_request_pending_ || !deadlock_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::DetectTrafficDeadlock::Request>();
    deadlock_request_pending_ = true;
    deadlock_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_mission::srv::DetectTrafficDeadlock>::SharedFuture future) {
          deadlock_request_pending_ = false;
          const auto response = future.get();
          deadlock_active_ = response->success && response->deadlocked;
        });
  }

  std::pair<std::string, std::string> ParseRouteEdge(const std::string& resource_id) const {
    constexpr char kPrefix[] = "route_edge:";
    const std::string prefix(kPrefix);
    if (resource_id.rfind(prefix, 0) != 0) {
      return {};
    }
    const auto route = resource_id.substr(prefix.size());
    const auto separator = route.find("__");
    if (separator == std::string::npos || separator == 0 || separator + 2 >= route.size()) {
      return {};
    }
    return {route.substr(0, separator), route.substr(separator + 2)};
  }

  void UpdatePayloadPose() {
    const Pose2D waiting_pose{-3.35, -0.45, 0.0};
    if (payload_loaded_ && have_robot_pose_) {
      const double elapsed = payload_load_started_s_ > 0.0 ? now().seconds() - payload_load_started_s_ : 999.0;
      if (elapsed < 2.0) {
        payload_display_pose_ = {waiting_pose.x, waiting_pose.y, 0.18, waiting_pose.yaw};
        SendPose(
            "payload_box_waiting",
            Pose(payload_display_pose_.x, payload_display_pose_.y, payload_display_pose_.z, payload_display_pose_.yaw));
        SendPose("payload_lock_indicator", HiddenPose());
        return;
      }
      if (elapsed < 5.0) {
        const double ratio = (elapsed - 2.0) / 3.0;
        payload_display_pose_ = {
            Lerp(waiting_pose.x, robot_pose_.x, ratio),
            Lerp(waiting_pose.y, robot_pose_.y, ratio),
            Lerp(0.18, 0.78, ratio) + 0.08 * std::sin(ratio * 3.14159265358979323846),
            robot_pose_.yaw};
        SendPose(
            "payload_box_waiting",
            Pose(payload_display_pose_.x, payload_display_pose_.y, payload_display_pose_.z, payload_display_pose_.yaw));
        SendPose("payload_lock_indicator", HiddenPose());
        return;
      }
      payload_display_pose_ = {robot_pose_.x, robot_pose_.y, 0.78, robot_pose_.yaw};
      SendPose(
          "payload_box_waiting",
          Pose(payload_display_pose_.x, payload_display_pose_.y, payload_display_pose_.z, payload_display_pose_.yaw));
      SendPose("payload_lock_indicator", Pose(robot_pose_.x, robot_pose_.y, 1.08, robot_pose_.yaw));
      return;
    }
    if (payload_delivered_visible_) {
      const double elapsed =
          payload_unload_started_s_ > 0.0 ? now().seconds() - payload_unload_started_s_ : 999.0;
      if (elapsed < 2.5 && have_robot_pose_) {
        const double ratio = elapsed / 2.5;
        payload_display_pose_ = {
            Lerp(robot_pose_.x, payload_delivered_pose_.x, ratio),
            Lerp(robot_pose_.y, payload_delivered_pose_.y, ratio),
            Lerp(0.78, 0.18, ratio),
            payload_delivered_pose_.yaw};
      } else {
        payload_display_pose_ = {
            payload_delivered_pose_.x, payload_delivered_pose_.y, 0.18, payload_delivered_pose_.yaw};
      }
      SendPose(
          "payload_box_waiting",
          Pose(payload_display_pose_.x, payload_display_pose_.y, payload_display_pose_.z, payload_display_pose_.yaw));
      SendPose(
          "payload_lock_indicator",
          Pose(payload_display_pose_.x, payload_display_pose_.y, payload_display_pose_.z + 0.74, payload_display_pose_.yaw));
      return;
    }
    payload_display_pose_ = {waiting_pose.x, waiting_pose.y, 0.18, waiting_pose.yaw};
    SendPose("payload_box_waiting", Pose(waiting_pose.x, waiting_pose.y, 0.18, waiting_pose.yaw));
    SendPose("payload_lock_indicator", HiddenPose());
  }

  void UpdateTopModulePose() {
    const bool active = !payload_top_module_.empty() && payload_last_action_ != "none" &&
                        !payload_last_action_.empty();
    if (active && have_robot_pose_) {
      const double elapsed =
          top_module_action_started_s_ > 0.0 ? now().seconds() - top_module_action_started_s_ : 0.0;
      const double scan_offset = 0.10 * std::sin(elapsed * 5.0);
      const double x = robot_pose_.x - std::sin(robot_pose_.yaw) * (0.34 + scan_offset);
      const double y = robot_pose_.y + std::cos(robot_pose_.yaw) * (0.34 + scan_offset);
      const double z = 0.98 + 0.05 * std::sin(elapsed * 8.0);
      SendPose("top_module_status_indicator", Pose(x, y, z, robot_pose_.yaw));
      SendLabelPose("label_top_module_status", x, y, z + 0.38, robot_pose_.yaw);
      return;
    }
    SendPose("top_module_status_indicator", HiddenPose());
    HideLabel("label_top_module_status");
  }

  void UpdateDoorPose() {
    const bool open = !door_available_ || ActiveStatus(door_status_);
    if (open) {
      SendPose("receiving_door_panel", Pose(-4.15, 0.52, 0.55, 1.57079632679));
      return;
    }
    SendPose("receiving_door_panel", Pose(-4.15, 0.0, 0.55, 0.0));
  }

  void UpdateDockPose() {
    const bool charging = dock_state_.find("CHARG") != std::string::npos ||
                          dock_state_.find("charging") != std::string::npos;
    const bool retrying = dock_state_.find("FAILED") != std::string::npos ||
                          dock_state_.find("failed") != std::string::npos ||
                          dock_state_.find("CONTACT") != std::string::npos;
    if (retrying) {
      docking_retry_visible_until_s_ = now().seconds() + 600.0;
    }
    if (charging) {
      charging_visible_until_s_ = now().seconds() + 240.0;
    }
    const bool show_charging = charging || now().seconds() < charging_visible_until_s_;
    SendPose("charging_status_beacon", Pose(-4.30, -3.80, show_charging ? 1.12 : 0.72, 0.0));
    SendLabelPose("label_charging_status", -4.30, -3.80, show_charging ? 1.50 : 1.10, 0.0);
    SendPose(
        "docking_retry_warning_beacon",
        Pose(-4.30, -3.42, now().seconds() < docking_retry_visible_until_s_ ? 1.05 : -2.0, 0.0));
    if (now().seconds() < docking_retry_visible_until_s_) {
      SendLabelPose("label_docking_retry_status", -4.30, -3.42, 1.42, 0.0);
    } else {
      HideLabel("label_docking_retry_status");
    }
    SendPose(
        "opportunity_charge_beacon",
        Pose(-3.55, -3.42, now().seconds() < opportunity_charge_visible_until_s_ ? 1.02 : -2.0, 0.0));
    if (now().seconds() < opportunity_charge_visible_until_s_) {
      SendLabelPose("label_opportunity_charge_status", -3.55, -3.42, 1.38, 0.0);
    } else {
      HideLabel("label_opportunity_charge_status");
    }
  }

  void UpdateLiftPose() {
    const bool upper_level = !lift_available_ || ActiveStatus(lift_status_) ||
                             lift_status_.find("floor_2") != std::string::npos;
    const double z = upper_level ? 0.86 : 0.04;
    SendPose("freight_elevator_cabin", Pose(3.65, 2.85, z, 0.0));
  }

  void UpdatePendingConfirmationPose() {
    if (!pending_confirmation_active_) {
      SendPose("operator_confirmation_panel", HiddenPose());
      HideLabel("label_operator_confirmation_status");
      return;
    }
    const auto station = stations_.find(pending_confirmation_station_);
    if (station == stations_.end()) {
      SendPose("operator_confirmation_panel", HiddenPose());
      HideLabel("label_operator_confirmation_status");
      return;
    }
    SendPose(
        "operator_confirmation_panel",
        Pose(station->second.x + 0.38, station->second.y + 0.38, 0.46, station->second.yaw));
    SendLabelPose(
        "label_operator_confirmation_status", station->second.x + 0.38,
        station->second.y + 0.38, 0.96, station->second.yaw);
  }

  void UpdateLocalizationPose() {
    const bool known = !localization_state_.empty() && localization_state_ != "UNKNOWN";
    const double z = known ? (localization_localized_ ? 1.24 : 0.92) : -2.0;
    SendPose("localization_status_beacon", Pose(-3.35, -3.55, z, 0.0));
    if (known) {
      SendLabelPose("label_localization_status", -3.35, -3.55, z + 0.34, 0.0);
    } else {
      HideLabel("label_localization_status");
    }
  }

  void UpdateTrafficPose() {
    const bool show_block =
        traffic_block_active_ || (now().seconds() < traffic_block_visible_until_s_ &&
                                  stations_.find(traffic_block_from_) != stations_.end() &&
                                  stations_.find(traffic_block_to_) != stations_.end());
    if (!show_block) {
      SendPose("traffic_route_block_barrier", HiddenPose());
      HideLabel("label_traffic_block_status");
      SendPose("traffic_deadlock_warning_beacon", Pose(0.0, 0.0, deadlock_active_ ? 1.1 : -2.0, 0.0));
      if (deadlock_active_) {
        SendLabelPose("label_traffic_deadlock_status", 0.0, 0.0, 1.46, 0.0);
      } else {
        HideLabel("label_traffic_deadlock_status");
      }
      UpdateTrafficReopenPose();
      UpdateTrafficWaitPassPose();
      return;
    }
    const auto from = stations_.at(traffic_block_from_);
    const auto to = stations_.at(traffic_block_to_);
    const double x = (from.x + to.x) * 0.5;
    const double y = (from.y + to.y) * 0.5;
    const double yaw = std::atan2(to.y - from.y, to.x - from.x) + 1.57079632679;
    SendPose("traffic_route_block_barrier", Pose(x, y, 0.28, yaw));
    SendLabelPose("label_traffic_block_status", x, y, 0.94, yaw);
    SendPose("traffic_deadlock_warning_beacon", Pose(x, y, 1.10, yaw));
    SendLabelPose("label_traffic_deadlock_status", x, y, 1.46, yaw);
    if (traffic_block_active_) {
      SendPose("traffic_route_reopen_beacon", HiddenPose());
      HideLabel("label_traffic_reopen_status");
    } else {
      UpdateTrafficReopenPose();
    }
    UpdateTrafficWaitPassPose();
  }

  void UpdateTrafficReopenPose() {
    if (now().seconds() >= traffic_reopen_visible_until_s_ ||
        stations_.find(traffic_reopen_from_) == stations_.end() ||
        stations_.find(traffic_reopen_to_) == stations_.end()) {
      SendPose("traffic_route_reopen_beacon", HiddenPose());
      HideLabel("label_traffic_reopen_status");
      return;
    }
    const auto from = stations_.at(traffic_reopen_from_);
    const auto to = stations_.at(traffic_reopen_to_);
    const double x = (from.x + to.x) * 0.5;
    const double y = (from.y + to.y) * 0.5;
    const double yaw = std::atan2(to.y - from.y, to.x - from.x) + 1.57079632679;
    SendPose("traffic_route_reopen_beacon", Pose(x, y, 0.92, yaw));
    SendLabelPose("label_traffic_reopen_status", x, y, 1.28, yaw);
  }

  void UpdateTrafficWaitPassPose() {
    if (now().seconds() < traffic_wait_visible_until_s_ &&
        stations_.find("traffic_wait") != stations_.end()) {
      const auto station = stations_.at("traffic_wait");
      SendPose("traffic_yield_wait_marker", Pose(station.x, station.y, 0.22, 0.0));
    } else {
      SendPose("traffic_yield_wait_marker", HiddenPose());
    }

    if (now().seconds() < traffic_go_visible_until_s_ &&
        stations_.find("storage_a") != stations_.end() &&
        stations_.find("packing") != stations_.end()) {
      const auto from = stations_.at("storage_a");
      const auto to = stations_.at("packing");
      const double x = (from.x + to.x) * 0.5;
      const double y = (from.y + to.y) * 0.5;
      const double yaw = std::atan2(to.y - from.y, to.x - from.x);
      SendPose("traffic_release_go_marker", Pose(x, y, 0.36, yaw));
    } else {
      SendPose("traffic_release_go_marker", HiddenPose());
    }
  }

  void UpdateSafetyPose() {
    const bool show_speed_limit = speed_limited_ || now().seconds() < speed_limit_visible_until_s_;
    SendPose("speed_limit_zone_indicator", Pose(1.2, -0.25, show_speed_limit ? 0.05 : -2.0, 0.0));
    if (show_speed_limit) {
      SendLabelPose("label_speed_limit_status", 1.2, -0.25, 0.72, 0.0);
    } else {
      HideLabel("label_speed_limit_status");
    }
    if (now().seconds() < speed_restore_visible_until_s_) {
      SendPose("speed_limit_restored_beacon", Pose(1.2, -0.25, 0.82, 0.0));
      SendLabelPose("label_speed_restored_status", 1.2, -0.25, 1.18, 0.0);
    } else {
      SendPose("speed_limit_restored_beacon", HiddenPose());
      HideLabel("label_speed_restored_status");
    }
    const bool show_obstacle = obstacle_blocked_ || now().seconds() < obstacle_block_visible_until_s_;
    if (show_obstacle && have_robot_pose_) {
      const double x = robot_pose_.x + std::cos(robot_pose_.yaw) * 0.65;
      const double y = robot_pose_.y + std::sin(robot_pose_.yaw) * 0.65;
      SendPose("safety_obstacle_block", Pose(x, y, 0.35, robot_pose_.yaw));
      SendLabelPose("label_obstacle_status", x, y, 1.00, robot_pose_.yaw);
      if (obstacle_blocked_) {
        SendPose("obstacle_clearance_indicator", HiddenPose());
        HideLabel("label_obstacle_clear_status");
        return;
      }
    } else {
      SendPose("safety_obstacle_block", HiddenPose());
      HideLabel("label_obstacle_status");
    }
    if (now().seconds() < obstacle_clear_visible_until_s_) {
      if (have_robot_pose_) {
        const double x = robot_pose_.x + std::cos(robot_pose_.yaw) * 0.65;
        const double y = robot_pose_.y + std::sin(robot_pose_.yaw) * 0.65;
        SendPose("obstacle_clearance_indicator", Pose(x, y, 0.85, robot_pose_.yaw));
        SendLabelPose("label_obstacle_clear_status", x, y, 1.20, robot_pose_.yaw);
        return;
      }
      SendPose("obstacle_clearance_indicator", Pose(0.65, 3.65, 0.85, 0.0));
      SendLabelPose("label_obstacle_clear_status", 0.65, 3.65, 1.20, 0.0);
      return;
    }
    SendPose("obstacle_clearance_indicator", Pose(0.65, 3.65, -2.0, 0.0));
    HideLabel("label_obstacle_clear_status");
  }

  void UpdateManualTakeoverPose() {
    if (manual_takeover_active_) {
      SendPose("manual_takeover_indicator", Pose(-4.35, 3.65, 0.82, 0.0));
      SendLabelPose("label_manual_takeover_status", -4.35, 3.65, 1.18, 0.0);
      return;
    }
    SendPose("manual_takeover_indicator", Pose(-4.35, 3.65, -2.0, 0.0));
    HideLabel("label_manual_takeover_status");
  }

  void UpdateRobot2Pose() {
    if (!robot2_pose_bridge_enabled_) {
      return;
    }
    const auto target = stations_.find(robot2_target_station_);
    if (target == stations_.end()) {
      SendPose("mobile_robot_2", Pose(robot2_pose_.x, robot2_pose_.y, 0.24, robot2_pose_.yaw));
      return;
    }
    const double dx = target->second.x - robot2_pose_.x;
    const double dy = target->second.y - robot2_pose_.y;
    const double distance = std::hypot(dx, dy);
    Pose2D next_pose = robot2_pose_;
    if (distance > 0.03) {
      const double step = std::min(distance, robot2_speed_mps_ * update_period_s_);
      next_pose.x += dx / distance * step;
      next_pose.y += dy / distance * step;
      next_pose.yaw = std::atan2(dy, dx);
    } else {
      next_pose = target->second;
    }
    robot2_pose_ = ApplyRobot2Separation(next_pose, target->second);
    SendPose("mobile_robot_2", Pose(robot2_pose_.x, robot2_pose_.y, 0.24, robot2_pose_.yaw));
  }

  Pose2D ApplyRobot2Separation(Pose2D proposed_pose, const Pose2D& target_pose) const {
    if (!have_robot_pose_ || robot2_min_separation_m_ <= 0.0) {
      return proposed_pose;
    }
    double dx = proposed_pose.x - robot_pose_.x;
    double dy = proposed_pose.y - robot_pose_.y;
    double distance = std::hypot(dx, dy);
    if (distance >= robot2_min_separation_m_) {
      return proposed_pose;
    }
    if (distance < 1e-3) {
      dx = robot2_pose_.x - robot_pose_.x;
      dy = robot2_pose_.y - robot_pose_.y;
      distance = std::hypot(dx, dy);
    }
    if (distance < 1e-3) {
      dx = -(target_pose.y - robot_pose_.y);
      dy = target_pose.x - robot_pose_.x;
      distance = std::hypot(dx, dy);
    }
    if (distance < 1e-3) {
      dx = 0.0;
      dy = -1.0;
      distance = 1.0;
    }

    proposed_pose.x = robot_pose_.x + dx / distance * robot2_min_separation_m_;
    proposed_pose.y = robot_pose_.y + dy / distance * robot2_min_separation_m_;
    proposed_pose.x = std::clamp(proposed_pose.x, -4.35, 4.35);
    proposed_pose.y = std::clamp(proposed_pose.y, -4.35, 4.35);
    proposed_pose.yaw = std::atan2(target_pose.y - proposed_pose.y, target_pose.x - proposed_pose.x);
    return proposed_pose;
  }

  void SendPose(const std::string& entity_name, const geometry_msgs::msg::Pose& pose) {
    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = entity_name;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose = pose;
    set_pose_client_->async_send_request(
        request,
        [this, entity_name](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future) {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "failed to set Gazebo pose for %s",
                entity_name.c_str());
          }
        });
  }

  void SendLabelPose(
      const std::string& entity_name, const double x, const double y, const double z,
      const double yaw) {
    if (!labels_enabled_) {
      HideLabel(entity_name);
      return;
    }
    (void)yaw;
    SendPose(entity_name, Pose(x, y, z, label_yaw_rad_));
  }

  void HideLabel(const std::string& entity_name) { SendPose(entity_name, HiddenPose()); }

  LabelPose PayloadLabelPose() const {
    return {
        "label_payload", "PAYLOAD", payload_display_pose_.x, payload_display_pose_.y,
        payload_display_pose_.z + 1.02, 0.0};
  }

  std::vector<LabelPose> BuildObjectLabels() const {
    const Pose2D robot_pose = have_robot_pose_ ? robot_pose_ : Pose2D{};
    return {
        {"label_robot_1", "ROBOT_1", robot_pose.x, robot_pose.y, 1.70, 0.0},
        {"label_robot_2", "ROBOT_2", robot2_pose_.x, robot2_pose_.y, 1.60, 0.0},
        {"label_dock", "DOCK", -3.90, -3.80, 1.65, 0.0},
        {"label_receiving", "RECEIVING", -3.70, 0.65, 1.55, 0.0},
        {"label_door", "DOOR", -4.15, 0.0, 1.70, 0.0},
        {"label_elevator", "ELEVATOR", 3.65, 2.85, 1.80, 0.0},
        {"label_floor_2", "FLOOR_2", 3.65, 3.85, 1.65, 0.0},
        {"label_storage_a", "STORAGE_A", 1.50, 1.50, 1.60, 0.0},
        {"label_storage_b", "STORAGE_B", -2.00, 2.00, 1.60, 0.0},
        {"label_packing", "PACKING", 2.00, -2.00, 1.60, 0.0},
        {"label_traffic", "TRAFFIC", 0.0, 0.0, 1.35, 0.0},
        {"label_slow_zone", "SLOW_ZONE", 1.20, -0.25, 1.25, 0.0},
        {"label_confirm", "CONFIRM", -3.32, 1.03, 1.35, 0.0},
        PayloadLabelPose(),
    };
  }

  void UpdateObjectLabels() {
    const auto labels = BuildObjectLabels();
    if (!labels_enabled_) {
      for (const auto& label : labels) {
        HideLabel(label.model_name);
      }
      return;
    }
    for (const auto& label : labels) {
      SendLabelPose(label.model_name, label.x, label.y, label.z, label.yaw);
    }
  }

  std::string set_pose_service_;
  std::string odom_topic_;
  std::string robot2_odom_topic_;
  double update_period_s_ = 0.5;
  double robot2_speed_mps_ = 0.45;
  double robot2_min_separation_m_ = 1.35;
  double label_yaw_rad_ = -2.356;
  bool robot2_pose_bridge_enabled_ = false;
  bool have_robot_pose_ = false;
  bool labels_enabled_ = true;
  bool payload_loaded_ = false;
  bool localization_localized_ = false;
  bool obstacle_blocked_ = false;
  bool speed_limited_ = false;
  bool speed_limit_seen_ = false;
  bool manual_takeover_active_ = false;
  bool payload_delivered_visible_ = false;
  bool door_available_ = true;
  bool lift_available_ = true;
  bool door_request_pending_ = false;
  bool dock_request_pending_ = false;
  bool lift_request_pending_ = false;
  bool pending_request_pending_ = false;
  bool fleet_request_pending_ = false;
  bool traffic_request_pending_ = false;
  bool events_request_pending_ = false;
  bool deadlock_request_pending_ = false;
  bool traffic_block_active_ = false;
  bool deadlock_active_ = false;
  bool pending_confirmation_active_ = false;
  double docking_retry_visible_until_s_ = 0.0;
  double charging_visible_until_s_ = 0.0;
  double opportunity_charge_visible_until_s_ = 0.0;
  double obstacle_block_visible_until_s_ = 0.0;
  double obstacle_clear_visible_until_s_ = 0.0;
  double traffic_block_visible_until_s_ = 0.0;
  double traffic_reopen_visible_until_s_ = 0.0;
  double traffic_wait_visible_until_s_ = 0.0;
  double traffic_go_visible_until_s_ = 0.0;
  double speed_limit_visible_until_s_ = 0.0;
  double speed_restore_visible_until_s_ = 0.0;
  double payload_load_started_s_ = 0.0;
  double payload_unload_started_s_ = 0.0;
  double top_module_action_started_s_ = 0.0;
  std::string payload_last_action_ = "none";
  std::string payload_top_module_;
  std::string door_status_ = "closed";
  std::string dock_state_ = "available";
  std::string lift_status_ = "floor_1_idle";
  std::string localization_state_ = "UNKNOWN";
  std::string robot2_target_station_ = "dock";
  std::string pending_confirmation_station_;
  std::string traffic_owner_id_;
  std::string traffic_reservation_type_;
  std::string traffic_block_from_;
  std::string traffic_block_to_;
  std::string traffic_reopen_from_;
  std::string traffic_reopen_to_;
  Pose2D robot_pose_;
  Pose2D robot2_pose_;
  Pose2D payload_delivered_pose_;
  DisplayPose payload_display_pose_{-3.35, -0.45, 0.18, 0.0};
  std::map<std::string, Pose2D> stations_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot2_odom_sub_;
  rclcpp::Subscription<robot_interfaces::msg::PayloadState>::SharedPtr payload_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizationHealth>::SharedPtr localization_sub_;
  rclcpp::Subscription<robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_takeover_sub_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::Client<robot_interfaces_facility::srv::GetDoorState>::SharedPtr door_client_;
  rclcpp::Client<robot_interfaces_mission::srv::GetDockState>::SharedPtr dock_client_;
  rclcpp::Client<robot_interfaces_facility::srv::GetLiftState>::SharedPtr lift_client_;
  rclcpp::Client<robot_interfaces_facility::srv::GetPendingConfirmations>::SharedPtr pending_client_;
  rclcpp::Client<robot_interfaces_fleet::srv::ListFleetRobots>::SharedPtr fleet_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedPtr events_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ListTrafficReservations>::SharedPtr traffic_client_;
  rclcpp::Client<robot_interfaces_mission::srv::DetectTrafficDeadlock>::SharedPtr deadlock_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmrSimGazeboEntityBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
