#include <chrono>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_interfaces/msg/payload_state.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_mission/srv/detect_traffic_deadlock.hpp"
#include "robot_interfaces_facility/srv/get_door_state.hpp"
#include "robot_interfaces_facility/srv/get_lift_state.hpp"
#include "robot_interfaces_business/srv/get_operator_snapshot.hpp"
#include "robot_interfaces_facility/srv/get_pending_confirmations.hpp"
#include "robot_interfaces_mission/srv/list_docks.hpp"
#include "robot_interfaces_facility/srv/list_facility_resources.hpp"
#include "robot_interfaces_mission/srv/list_mission_events.hpp"
#include "robot_interfaces_mission/srv/list_traffic_reservations.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct FacilityState {
  std::string type;
  std::string station_id;
  bool available = true;
  std::string status = "available";
  std::string holder_id;
};

struct TrafficReservation {
  std::string resource_id;
  std::string owner_id;
  std::string type;
};

struct PendingConfirmation {
  std::string confirmation_id;
  std::string station_id;
  std::string action;
  std::string mission_id;
  std::string state;
};

struct MissionEvent {
  std::string mission_id;
  std::string state;
  std::string message;
};

struct FleetRobotSummary {
  std::string id;
  std::string state;
  std::string station_id;
  double battery_voltage = 0.0;
  bool enabled = true;
};

std_msgs::msg::ColorRGBA Color(const double r, const double g, const double b, const double a = 1.0) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

geometry_msgs::msg::Point Point(const double x, const double y, const double z) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::msg::Pose Pose(const double x, const double y, const double z) {
  geometry_msgs::msg::Pose pose;
  pose.position = Point(x, y, z);
  pose.orientation.w = 1.0;
  return pose;
}

bool Contains(const std::string& text, const std::string& needle) {
  return text.find(needle) != std::string::npos;
}

std::string Truncate(const std::string& text, const std::size_t limit) {
  if (text.size() <= limit) {
    return text;
  }
  if (limit <= 3) {
    return text.substr(0, limit);
  }
  return text.substr(0, limit - 3) + "...";
}

}  // namespace

class AmrSimVisualizerNode final : public rclcpp::Node {
 public:
  AmrSimVisualizerNode() : Node("amr_sim_visualizer_node") {
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/amr_simulation/markers");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/model/mobile_robot/odometry");
    timeline_topic_ =
        declare_parameter<std::string>("timeline_topic", "/amr_simulation/demo_timeline");

    InitLayout();
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, rclcpp::QoS(1).transient_local());
    timeline_sub_ = create_subscription<std_msgs::msg::String>(
        timeline_topic_, rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::SharedPtr msg) { demo_timeline_text_ = msg->data; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          robot_x_ = msg->pose.pose.position.x;
          robot_y_ = msg->pose.pose.position.y;
          have_robot_pose_ = true;
        });
    current_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/navigate_sequence/current_goal", rclcpp::QoS(1).transient_local(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          current_goal_x_ = msg->pose.position.x;
          current_goal_y_ = msg->pose.position.y;
          have_current_goal_ = true;
        });
    payload_sub_ = create_subscription<robot_interfaces::msg::PayloadState>(
        "/payload/state", 10, [this](robot_interfaces::msg::PayloadState::SharedPtr msg) {
          payload_loaded_ = msg->loaded;
          payload_id_ = msg->payload_id.empty() ? "payload_box" : msg->payload_id;
          payload_state_ = msg->state;
          payload_module_ = msg->top_module_type;
          payload_action_ = msg->last_action;
        });
    localization_sub_ = create_subscription<robot_interfaces::msg::LocalizationHealth>(
        "/localization_health", 10, [this](robot_interfaces::msg::LocalizationHealth::SharedPtr msg) {
          localization_state_ = msg->state.empty() ? (msg->localized ? "LOCALIZED" : "LOST") : msg->state;
          localization_station_ = msg->station_id;
          localization_covariance_xy_ = msg->covariance_xy;
          localization_threshold_ = msg->covariance_threshold;
          localized_ = msg->localized;
        });
    safety_sub_ = create_subscription<robot_interfaces::msg::SafetyState>(
        "/safety_state", 10, [this](robot_interfaces::msg::SafetyState::SharedPtr msg) {
          safety_state_ = msg->state;
          safety_stop_ = msg->safety_stop;
          obstacle_blocked_ = msg->obstacle_blocked;
          speed_limit_mps_ = msg->effective_speed_limit_mps;
          stop_reason_ = msg->stop_reason;
          blockage_id_ = msg->blockage_id;
        });

    facility_client_ = create_client<robot_interfaces_facility::srv::ListFacilityResources>("/v2/list_facility_resources");
    docks_client_ = create_client<robot_interfaces_mission::srv::ListDocks>("/v2/list_docks");
    door_client_ = create_client<robot_interfaces_facility::srv::GetDoorState>("/v2/get_door_state");
    lift_client_ = create_client<robot_interfaces_facility::srv::GetLiftState>("/v2/get_lift_state");
    traffic_client_ = create_client<robot_interfaces_mission::srv::ListTrafficReservations>("/v2/list_traffic_reservations");
    pending_client_ = create_client<robot_interfaces_facility::srv::GetPendingConfirmations>("/v2/get_pending_confirmations");
    events_client_ = create_client<robot_interfaces_mission::srv::ListMissionEvents>("/v2/list_mission_events");
    snapshot_client_ = create_client<robot_interfaces_business::srv::GetOperatorSnapshot>("/v2/get_operator_snapshot");
    deadlock_client_ = create_client<robot_interfaces_mission::srv::DetectTrafficDeadlock>("/v2/detect_traffic_deadlock");
    timer_ = create_wall_timer(500ms, [this]() { Tick(); });
  }

 private:
  void InitLayout() {
    stations_ = {
        {"dock", {-3.9, -3.8, 3.14}},
        {"receiving", {-3.7, 0.0, 0.0}},
        {"storage_a", {1.5, 1.5, 1.57}},
        {"storage_b", {-2.0, 2.0, 1.57}},
        {"packing", {2.0, -2.0, -1.57}},
        {"traffic_wait", {-1.0, -4.2, 0.0}},
        {"lift_lobby", {3.65, 2.85, 1.57}},
        {"floor_2_dropoff", {3.65, 3.8, 1.57}},
    };
    routes_ = {
        {"dock", "receiving"},
        {"receiving", "storage_a"},
        {"receiving", "storage_b"},
        {"storage_a", "packing"},
        {"storage_b", "packing"},
        {"packing", "dock"},
        {"packing", "lift_lobby"},
        {"lift_lobby", "floor_2_dropoff"},
    };
    facilities_ = {
        {"charger_main", {"charger", "dock", true, "available", ""}},
        {"receiving_door", {"door", "receiving", true, "closed", ""}},
        {"freight_elevator", {"elevator", "lift_lobby", true, "floor_1_idle", ""}},
    };
  }

  void Tick() {
    PollFacilities();
    PollDocks();
    PollDoorState();
    PollLiftState();
    PollTraffic();
    PollPendingConfirmations();
    PollMissionEvents();
    PollOperatorSnapshot();
    PollDeadlock();
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    AddDeleteAll(&markers, id++);
    AddFloorLabels(&markers, id);
    AddRoutes(&markers, id);
    AddTrafficReservations(&markers, id);
    AddStations(&markers, id);
    AddFacilities(&markers, id);
    AddPendingConfirmations(&markers, id);
    AddPayload(&markers, id);
    AddSafetyMarkers(&markers, id);
    AddLocalizationMarkers(&markers, id);
    AddMissionPanel(&markers, id);
    AddRobotStatus(&markers, id);
    marker_pub_->publish(markers);
  }

  void PollFacilities() {
    if (facility_request_pending_ || !facility_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::ListFacilityResources::Request>();
    request->include_disabled = true;
    facility_request_pending_ = true;
    facility_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_facility::srv::ListFacilityResources>::SharedFuture future) {
          facility_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          for (std::size_t i = 0; i < response->resource_ids.size(); ++i) {
            const auto& resource_id = response->resource_ids[i];
            if ((resource_id == "receiving_door" && door_client_->service_is_ready()) ||
                (resource_id == "freight_elevator" && lift_client_->service_is_ready())) {
              continue;
            }
            auto& state = facilities_[resource_id];
            if (i < response->resource_types.size()) {
              state.type = response->resource_types[i];
            }
            if (i < response->station_ids.size()) {
              state.station_id = response->station_ids[i];
            }
            if (i < response->available.size()) {
              state.available = response->available[i];
            }
            if (i < response->status.size()) {
              state.status = response->status[i];
            }
            if (i < response->holder_ids.size()) {
              state.holder_id = response->holder_ids[i];
            }
          }
        });
  }

  void PollDocks() {
    if (dock_request_pending_ || !docks_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::ListDocks::Request>();
    request->include_disabled = true;
    dock_request_pending_ = true;
    docks_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_mission::srv::ListDocks>::SharedFuture future) {
          dock_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          if (!response->states.empty()) {
            dock_state_ = response->states.front();
          }
        });
  }

  void PollDoorState() {
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
          if (!response->success) {
            return;
          }
          auto& state = facilities_[response->door_id];
          state.type = "door";
          state.station_id = response->station_id;
          state.available = response->available;
          state.status = response->status;
          state.holder_id = response->holder_id;
        });
  }

  void PollLiftState() {
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
          if (!response->success) {
            return;
          }
          auto& state = facilities_[response->lift_id];
          state.type = "elevator";
          state.station_id = response->station_id;
          state.available = response->available;
          state.status = response->status;
          state.holder_id = response->holder_id;
        });
  }

  void PollTraffic() {
    if (traffic_request_pending_ || !traffic_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::ListTrafficReservations::Request>();
    traffic_request_pending_ = true;
    traffic_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_mission::srv::ListTrafficReservations>::SharedFuture future) {
          traffic_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          traffic_reservations_.clear();
          for (std::size_t i = 0; i < response->resource_ids.size(); ++i) {
            TrafficReservation reservation;
            reservation.resource_id = response->resource_ids[i];
            if (i < response->owner_ids.size()) {
              reservation.owner_id = response->owner_ids[i];
            }
            if (i < response->reservation_types.size()) {
              reservation.type = response->reservation_types[i];
            }
            traffic_reservations_.push_back(reservation);
          }
        });
  }

  void PollPendingConfirmations() {
    if (pending_request_pending_ || !pending_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::GetPendingConfirmations::Request>();
    pending_request_pending_ = true;
    pending_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_facility::srv::GetPendingConfirmations>::SharedFuture future) {
          pending_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          pending_confirmations_.clear();
          for (std::size_t i = 0; i < response->confirmation_ids.size(); ++i) {
            PendingConfirmation confirmation;
            confirmation.confirmation_id = response->confirmation_ids[i];
            if (i < response->station_ids.size()) {
              confirmation.station_id = response->station_ids[i];
            }
            if (i < response->actions.size()) {
              confirmation.action = response->actions[i];
            }
            if (i < response->mission_ids.size()) {
              confirmation.mission_id = response->mission_ids[i];
            }
            if (i < response->states.size()) {
              confirmation.state = response->states[i];
            }
            pending_confirmations_.push_back(confirmation);
          }
        });
  }

  void PollMissionEvents() {
    if (events_request_pending_ || !events_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::ListMissionEvents::Request>();
    request->limit = 6;
    events_request_pending_ = true;
    events_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedFuture future) {
          events_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          recent_events_.clear();
          for (std::size_t i = 0; i < response->mission_ids.size(); ++i) {
            MissionEvent event;
            event.mission_id = response->mission_ids[i];
            if (i < response->states.size()) {
              event.state = response->states[i];
            }
            if (i < response->messages.size()) {
              event.message = response->messages[i];
            }
            recent_events_.push_back(event);
          }
        });
  }

  void PollOperatorSnapshot() {
    if (snapshot_request_pending_ || !snapshot_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_business::srv::GetOperatorSnapshot::Request>();
    request->event_limit = 3;
    snapshot_request_pending_ = true;
    snapshot_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_business::srv::GetOperatorSnapshot>::SharedFuture future) {
          snapshot_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          mission_state_ = response->mission_state;
          mission_message_ = response->mission_message;
          active_mission_id_ = response->active_mission_id;
          queue_size_ = response->queue_size;
          queued_mission_ids_ = response->queued_mission_ids;
          mission_paused_ = response->paused;
          remote_task_count_ = static_cast<int>(response->remote_task_ids.size());
          fleet_robot_count_ = static_cast<int>(response->fleet_robot_ids.size());
          business_type_count_ = static_cast<int>(response->business_types.size());
          UpdatePrimaryRobot(*response);
        });
  }

  void PollDeadlock() {
    if (deadlock_request_pending_ || !deadlock_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_mission::srv::DetectTrafficDeadlock::Request>();
    deadlock_request_pending_ = true;
    deadlock_client_->async_send_request(
        request, [this](rclcpp::Client<robot_interfaces_mission::srv::DetectTrafficDeadlock>::SharedFuture future) {
          deadlock_request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          traffic_deadlocked_ = response->deadlocked;
          deadlock_missions_ = response->mission_ids;
        });
  }

  visualization_msgs::msg::Marker BaseMarker(const int id, const std::string& ns, const int type) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime.sec = 1;
    marker.lifetime.nanosec = 200000000;
    return marker;
  }

  void AddDeleteAll(visualization_msgs::msg::MarkerArray* markers, const int id) const {
    auto marker = BaseMarker(id, "reset", visualization_msgs::msg::Marker::CUBE);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers->markers.push_back(marker);
  }

  void AddFloorLabels(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    auto floor_1 = TextMarker(id++, "floor_label", "F1 warehouse / charging / packing", 0.0, -5.25, 0.05, 0.28);
    markers->markers.push_back(floor_1);
    auto floor_2 = TextMarker(id++, "floor_label", "F2 simulated dropoff zone", 3.65, 3.35, 0.9, 0.22);
    markers->markers.push_back(floor_2);
  }

  void AddRoutes(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    auto route = BaseMarker(id++, "station_routes", visualization_msgs::msg::Marker::LINE_LIST);
    route.scale.x = 0.035;
    route.color = Color(0.35, 0.50, 0.66, 0.45);
    for (const auto& edge : routes_) {
      const auto from = stations_.find(edge.first);
      const auto to = stations_.find(edge.second);
      if (from == stations_.end() || to == stations_.end()) {
        continue;
      }
      route.points.push_back(Point(from->second.x, from->second.y, 0.04));
      route.points.push_back(Point(to->second.x, to->second.y, 0.04));
    }
    markers->markers.push_back(route);

    auto traffic = BaseMarker(id++, "traffic_intersection", visualization_msgs::msg::Marker::CYLINDER);
    traffic.pose = Pose(0.0, 0.0, 0.03);
    traffic.scale.x = 1.0;
    traffic.scale.y = 1.0;
    traffic.scale.z = 0.03;
    traffic.color = Color(1.0, 0.72, 0.1, 0.35);
    markers->markers.push_back(traffic);
    markers->markers.push_back(TextMarker(id++, "traffic_intersection", "traffic lock zone", 0.0, 0.0, 0.35, 0.18));
  }

  void AddTrafficReservations(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    for (const auto& reservation : traffic_reservations_) {
      if (Contains(reservation.resource_id, "route_edge:")) {
        const auto route = ParseRouteEdge(reservation.resource_id);
        if (route.first.empty() || route.second.empty()) {
          continue;
        }
        const auto from = stations_.find(route.first);
        const auto to = stations_.find(route.second);
        if (from == stations_.end() || to == stations_.end()) {
          continue;
        }
        auto line = BaseMarker(id++, "traffic_reservations", visualization_msgs::msg::Marker::LINE_LIST);
        line.scale.x = reservation.type == "blocked" ? 0.12 : 0.08;
        line.color = reservation.type == "blocked" ? Color(1.0, 0.05, 0.05, 0.9)
                                                   : Color(1.0, 0.65, 0.05, 0.8);
        line.points.push_back(Point(from->second.x, from->second.y, 0.18));
        line.points.push_back(Point(to->second.x, to->second.y, 0.18));
        markers->markers.push_back(line);
        const double x = (from->second.x + to->second.x) * 0.5;
        const double y = (from->second.y + to->second.y) * 0.5;
        markers->markers.push_back(TextMarker(
            id++, "traffic_reservations",
            reservation.type + ": " + Truncate(reservation.owner_id, 24), x, y, 0.55, 0.14));
      } else if (Contains(reservation.resource_id, "route_node:")) {
        const auto station_id = reservation.resource_id.substr(std::string("route_node:").size());
        const auto station = stations_.find(station_id);
        if (station == stations_.end()) {
          continue;
        }
        auto node = BaseMarker(id++, "traffic_reservations", visualization_msgs::msg::Marker::SPHERE);
        node.pose = Pose(station->second.x, station->second.y, 0.28);
        node.scale.x = 0.35;
        node.scale.y = 0.35;
        node.scale.z = 0.35;
        node.color = Color(1.0, 0.5, 0.05, 0.65);
        markers->markers.push_back(node);
      }
    }
    if (traffic_deadlocked_) {
      auto deadlock = BaseMarker(id++, "traffic_deadlock", visualization_msgs::msg::Marker::CYLINDER);
      deadlock.pose = Pose(0.0, 0.0, 0.08);
      deadlock.scale.x = 1.35;
      deadlock.scale.y = 1.35;
      deadlock.scale.z = 0.08;
      deadlock.color = Color(1.0, 0.0, 0.0, 0.55);
      markers->markers.push_back(deadlock);
      markers->markers.push_back(TextMarker(
          id++, "traffic_deadlock", "deadlock: " + Truncate(Join(deadlock_missions_, ","), 36),
          0.0, -0.65, 0.65, 0.16));
    }
  }

  void AddStations(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    for (const auto& [station_id, pose] : stations_) {
      auto marker = BaseMarker(id++, "stations", visualization_msgs::msg::Marker::CYLINDER);
      marker.pose = Pose(pose.x, pose.y, 0.025);
      marker.scale.x = 0.55;
      marker.scale.y = 0.55;
      marker.scale.z = 0.05;
      marker.color = station_id == "dock" ? Color(0.1, 0.9, 0.25, 0.55) : Color(0.1, 0.3, 1.0, 0.45);
      markers->markers.push_back(marker);
      markers->markers.push_back(TextMarker(id++, "station_labels", station_id, pose.x, pose.y, 0.35, 0.16));
    }
  }

  void AddFacilities(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    AddDockMarkers(markers, id);
    AddDoorMarkers(markers, id);
    AddElevatorMarkers(markers, id);
  }

  void AddDockMarkers(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    const auto dock = stations_.at("dock");
    auto zone = BaseMarker(id++, "dock_zone", visualization_msgs::msg::Marker::CUBE);
    zone.pose = Pose(dock.x, dock.y, 0.04);
    zone.scale.x = 1.25;
    zone.scale.y = 0.95;
    zone.scale.z = 0.08;
    const bool charging = dock_state_.find("CHARG") != std::string::npos || dock_state_.find("charging") != std::string::npos;
    zone.color = charging ? Color(0.0, 1.0, 0.25, 0.55) : Color(0.0, 0.65, 0.25, 0.28);
    markers->markers.push_back(zone);
    markers->markers.push_back(TextMarker(id++, "dock_zone", "charging dock: " + dock_state_, dock.x, dock.y - 0.55, 0.45, 0.17));
  }

  void AddDoorMarkers(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    const auto station = stations_.at("receiving");
    const auto facility = facilities_.find("receiving_door");
    const std::string status = facility == facilities_.end() ? "closed" : facility->second.status;
    const std::string holder = facility == facilities_.end() ? "" : facility->second.holder_id;
    const bool open = status.find("open") != std::string::npos || status.find("occupied") != std::string::npos;
    auto door = BaseMarker(id++, "door", visualization_msgs::msg::Marker::CUBE);
    door.pose = Pose(station.x - 0.45, station.y, 0.45);
    door.scale.x = open ? 0.08 : 0.12;
    door.scale.y = open ? 0.25 : 1.0;
    door.scale.z = 0.9;
    door.color = open ? Color(0.1, 0.9, 0.25, 0.7) : Color(0.9, 0.15, 0.1, 0.75);
    markers->markers.push_back(door);
    const std::string holder_suffix =
        holder.empty() ? "" : " / holder: " + Truncate(holder, 22);
    markers->markers.push_back(TextMarker(
        id++, "door", "door: " + status + holder_suffix, station.x - 0.45, station.y + 0.7,
        0.95, 0.16));
  }

  void AddElevatorMarkers(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    const auto lift = stations_.at("lift_lobby");
    const auto facility = facilities_.find("freight_elevator");
    const std::string status = facility == facilities_.end() ? "floor_1_idle" : facility->second.status;
    const std::string holder = facility == facilities_.end() ? "" : facility->second.holder_id;
    auto cabin = BaseMarker(id++, "elevator", visualization_msgs::msg::Marker::CUBE);
    cabin.pose = Pose(lift.x, lift.y, 0.08);
    cabin.scale.x = 1.1;
    cabin.scale.y = 1.1;
    cabin.scale.z = 0.08;
    cabin.color = status.find("occupied") != std::string::npos ? Color(1.0, 0.75, 0.1, 0.55) : Color(0.55, 0.25, 0.95, 0.35);
    markers->markers.push_back(cabin);
    const std::string holder_suffix =
        holder.empty() ? "" : " / holder: " + Truncate(holder, 22);
    markers->markers.push_back(
        TextMarker(id++, "elevator", "elevator: " + status + holder_suffix, lift.x, lift.y, 0.55, 0.16));

    auto floor_2 = BaseMarker(id++, "elevator", visualization_msgs::msg::Marker::CUBE);
    floor_2.pose = Pose(3.65, 3.8, 0.85);
    floor_2.scale.x = 1.2;
    floor_2.scale.y = 1.2;
    floor_2.scale.z = 0.06;
    floor_2.color = Color(0.45, 0.45, 0.55, 0.45);
    markers->markers.push_back(floor_2);
  }

  void AddPayload(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    const double x = payload_loaded_ && have_robot_pose_ ? robot_x_ : stations_.at("receiving").x + 0.35;
    const double y = payload_loaded_ && have_robot_pose_ ? robot_y_ : stations_.at("receiving").y - 0.45;
    const double z = payload_loaded_ ? 0.75 : 0.18;
    auto payload = BaseMarker(id++, "payload", visualization_msgs::msg::Marker::CUBE);
    payload.pose = Pose(x, y, z);
    payload.scale.x = 0.42;
    payload.scale.y = 0.32;
    payload.scale.z = 0.26;
    payload.color = payload_loaded_ ? Color(0.05, 0.75, 0.95, 0.9) : Color(0.9, 0.55, 0.1, 0.85);
    markers->markers.push_back(payload);
    const std::string label = payload_loaded_ ? "payload on robot: " + payload_id_ : "waiting payload";
    markers->markers.push_back(TextMarker(id++, "payload", label, x, y, z + 0.3, 0.15));
    if (!payload_state_.empty() || !payload_action_.empty() || !payload_module_.empty()) {
      const std::string state = "payload: " + Truncate(payload_state_, 18) +
                                (payload_action_.empty() ? "" : " / " + Truncate(payload_action_, 14));
      markers->markers.push_back(TextMarker(id++, "payload", state, x, y, z + 0.5, 0.13));
    }
  }

  void AddPendingConfirmations(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    for (const auto& confirmation : pending_confirmations_) {
      const auto station = stations_.find(confirmation.station_id);
      if (station == stations_.end()) {
        continue;
      }
      auto kiosk = BaseMarker(id++, "pending_confirmations", visualization_msgs::msg::Marker::CUBE);
      kiosk.pose = Pose(station->second.x + 0.38, station->second.y + 0.38, 0.28);
      kiosk.scale.x = 0.22;
      kiosk.scale.y = 0.22;
      kiosk.scale.z = 0.55;
      kiosk.color = Color(1.0, 0.85, 0.05, 0.9);
      markers->markers.push_back(kiosk);
      markers->markers.push_back(TextMarker(
          id++, "pending_confirmations",
          "confirm: " + Truncate(confirmation.action, 22), station->second.x + 0.42,
          station->second.y + 0.62, 0.75, 0.14));
    }
  }

  void AddSafetyMarkers(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    auto speed_zone = BaseMarker(id++, "safety", visualization_msgs::msg::Marker::CYLINDER);
    speed_zone.pose = Pose(1.2, -0.25, 0.02);
    speed_zone.scale.x = 1.1;
    speed_zone.scale.y = 1.1;
    speed_zone.scale.z = 0.025;
    speed_zone.color = speed_limit_mps_ > 0.0 && speed_limit_mps_ < 0.5 ? Color(0.1, 0.7, 1.0, 0.35)
                                                                        : Color(0.1, 0.7, 1.0, 0.16);
    markers->markers.push_back(speed_zone);
    std::ostringstream speed_label;
    speed_label << "speed limit: ";
    if (speed_limit_mps_ > 0.0) {
      speed_label << speed_limit_mps_ << " mps";
    } else {
      speed_label << "normal";
    }
    markers->markers.push_back(TextMarker(id++, "safety", speed_label.str(), 1.2, -0.25, 0.36, 0.14));

    if (safety_stop_ || obstacle_blocked_) {
      const double x = have_robot_pose_ ? robot_x_ + 0.65 : 0.55;
      const double y = have_robot_pose_ ? robot_y_ : 0.55;
      auto obstacle = BaseMarker(id++, "safety", visualization_msgs::msg::Marker::CUBE);
      obstacle.pose = Pose(x, y, 0.35);
      obstacle.scale.x = 0.42;
      obstacle.scale.y = 0.42;
      obstacle.scale.z = 0.7;
      obstacle.color = Color(1.0, 0.05, 0.05, 0.85);
      markers->markers.push_back(obstacle);
      markers->markers.push_back(TextMarker(
          id++, "safety", "blocked: " + Truncate(blockage_id_.empty() ? stop_reason_ : blockage_id_, 26),
          x, y, 0.9, 0.15));
    }
  }

  void AddLocalizationMarkers(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    const double x = have_robot_pose_ ? robot_x_ : -0.6;
    const double y = have_robot_pose_ ? robot_y_ : 0.6;
    auto covariance = BaseMarker(id++, "localization", visualization_msgs::msg::Marker::CYLINDER);
    covariance.pose = Pose(x, y, 0.03);
    const double radius = std::clamp(localization_covariance_xy_ * 6.0, 0.45, 1.8);
    covariance.scale.x = radius;
    covariance.scale.y = radius;
    covariance.scale.z = 0.035;
    covariance.color = localized_ ? Color(0.15, 1.0, 0.25, 0.2) : Color(1.0, 0.0, 0.0, 0.32);
    markers->markers.push_back(covariance);
    std::ostringstream label;
    label << "localization: " << (localization_state_.empty() ? "unknown" : localization_state_);
    if (localization_threshold_ > 0.0) {
      label << " cov=" << localization_covariance_xy_;
    }
    markers->markers.push_back(TextMarker(id++, "localization", label.str(), x, y - 0.55, 0.72, 0.14));
  }

  void AddMissionPanel(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    std::vector<std::string> lines;
    lines.push_back("AMR任务看板 / task_board");
    lines.push_back("当前机器人 robot: " + RobotSummary());
    lines.push_back("当前任务 task: " + ActiveTaskSummary());
    lines.push_back("当前阶段 phase: " + StageSummary());
    lines.push_back("目标站点 target: " + GoalSummary());
    lines.push_back("payload: " + PayloadSummary());
    lines.push_back("dock: " + Truncate(dock_state_, 42));
    lines.push_back("safety: " + SafetySummary());
    lines.push_back("traffic: " + TrafficSummary());
    lines.push_back("queue: " + std::to_string(queue_size_) + "  fleet: " +
                    std::to_string(fleet_robot_count_) + "  remote: " +
                    std::to_string(remote_task_count_) + "  business: " +
                    std::to_string(business_type_count_));
    if (!mission_message_.empty()) {
      lines.push_back("message: " + Truncate(mission_message_, 54));
    }
    lines.push_back("最近事件 events:");
    const std::size_t event_count = std::min<std::size_t>(recent_events_.size(), 3);
    for (std::size_t i = 0; i < event_count; ++i) {
      lines.push_back("- " + Truncate(
          recent_events_[i].state + " " + recent_events_[i].mission_id + " " +
              recent_events_[i].message,
          58));
    }
    markers->markers.push_back(TextMarker(id++, "mission_panel", Join(lines, "\n"), -4.75, 4.85, 1.25, 0.115));
  }

  void AddRobotStatus(visualization_msgs::msg::MarkerArray* markers, int& id) const {
    if (!have_robot_pose_) {
      return;
    }
    markers->markers.push_back(TextMarker(id++, "robot_status", "mobile_robot", robot_x_, robot_y_, 0.95, 0.16));
  }

  visualization_msgs::msg::Marker TextMarker(
      const int id, const std::string& ns, const std::string& text, const double x, const double y,
      const double z, const double height) const {
    auto marker = BaseMarker(id, ns, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    marker.pose = Pose(x, y, z);
    marker.scale.z = height;
    marker.color = Color(1.0, 1.0, 1.0, 0.95);
    marker.text = text;
    return marker;
  }

  static std::pair<std::string, std::string> ParseRouteEdge(const std::string& resource_id) {
    const std::string prefix = "route_edge:";
    if (!Contains(resource_id, prefix)) {
      return {"", ""};
    }
    const auto start = resource_id.find(prefix) + prefix.size();
    const auto separator = resource_id.find("__", start);
    if (separator == std::string::npos) {
      return {"", ""};
    }
    return {resource_id.substr(start, separator - start), resource_id.substr(separator + 2)};
  }

  static std::string Join(const std::vector<std::string>& values, const std::string& separator) {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        out << separator;
      }
      out << values[i];
    }
    return out.str();
  }

  void UpdatePrimaryRobot(const robot_interfaces_business::srv::GetOperatorSnapshot::Response& response) {
    primary_robot_ = FleetRobotSummary{};
    if (response.fleet_robot_ids.empty()) {
      primary_robot_.id = "robot_1";
      primary_robot_.state = "unknown";
      primary_robot_.station_id = "unknown";
      primary_robot_.enabled = true;
      return;
    }
    std::size_t selected = 0;
    for (std::size_t i = 0; i < response.fleet_robot_ids.size(); ++i) {
      if (response.fleet_robot_ids[i] == "robot_1") {
        selected = i;
        break;
      }
    }
    primary_robot_.id = response.fleet_robot_ids[selected];
    if (selected < response.fleet_states.size()) {
      primary_robot_.state = response.fleet_states[selected];
    }
    if (selected < response.fleet_current_station_ids.size()) {
      primary_robot_.station_id = response.fleet_current_station_ids[selected];
    }
    if (selected < response.fleet_battery_voltage.size()) {
      primary_robot_.battery_voltage = response.fleet_battery_voltage[selected];
    }
    if (selected < response.fleet_enabled.size()) {
      primary_robot_.enabled = response.fleet_enabled[selected];
    }
  }

  std::string RobotSummary() const {
    std::ostringstream out;
    out << (primary_robot_.id.empty() ? "robot_1" : primary_robot_.id);
    if (!primary_robot_.station_id.empty()) {
      out << " @ " << primary_robot_.station_id;
    }
    if (!primary_robot_.state.empty()) {
      out << " / " << primary_robot_.state;
    }
    if (primary_robot_.battery_voltage > 0.0) {
      out << " / " << FormatDouble(primary_robot_.battery_voltage, 1) << "V";
    }
    if (!primary_robot_.enabled) {
      out << " / disabled";
    }
    return out.str();
  }

  std::string ActiveTaskSummary() const {
    if (!active_mission_id_.empty()) {
      return Truncate(active_mission_id_, 50);
    }
    if (!queued_mission_ids_.empty()) {
      return "waiting " + Truncate(queued_mission_ids_.front(), 40);
    }
    return "idle";
  }

  std::string StageSummary() const {
    std::string stage = mission_state_.empty() ? "WAITING" : mission_state_;
    if (mission_paused_) {
      stage += " paused";
    }
    if (!demo_timeline_text_.empty()) {
      auto first_line = demo_timeline_text_;
      const auto newline = first_line.find('\n');
      if (newline != std::string::npos) {
        first_line = first_line.substr(0, newline);
      }
      stage += " / " + first_line;
    }
    return Truncate(stage, 58);
  }

  std::string GoalSummary() const {
    if (!have_current_goal_) {
      return "waiting_navigation_goal";
    }
    std::string nearest_station;
    double nearest_distance = std::numeric_limits<double>::max();
    for (const auto& [station_id, pose] : stations_) {
      const double dx = pose.x - current_goal_x_;
      const double dy = pose.y - current_goal_y_;
      const double distance = std::hypot(dx, dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_station = station_id;
      }
    }
    std::ostringstream out;
    if (!nearest_station.empty() && nearest_distance <= 0.8) {
      out << nearest_station << " ";
    }
    out << "(" << FormatDouble(current_goal_x_, 1) << ", " << FormatDouble(current_goal_y_, 1) << ")";
    return out.str();
  }

  std::string PayloadSummary() const {
    std::string summary = payload_loaded_ ? "loaded " : "not_loaded ";
    summary += payload_state_.empty() ? "unknown" : payload_state_;
    if (!payload_action_.empty()) {
      summary += " / " + payload_action_;
    }
    if (!payload_module_.empty()) {
      summary += " / " + payload_module_;
    }
    return Truncate(summary, 48);
  }

  std::string SafetySummary() const {
    std::ostringstream out;
    out << (safety_state_.empty() ? "unknown" : safety_state_);
    if (safety_stop_) {
      out << " / stop";
    }
    if (speed_limit_mps_ > 0.0) {
      out << " / limit " << FormatDouble(speed_limit_mps_, 2) << "mps";
    }
    if (!blockage_id_.empty()) {
      out << " / " << blockage_id_;
    } else if (!stop_reason_.empty()) {
      out << " / " << stop_reason_;
    }
    return Truncate(out.str(), 54);
  }

  std::string TrafficSummary() const {
    if (traffic_reservations_.empty()) {
      return "none";
    }
    std::vector<std::string> parts;
    const std::size_t count = std::min<std::size_t>(traffic_reservations_.size(), 2);
    for (std::size_t i = 0; i < count; ++i) {
      parts.push_back(ShortTrafficResource(traffic_reservations_[i]));
    }
    std::string summary = std::to_string(traffic_reservations_.size()) + " " + Join(parts, " | ");
    return Truncate(summary, 58);
  }

  static std::string ShortTrafficResource(const TrafficReservation& reservation) {
    std::string resource = reservation.resource_id;
    const std::string route_edge_prefix = "route_edge:";
    const std::string route_node_prefix = "route_node:";
    if (resource.rfind(route_edge_prefix, 0) == 0) {
      resource = resource.substr(route_edge_prefix.size());
    } else if (resource.rfind(route_node_prefix, 0) == 0) {
      resource = resource.substr(route_node_prefix.size());
    }
    if (!reservation.owner_id.empty()) {
      resource += "<-" + reservation.owner_id;
    }
    return resource;
  }

  static std::string FormatDouble(const double value, const int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
  }

  std::string frame_id_;
  std::string marker_topic_;
  std::string odom_topic_;
  std::string timeline_topic_;
  std::map<std::string, Pose2D> stations_;
  std::vector<std::pair<std::string, std::string>> routes_;
  std::map<std::string, FacilityState> facilities_;
  std::vector<TrafficReservation> traffic_reservations_;
  std::vector<PendingConfirmation> pending_confirmations_;
  std::vector<MissionEvent> recent_events_;
  std::vector<std::string> deadlock_missions_;
  std::string dock_state_ = "available";
  bool payload_loaded_ = false;
  std::string payload_id_ = "payload_box";
  std::string payload_state_ = "waiting";
  std::string payload_module_;
  std::string payload_action_;
  std::string localization_state_ = "unknown";
  std::string localization_station_;
  double localization_covariance_xy_ = 0.0;
  double localization_threshold_ = 0.0;
  bool localized_ = true;
  std::string safety_state_ = "normal";
  bool safety_stop_ = false;
  bool obstacle_blocked_ = false;
  double speed_limit_mps_ = 0.0;
  std::string stop_reason_;
  std::string blockage_id_;
  std::string mission_state_;
  std::string mission_message_;
  std::string active_mission_id_;
  std::vector<std::string> queued_mission_ids_;
  std::string demo_timeline_text_;
  FleetRobotSummary primary_robot_;
  int queue_size_ = 0;
  bool mission_paused_ = false;
  int remote_task_count_ = 0;
  int fleet_robot_count_ = 0;
  int business_type_count_ = 0;
  bool traffic_deadlocked_ = false;
  bool have_robot_pose_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  bool have_current_goal_ = false;
  double current_goal_x_ = 0.0;
  double current_goal_y_ = 0.0;
  bool facility_request_pending_ = false;
  bool dock_request_pending_ = false;
  bool door_request_pending_ = false;
  bool lift_request_pending_ = false;
  bool traffic_request_pending_ = false;
  bool pending_request_pending_ = false;
  bool events_request_pending_ = false;
  bool snapshot_request_pending_ = false;
  bool deadlock_request_pending_ = false;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr timeline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_sub_;
  rclcpp::Subscription<robot_interfaces::msg::PayloadState>::SharedPtr payload_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizationHealth>::SharedPtr localization_sub_;
  rclcpp::Subscription<robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Client<robot_interfaces_facility::srv::ListFacilityResources>::SharedPtr facility_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ListDocks>::SharedPtr docks_client_;
  rclcpp::Client<robot_interfaces_facility::srv::GetDoorState>::SharedPtr door_client_;
  rclcpp::Client<robot_interfaces_facility::srv::GetLiftState>::SharedPtr lift_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ListTrafficReservations>::SharedPtr traffic_client_;
  rclcpp::Client<robot_interfaces_facility::srv::GetPendingConfirmations>::SharedPtr pending_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedPtr events_client_;
  rclcpp::Client<robot_interfaces_business::srv::GetOperatorSnapshot>::SharedPtr snapshot_client_;
  rclcpp::Client<robot_interfaces_mission::srv::DetectTrafficDeadlock>::SharedPtr deadlock_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmrSimVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
