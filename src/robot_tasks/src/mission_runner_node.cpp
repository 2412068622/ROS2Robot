#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/navigate_sequence.hpp"
#include "robot_interfaces/msg/chassis_state.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_interfaces/msg/payload_state.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_business/srv/cancel_workflow.hpp"
#include "robot_interfaces_business/srv/confirm_dropoff.hpp"
#include "robot_interfaces_business/srv/confirm_pick.hpp"
#include "robot_interfaces_business/srv/get_operator_snapshot.hpp"
#include "robot_interfaces_business/srv/get_order_fulfillment_status.hpp"
#include "robot_interfaces_business/srv/get_workflow_status.hpp"
#include "robot_interfaces_business/srv/list_business_order_types.hpp"
#include "robot_interfaces_business/srv/list_scenario_tasks.hpp"
#include "robot_interfaces_business/srv/list_scenario_workflows.hpp"
#include "robot_interfaces_business/srv/pause_workflow.hpp"
#include "robot_interfaces_business/srv/report_short_pick.hpp"
#include "robot_interfaces_business/srv/resume_workflow.hpp"
#include "robot_interfaces_business/srv/submit_business_order.hpp"
#include "robot_interfaces_business/srv/submit_milk_run.hpp"
#include "robot_interfaces_business/srv/submit_order.hpp"
#include "robot_interfaces_business/srv/submit_picking_wave.hpp"
#include "robot_interfaces_business/srv/submit_replenishment_order.hpp"
#include "robot_interfaces_business/srv/submit_scenario_task.hpp"
#include "robot_interfaces_business/srv/submit_scenario_workflow.hpp"
#include "robot_interfaces_business/srv/submit_transport_order.hpp"
#include "robot_interfaces_business/srv/submit_vda5050_order.hpp"
#include "robot_interfaces_facility/srv/confirm_load.hpp"
#include "robot_interfaces_facility/srv/confirm_station_action.hpp"
#include "robot_interfaces_facility/srv/confirm_unload.hpp"
#include "robot_interfaces_facility/srv/execute_facility_action.hpp"
#include "robot_interfaces_facility/srv/execute_top_module_action.hpp"
#include "robot_interfaces_facility/srv/get_door_state.hpp"
#include "robot_interfaces_facility/srv/get_lift_state.hpp"
#include "robot_interfaces_facility/srv/get_pending_confirmations.hpp"
#include "robot_interfaces_facility/srv/list_facility_resources.hpp"
#include "robot_interfaces_facility/srv/reject_station_action.hpp"
#include "robot_interfaces_facility/srv/release_facility_resource.hpp"
#include "robot_interfaces_facility/srv/release_lift_session.hpp"
#include "robot_interfaces_facility/srv/request_lift_session.hpp"
#include "robot_interfaces_facility/srv/request_station_confirmation.hpp"
#include "robot_interfaces_facility/srv/reserve_facility_resource.hpp"
#include "robot_interfaces_facility/srv/set_payload_loaded.hpp"
#include "robot_interfaces_fleet/srv/list_fleet_robots.hpp"
#include "robot_interfaces_fleet/srv/preview_fleet_station_task.hpp"
#include "robot_interfaces_fleet/srv/submit_fleet_station_task.hpp"
#include "robot_interfaces_fleet/srv/update_fleet_robot_state.hpp"
#include "robot_interfaces_fleet/srv/update_remote_task_state.hpp"
#include "robot_interfaces_mission/srv/block_station_route.hpp"
#include "robot_interfaces_mission/srv/cancel_queued_mission.hpp"
#include "robot_interfaces_mission/srv/cancel_station_order_batch.hpp"
#include "robot_interfaces_mission/srv/clear_traffic_reservation.hpp"
#include "robot_interfaces_mission/srv/detect_traffic_deadlock.hpp"
#include "robot_interfaces_mission/srv/enqueue_mission.hpp"
#include "robot_interfaces_mission/srv/estimate_mission_cost.hpp"
#include "robot_interfaces_mission/srv/estimate_station_sequence.hpp"
#include "robot_interfaces_mission/srv/estimate_station_transport.hpp"
#include "robot_interfaces_mission/srv/get_dock_state.hpp"
#include "robot_interfaces_mission/srv/get_mission_recovery_state.hpp"
#include "robot_interfaces_mission/srv/list_docks.hpp"
#include "robot_interfaces_mission/srv/list_mission_events.hpp"
#include "robot_interfaces_mission/srv/list_station_routes.hpp"
#include "robot_interfaces_mission/srv/list_stations.hpp"
#include "robot_interfaces_mission/srv/list_traffic_reservations.hpp"
#include "robot_interfaces_mission/srv/preempt_mission.hpp"
#include "robot_interfaces_mission/srv/reprioritize_batch.hpp"
#include "robot_interfaces_mission/srv/reprioritize_queued_mission.hpp"
#include "robot_interfaces_mission/srv/request_charging.hpp"
#include "robot_interfaces_mission/srv/request_docking.hpp"
#include "robot_interfaces_mission/srv/request_opportunity_charging.hpp"
#include "robot_interfaces_mission/srv/request_undocking.hpp"
#include "robot_interfaces_mission/srv/submit_station_order_batch.hpp"
#include "robot_interfaces_mission/srv/submit_station_sequence_task.hpp"
#include "robot_interfaces_mission/srv/submit_station_transport_order.hpp"
#include "robot_interfaces_mission/srv/unblock_station_route.hpp"
#include "robot_interfaces_navigation/srv/clear_obstacle_blockage.hpp"
#include "robot_interfaces_navigation/srv/report_obstacle_blockage.hpp"
#include "robot_interfaces_navigation/srv/request_relocalization.hpp"
#include "robot_interfaces_navigation/srv/set_dynamic_speed_limit.hpp"
#include "robot_interfaces_navigation/srv/set_initial_pose_from_station.hpp"
#include "robot_navigation/zone_catalog.hpp"
#include "robot_tasks/business_order_catalog.hpp"
#include "robot_tasks/dock_catalog.hpp"
#include "robot_tasks/docking_workflow.hpp"
#include "robot_tasks/facility_catalog.hpp"
#include "robot_tasks/facility_reservation.hpp"
#include "robot_tasks/facility_workflow.hpp"
#include "robot_tasks/fleet_catalog.hpp"
#include "robot_tasks/fleet_runtime_state.hpp"
#include "robot_tasks/fulfillment_workflow.hpp"
#include "robot_tasks/mission_charging_workflow.hpp"
#include "robot_tasks/mission_event_log.hpp"
#include "robot_tasks/mission_estimator.hpp"
#include "robot_tasks/mission_localization_workflow.hpp"
#include "robot_tasks/mission_preflight.hpp"
#include "robot_tasks/mission_profile.hpp"
#include "robot_tasks/mission_profile_builders.hpp"
#include "robot_tasks/mission_queue_behavior_tree.hpp"
#include "robot_tasks/mission_queue.hpp"
#include "robot_tasks/mission_recovery_behavior_tree.hpp"
#include "robot_tasks/mission_recovery_policy.hpp"
#include "robot_tasks/mission_resource_cleanup_behavior_tree.hpp"
#include "robot_tasks/mission_resource_cleanup.hpp"
#include "robot_tasks/mission_result_behavior_tree.hpp"
#include "robot_tasks/mission_result_workflow.hpp"
#include "robot_tasks/mission_runtime_behavior_tree.hpp"
#include "robot_tasks/mission_safety_workflow.hpp"
#include "robot_tasks/operator_snapshot.hpp"
#include "robot_tasks/payload_workflow.hpp"
#include "robot_tasks/queued_mission_start_behavior_tree.hpp"
#include "robot_tasks/scenario_catalog.hpp"
#include "robot_tasks/station_confirmation_workflow.hpp"
#include "robot_tasks/station_sequence_workflow.hpp"
#include "robot_tasks/station_catalog.hpp"
#include "robot_tasks/station_transport_workflow.hpp"
#include "robot_tasks/submit_order_behavior_tree.hpp"
#include "robot_tasks/submit_order_router.hpp"
#include "robot_tasks/traffic_reservation.hpp"
#include "robot_tasks/workflow_control.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

using NavigateSequence = robot_interfaces::action::NavigateSequence;
using GoalHandleNavigateSequence = rclcpp_action::ClientGoalHandle<NavigateSequence>;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

namespace {

geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

geometry_msgs::msg::PoseStamped PoseStampedFromWaypoint(
    const robot_tasks::MissionWaypoint& waypoint, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id.empty() ? "map" : frame_id;
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.orientation = QuaternionFromYaw(waypoint.yaw);
  return pose;
}

double PathLength(const nav_msgs::msg::Path& path) {
  double distance_m = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto& previous = path.poses[i - 1].pose.position;
    const auto& current = path.poses[i].pose.position;
    distance_m += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return distance_m;
}

std::string SanitizeId(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (const char ch : value) {
    const auto uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) || ch == '_' || ch == '-') {
      result.push_back(ch);
    } else {
      result.push_back('_');
    }
  }
  return result;
}

robot_tasks::MissionActionResultCode ToMissionActionResultCode(
    const rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return robot_tasks::MissionActionResultCode::kSucceeded;
    case rclcpp_action::ResultCode::CANCELED:
      return robot_tasks::MissionActionResultCode::kCanceled;
    default:
      return robot_tasks::MissionActionResultCode::kFailed;
  }
}

}  // namespace

class MissionRunnerNode final : public rclcpp::Node {
 public:
  MissionRunnerNode() : Node("mission_runner_node") {
    const auto default_mission_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "missions" / "demo_patrol.yaml";
    const auto default_dock_mission_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "missions" / "dock_return.yaml";
    const auto default_station_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "stations" / "warehouse_stations.yaml";
    const auto default_facility_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "facilities" / "warehouse_facilities.yaml";
    const auto default_dock_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "docks" / "docks.yaml";
    const auto default_fleet_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "fleet" / "warehouse_fleet.yaml";
    const auto default_scenario_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "scenarios" / "industry_scenarios.yaml";
    const auto default_business_order_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "business_orders" / "business_orders.yaml";
    const auto default_zones_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_navigation")) /
        "config" / "map_zones.yaml";
    mission_file_ = declare_parameter<std::string>("mission_file", default_mission_file.string());
    dock_mission_file_ =
        declare_parameter<std::string>("dock_mission_file", default_dock_mission_file.string());
    station_file_ = declare_parameter<std::string>("station_file", default_station_file.string());
    facility_file_ =
        declare_parameter<std::string>("facility_file", default_facility_file.string());
    dock_file_ = declare_parameter<std::string>("dock_file", default_dock_file.string());
    fleet_file_ = declare_parameter<std::string>("fleet_file", default_fleet_file.string());
    scenario_file_ =
        declare_parameter<std::string>("scenario_file", default_scenario_file.string());
    business_order_file_ = declare_parameter<std::string>(
        "business_order_file", default_business_order_file.string());
    robot_id_ = declare_parameter<std::string>("robot_id", "robot_1");
    preflight_zones_file_ =
        declare_parameter<std::string>("preflight_zones_file", default_zones_file.string());
    preflight_map_name_ = declare_parameter<std::string>("preflight_map_name", "indoor_room");
    action_name_ = declare_parameter<std::string>("action_name", "/navigate_sequence");
    pause_service_name_ = declare_parameter<std::string>("pause_service_name", "/pause_mission");
    resume_service_name_ = declare_parameter<std::string>("resume_service_name", "/resume_mission");
    cancel_service_name_ = declare_parameter<std::string>("cancel_service_name", "/cancel_mission");
    auto_start_queue_ = declare_parameter<bool>("auto_start_queue", true);
    return_to_dock_after_mission_ = declare_parameter<bool>("return_to_dock_after_mission", false);
    return_to_dock_on_low_battery_ = declare_parameter<bool>("return_to_dock_on_low_battery", true);
    autostart_ = declare_parameter<bool>("autostart", false);
    dock_priority_ = declare_parameter<int>("dock_priority", 100);
    server_timeout_ms_ = declare_parameter<int>("server_timeout_ms", 5000);
    dock_low_battery_voltage_ = declare_parameter<double>("dock_low_battery_voltage", 22.0);
    opportunity_charge_threshold_voltage_ =
        declare_parameter<double>("opportunity_charge_threshold_voltage", 23.2);
    critical_charge_threshold_voltage_ =
        declare_parameter<double>("critical_charge_threshold_voltage", dock_low_battery_voltage_);
    opportunity_charge_priority_ = declare_parameter<int>("opportunity_charge_priority", 75);
    opportunity_charge_require_idle_queue_ =
        declare_parameter<bool>("opportunity_charge_require_idle_queue", true);
    estimate_nominal_speed_mps_ = declare_parameter<double>("estimate_nominal_speed_mps", 0.35);
    estimate_battery_drop_per_meter_ =
        declare_parameter<double>("estimate_battery_drop_per_meter", 0.015);
    estimate_min_battery_voltage_ =
        declare_parameter<double>("estimate_min_battery_voltage", dock_low_battery_voltage_);
    estimate_use_nav2_path_ = declare_parameter<bool>("estimate_use_nav2_path", true);
    estimate_nav2_compute_path_action_ =
        declare_parameter<std::string>("estimate_nav2_compute_path_action", "/compute_path_to_pose");
    estimate_nav2_planner_id_ = declare_parameter<std::string>("estimate_nav2_planner_id", "");
    estimate_nav2_server_timeout_ms_ =
        declare_parameter<int>("estimate_nav2_server_timeout_ms", 50);
    estimate_nav2_result_timeout_ms_ =
        declare_parameter<int>("estimate_nav2_result_timeout_ms", 1500);
    failure_recovery_enabled_ = declare_parameter<bool>("failure_recovery_enabled", true);
    failure_recovery_policy_ =
        declare_parameter<std::string>("failure_recovery_policy", "retry_then_dock");
    failure_retry_limit_ = declare_parameter<int>("failure_retry_limit", 1);
    failure_recovery_priority_ = declare_parameter<int>("failure_recovery_priority", 90);
    preflight_enabled_ = declare_parameter<bool>("preflight_enabled", true);
    preflight_require_station_route_ =
        declare_parameter<bool>("preflight_require_station_route", true);
    preflight_default_speed_limit_mps_ =
        declare_parameter<double>("preflight_default_speed_limit_mps", 0.6);
    traffic_intersection_locks_enabled_ =
        declare_parameter<bool>("traffic_intersection_locks_enabled", true);
    payload_capacity_kg_ = declare_parameter<double>("payload_capacity_kg", 30.0);
    top_module_type_ = declare_parameter<std::string>("top_module_type", "manual_load");
    localization_covariance_threshold_ =
        declare_parameter<double>("localization_covariance_threshold", 0.5);
    localization_pause_on_lost_ = declare_parameter<bool>("localization_pause_on_lost", true);
    safety_zone_speed_limit_mps_ =
        declare_parameter<double>("safety_zone_speed_limit_mps", preflight_default_speed_limit_mps_);
    safety_obstacle_pause_on_blockage_ =
        declare_parameter<bool>("safety_obstacle_pause_on_blockage", true);

    action_client_ = rclcpp_action::create_client<NavigateSequence>(this, action_name_);
    nav2_path_node_ = std::make_shared<rclcpp::Node>(
        "mission_runner_nav2_path_estimator",
        rclcpp::NodeOptions().use_global_arguments(false));
    nav2_path_client_ =
        rclcpp_action::create_client<ComputePathToPose>(
            nav2_path_node_, estimate_nav2_compute_path_action_);
    pause_client_ = create_client<std_srvs::srv::Trigger>(pause_service_name_);
    resume_client_ = create_client<std_srvs::srv::Trigger>(resume_service_name_);
    cancel_client_ = create_client<std_srvs::srv::Trigger>(cancel_service_name_);
    state_pub_ = create_publisher<robot_interfaces::msg::RobotState>("/mission_runner/state", 10);
    payload_state_pub_ =
        create_publisher<robot_interfaces::msg::PayloadState>("/payload/state", 10);
    localization_health_pub_ =
        create_publisher<robot_interfaces::msg::LocalizationHealth>("/localization_health", 10);
    safety_state_pub_ =
        create_publisher<robot_interfaces::msg::SafetyState>("/safety_state", 10);
    initial_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    chassis_sub_ = create_subscription<robot_interfaces::msg::ChassisState>(
        "/chassis/state", 10,
        [this](robot_interfaces::msg::ChassisState::SharedPtr msg) { HandleChassisState(*msg); });
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          HandleLocalizationPose(*msg);
        });
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "/start_mission_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          StartMission(response);
        });
    reload_srv_ = create_service<std_srvs::srv::Trigger>(
        "/reload_mission_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          LoadProfile(response);
        });
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
        "/pause_mission_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          PauseMission(response);
        });
    resume_srv_ = create_service<std_srvs::srv::Trigger>(
        "/resume_mission_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ResumeMission(response);
        });
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
        "/cancel_mission_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          CancelMission(response);
        });
    v2_recovery_state_srv_ =
        create_service<robot_interfaces_mission::srv::GetMissionRecoveryState>(
            "/v2/get_mission_recovery_state",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::GetMissionRecoveryState::Request>,
                std::shared_ptr<
                    robot_interfaces_mission::srv::GetMissionRecoveryState::Response>
                    response) { GetMissionRecoveryState(response); });
    v2_list_mission_events_srv_ =
        create_service<robot_interfaces_mission::srv::ListMissionEvents>(
            "/v2/list_mission_events",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::ListMissionEvents::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::ListMissionEvents::Response>
                    response) { ListMissionEvents(request, response); });
    reset_recovery_srv_ = create_service<std_srvs::srv::Trigger>(
        "/reset_mission_recovery_state",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ResetMissionRecoveryState(response);
        });
    v2_enqueue_srv_ = create_service<robot_interfaces_mission::srv::EnqueueMission>(
        "/v2/enqueue_mission_profile",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::EnqueueMission::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::EnqueueMission::Response>
                   response) { EnqueueMission(request, response); });
    v2_cancel_queued_srv_ =
        create_service<robot_interfaces_mission::srv::CancelQueuedMission>(
            "/v2/cancel_queued_mission",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::CancelQueuedMission::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::CancelQueuedMission::Response>
                    response) { CancelQueuedMission(request, response); });
    v2_cancel_workflow_srv_ = create_service<robot_interfaces_business::srv::CancelWorkflow>(
        "/v2/cancel_workflow",
        [this](const std::shared_ptr<robot_interfaces_business::srv::CancelWorkflow::Request>
                   request,
               std::shared_ptr<robot_interfaces_business::srv::CancelWorkflow::Response>
                   response) { CancelWorkflow(request, response); });
    v2_pause_workflow_srv_ = create_service<robot_interfaces_business::srv::PauseWorkflow>(
        "/v2/pause_workflow",
        [this](const std::shared_ptr<robot_interfaces_business::srv::PauseWorkflow::Request>
                   request,
               std::shared_ptr<robot_interfaces_business::srv::PauseWorkflow::Response>
                   response) { PauseWorkflow(request, response); });
    v2_resume_workflow_srv_ = create_service<robot_interfaces_business::srv::ResumeWorkflow>(
        "/v2/resume_workflow",
        [this](const std::shared_ptr<robot_interfaces_business::srv::ResumeWorkflow::Request>
                   request,
               std::shared_ptr<robot_interfaces_business::srv::ResumeWorkflow::Response>
                   response) { ResumeWorkflow(request, response); });
    v2_get_workflow_status_srv_ =
        create_service<robot_interfaces_business::srv::GetWorkflowStatus>(
            "/v2/get_workflow_status",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::GetWorkflowStatus::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::GetWorkflowStatus::Response>
                    response) { GetWorkflowStatus(request, response); });
    v2_reprioritize_queued_mission_srv_ =
        create_service<robot_interfaces_mission::srv::ReprioritizeQueuedMission>(
            "/v2/reprioritize_queued_mission",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::ReprioritizeQueuedMission::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::ReprioritizeQueuedMission::Response>
                    response) { ReprioritizeQueuedMission(request, response); });
    v2_reprioritize_batch_srv_ =
        create_service<robot_interfaces_mission::srv::ReprioritizeBatch>(
            "/v2/reprioritize_batch",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::ReprioritizeBatch::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::ReprioritizeBatch::Response>
                    response) { ReprioritizeBatch(request, response); });
    v2_preempt_srv_ = create_service<robot_interfaces_mission::srv::PreemptMission>(
        "/v2/preempt_mission_profile",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::PreemptMission::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::PreemptMission::Response>
                   response) { PreemptMission(request, response); });
    v2_estimate_mission_srv_ =
        create_service<robot_interfaces_mission::srv::EstimateMissionCost>(
            "/v2/estimate_mission_cost",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::EstimateMissionCost::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::EstimateMissionCost::Response>
                    response) { EstimateMission(request, response); });
    v2_submit_unified_order_srv_ =
        create_service<robot_interfaces_business::srv::SubmitOrder>(
            "/v2/submit_order",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::SubmitOrder::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::SubmitOrder::Response>
                    response) { SubmitOrder(request, response); });
    v2_list_stations_srv_ = create_service<robot_interfaces_mission::srv::ListStations>(
        "/v2/list_stations",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::ListStations::Request>,
               std::shared_ptr<robot_interfaces_mission::srv::ListStations::Response>
                   response) { ListStations(response); });
    v2_list_station_routes_srv_ =
        create_service<robot_interfaces_mission::srv::ListStationRoutes>(
            "/v2/list_station_routes",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::ListStationRoutes::Request>,
                std::shared_ptr<robot_interfaces_mission::srv::ListStationRoutes::Response>
                    response) { ListStationRoutes(response); });
    v2_block_station_route_srv_ =
        create_service<robot_interfaces_mission::srv::BlockStationRoute>(
            "/v2/block_station_route",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::BlockStationRoute::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::BlockStationRoute::Response>
                    response) { BlockStationRoute(request, response); });
    v2_unblock_station_route_srv_ =
        create_service<robot_interfaces_mission::srv::UnblockStationRoute>(
            "/v2/unblock_station_route",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::UnblockStationRoute::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::UnblockStationRoute::Response>
                    response) { UnblockStationRoute(request, response); });
    v2_list_traffic_reservations_srv_ =
        create_service<robot_interfaces_mission::srv::ListTrafficReservations>(
            "/v2/list_traffic_reservations",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::ListTrafficReservations::Request>,
                std::shared_ptr<
                    robot_interfaces_mission::srv::ListTrafficReservations::Response>
                    response) { ListTrafficReservations(response); });
    v2_clear_traffic_reservation_srv_ =
        create_service<robot_interfaces_mission::srv::ClearTrafficReservation>(
            "/v2/clear_traffic_reservation",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::ClearTrafficReservation::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::ClearTrafficReservation::Response>
                    response) { ClearTrafficReservation(request, response); });
    v2_detect_traffic_deadlock_srv_ =
        create_service<robot_interfaces_mission::srv::DetectTrafficDeadlock>(
            "/v2/detect_traffic_deadlock",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::DetectTrafficDeadlock::Request>,
                std::shared_ptr<robot_interfaces_mission::srv::DetectTrafficDeadlock::Response>
                    response) { DetectTrafficDeadlock(response); });
    v2_list_fleet_robots_srv_ =
        create_service<robot_interfaces_fleet::srv::ListFleetRobots>(
            "/v2/list_fleet_robots",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::ListFleetRobots::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::ListFleetRobots::Response>
                    response) { ListFleetRobots(request, response); });
    v2_update_fleet_robot_state_srv_ =
        create_service<robot_interfaces_fleet::srv::UpdateFleetRobotState>(
            "/v2/update_fleet_robot_state",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::UpdateFleetRobotState::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::UpdateFleetRobotState::Response>
                    response) { UpdateFleetRobotState(request, response); });
    v2_update_remote_task_state_srv_ =
        create_service<robot_interfaces_fleet::srv::UpdateRemoteTaskState>(
            "/v2/update_remote_task_state",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::UpdateRemoteTaskState::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::UpdateRemoteTaskState::Response>
                    response) { UpdateRemoteTaskState(request, response); });
    v2_preview_fleet_station_task_srv_ =
        create_service<robot_interfaces_fleet::srv::PreviewFleetStationTask>(
            "/v2/preview_fleet_station_task",
            [this](
                const std::shared_ptr<
                    robot_interfaces_fleet::srv::PreviewFleetStationTask::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::PreviewFleetStationTask::Response>
                    response) { PreviewFleetStationTask(request, response); });
    v2_submit_fleet_station_task_srv_ =
        create_service<robot_interfaces_fleet::srv::SubmitFleetStationTask>(
            "/v2/submit_fleet_station_task",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::SubmitFleetStationTask::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::SubmitFleetStationTask::Response>
                    response) { SubmitFleetStationTask(request, response); });
    v2_list_business_order_types_srv_ =
        create_service<robot_interfaces_business::srv::ListBusinessOrderTypes>(
            "/v2/list_business_order_types",
            [this](
                const std::shared_ptr<
                    robot_interfaces_business::srv::ListBusinessOrderTypes::Request>,
                std::shared_ptr<robot_interfaces_business::srv::ListBusinessOrderTypes::Response>
                    response) { ListBusinessOrderTypes(response); });
    v2_get_operator_snapshot_srv_ =
        create_service<robot_interfaces_business::srv::GetOperatorSnapshot>(
            "/v2/get_operator_snapshot",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::GetOperatorSnapshot::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::GetOperatorSnapshot::Response>
                    response) { GetOperatorSnapshot(request, response); });
    v2_list_scenario_tasks_srv_ =
        create_service<robot_interfaces_business::srv::ListScenarioTasks>(
            "/v2/list_scenario_tasks",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ListScenarioTasks::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ListScenarioTasks::Response>
                    response) { ListScenarioTasks(request, response); });
    v2_list_scenario_workflows_srv_ =
        create_service<robot_interfaces_business::srv::ListScenarioWorkflows>(
            "/v2/list_scenario_workflows",
            [this](
                const std::shared_ptr<
                    robot_interfaces_business::srv::ListScenarioWorkflows::Request> request,
                std::shared_ptr<robot_interfaces_business::srv::ListScenarioWorkflows::Response>
                    response) { ListScenarioWorkflows(request, response); });
    v2_list_facility_resources_srv_ =
        create_service<robot_interfaces_facility::srv::ListFacilityResources>(
            "/v2/list_facility_resources",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ListFacilityResources::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ListFacilityResources::Response>
                    response) { ListFacilityResources(request, response); });
    v2_reserve_facility_resource_srv_ =
        create_service<robot_interfaces_facility::srv::ReserveFacilityResource>(
            "/v2/reserve_facility_resource",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ReserveFacilityResource::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_facility::srv::ReserveFacilityResource::Response>
                    response) { ReserveFacilityResource(request, response); });
    v2_release_facility_resource_srv_ =
        create_service<robot_interfaces_facility::srv::ReleaseFacilityResource>(
            "/v2/release_facility_resource",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ReleaseFacilityResource::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_facility::srv::ReleaseFacilityResource::Response>
                    response) { ReleaseFacilityResource(request, response); });
    v2_request_lift_session_srv_ =
        create_service<robot_interfaces_facility::srv::RequestLiftSession>(
            "/v2/request_lift_session",
            [this](
                const std::shared_ptr<robot_interfaces_facility::srv::RequestLiftSession::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::RequestLiftSession::Response>
                    response) { RequestLiftSession(request, response); });
    v2_release_lift_session_srv_ =
        create_service<robot_interfaces_facility::srv::ReleaseLiftSession>(
            "/v2/release_lift_session",
            [this](
                const std::shared_ptr<robot_interfaces_facility::srv::ReleaseLiftSession::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ReleaseLiftSession::Response>
                    response) { ReleaseLiftSession(request, response); });
    v2_get_lift_state_srv_ = create_service<robot_interfaces_facility::srv::GetLiftState>(
        "/v2/get_lift_state",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::GetLiftState::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::GetLiftState::Response>
                   response) { GetLiftState(request, response); });
    v2_get_door_state_srv_ = create_service<robot_interfaces_facility::srv::GetDoorState>(
        "/v2/get_door_state",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::GetDoorState::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::GetDoorState::Response>
                   response) { GetDoorState(request, response); });
    v2_list_docks_srv_ = create_service<robot_interfaces_mission::srv::ListDocks>(
        "/v2/list_docks",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::ListDocks::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::ListDocks::Response>
                   response) { ListDocks(request, response); });
    v2_request_docking_srv_ = create_service<robot_interfaces_mission::srv::RequestDocking>(
        "/v2/request_docking",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::RequestDocking::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::RequestDocking::Response>
                   response) { RequestDocking(request, response); });
    v2_request_undocking_srv_ =
        create_service<robot_interfaces_mission::srv::RequestUndocking>(
            "/v2/request_undocking",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::RequestUndocking::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::RequestUndocking::Response>
                    response) { RequestUndocking(request, response); });
    v2_get_dock_state_srv_ = create_service<robot_interfaces_mission::srv::GetDockState>(
        "/v2/get_dock_state",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::GetDockState::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::GetDockState::Response>
                   response) { GetDockState(request, response); });
    v2_confirm_load_srv_ = create_service<robot_interfaces_facility::srv::ConfirmLoad>(
        "/v2/confirm_load",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::ConfirmLoad::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::ConfirmLoad::Response>
                   response) { ConfirmLoad(request, response); });
    v2_confirm_unload_srv_ = create_service<robot_interfaces_facility::srv::ConfirmUnload>(
        "/v2/confirm_unload",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::ConfirmUnload::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::ConfirmUnload::Response>
                   response) { ConfirmUnload(request, response); });
    v2_set_payload_loaded_srv_ =
        create_service<robot_interfaces_facility::srv::SetPayloadLoaded>(
            "/v2/set_payload_loaded",
            [this](
                const std::shared_ptr<robot_interfaces_facility::srv::SetPayloadLoaded::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::SetPayloadLoaded::Response>
                    response) { SetPayloadLoaded(request, response); });
    v2_execute_top_module_action_srv_ =
        create_service<robot_interfaces_facility::srv::ExecuteTopModuleAction>(
            "/v2/execute_top_module_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ExecuteTopModuleAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ExecuteTopModuleAction::Response>
                    response) { ExecuteTopModuleAction(request, response); });
    v2_request_relocalization_srv_ =
        create_service<robot_interfaces_navigation::srv::RequestRelocalization>(
            "/v2/request_relocalization",
            [this](
                const std::shared_ptr<robot_interfaces_navigation::srv::RequestRelocalization::Request>
                    request,
                std::shared_ptr<robot_interfaces_navigation::srv::RequestRelocalization::Response>
                    response) { RequestRelocalization(request, response); });
    v2_set_initial_pose_from_station_srv_ =
        create_service<robot_interfaces_navigation::srv::SetInitialPoseFromStation>(
            "/v2/set_initial_pose_from_station",
            [this](
                const std::shared_ptr<robot_interfaces_navigation::srv::SetInitialPoseFromStation::Request>
                    request,
                std::shared_ptr<robot_interfaces_navigation::srv::SetInitialPoseFromStation::Response>
                    response) { SetInitialPoseFromStation(request, response); });
    v2_set_dynamic_speed_limit_srv_ =
        create_service<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>(
            "/v2/set_dynamic_speed_limit",
            [this](const std::shared_ptr<robot_interfaces_navigation::srv::SetDynamicSpeedLimit::Request>
                       request,
                   std::shared_ptr<robot_interfaces_navigation::srv::SetDynamicSpeedLimit::Response>
                       response) { SetDynamicSpeedLimit(request, response); });
    v2_report_obstacle_blockage_srv_ =
        create_service<robot_interfaces_navigation::srv::ReportObstacleBlockage>(
            "/v2/report_obstacle_blockage",
            [this](const std::shared_ptr<robot_interfaces_navigation::srv::ReportObstacleBlockage::Request>
                       request,
                   std::shared_ptr<robot_interfaces_navigation::srv::ReportObstacleBlockage::Response>
                       response) { ReportObstacleBlockage(request, response); });
    v2_clear_obstacle_blockage_srv_ =
        create_service<robot_interfaces_navigation::srv::ClearObstacleBlockage>(
            "/v2/clear_obstacle_blockage",
            [this](const std::shared_ptr<robot_interfaces_navigation::srv::ClearObstacleBlockage::Request>
                       request,
                   std::shared_ptr<robot_interfaces_navigation::srv::ClearObstacleBlockage::Response>
                       response) { ClearObstacleBlockage(request, response); });
    v2_request_charging_srv_ =
        create_service<robot_interfaces_mission::srv::RequestCharging>(
            "/v2/request_charging",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::RequestCharging::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::RequestCharging::Response>
                    response) { RequestCharging(request, response); });
    v2_request_opportunity_charging_srv_ =
        create_service<robot_interfaces_mission::srv::RequestOpportunityCharging>(
            "/v2/request_opportunity_charging",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::RequestOpportunityCharging::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::RequestOpportunityCharging::Response>
                    response) { RequestOpportunityCharging(request, response); });
    v2_execute_facility_action_srv_ =
        create_service<robot_interfaces_facility::srv::ExecuteFacilityAction>(
            "/v2/execute_facility_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ExecuteFacilityAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ExecuteFacilityAction::Response>
                    response) { ExecuteFacilityAction(request, response); });
    v2_request_station_confirmation_srv_ =
        create_service<robot_interfaces_facility::srv::RequestStationConfirmation>(
            "/v2/request_station_confirmation",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::RequestStationConfirmation::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_facility::srv::RequestStationConfirmation::Response>
                    response) { RequestStationConfirmation(request, response); });
    v2_get_pending_confirmations_srv_ =
        create_service<robot_interfaces_facility::srv::GetPendingConfirmations>(
            "/v2/get_pending_confirmations",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::GetPendingConfirmations::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::GetPendingConfirmations::Response>
                    response) { GetPendingConfirmations(request, response); });
    v2_confirm_station_action_srv_ =
        create_service<robot_interfaces_facility::srv::ConfirmStationAction>(
            "/v2/confirm_station_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ConfirmStationAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ConfirmStationAction::Response>
                    response) { ConfirmStationAction(request, response); });
    v2_reject_station_action_srv_ =
        create_service<robot_interfaces_facility::srv::RejectStationAction>(
            "/v2/reject_station_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::RejectStationAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::RejectStationAction::Response>
                    response) { RejectStationAction(request, response); });
    v2_confirm_pick_srv_ =
        create_service<robot_interfaces_business::srv::ConfirmPick>(
            "/v2/confirm_pick",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ConfirmPick::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ConfirmPick::Response>
                    response) { ConfirmPick(request, response); });
    v2_confirm_dropoff_srv_ =
        create_service<robot_interfaces_business::srv::ConfirmDropoff>(
            "/v2/confirm_dropoff",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ConfirmDropoff::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ConfirmDropoff::Response>
                    response) { ConfirmDropoff(request, response); });
    v2_report_short_pick_srv_ =
        create_service<robot_interfaces_business::srv::ReportShortPick>(
            "/v2/report_short_pick",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ReportShortPick::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ReportShortPick::Response>
                    response) { ReportShortPick(request, response); });
    v2_get_order_fulfillment_status_srv_ =
        create_service<robot_interfaces_business::srv::GetOrderFulfillmentStatus>(
            "/v2/get_order_fulfillment_status",
            [this](
                const std::shared_ptr<
                    robot_interfaces_business::srv::GetOrderFulfillmentStatus::Request> request,
                std::shared_ptr<
                    robot_interfaces_business::srv::GetOrderFulfillmentStatus::Response>
                    response) { GetOrderFulfillmentStatus(request, response); });
    v2_submit_station_order_srv_ =
        create_service<robot_interfaces_mission::srv::SubmitStationTransportOrder>(
            "/v2/submit_station_transport_order",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>
                    response) { SubmitStationTransportOrder(request, response); });
    v2_submit_station_order_batch_srv_ =
        create_service<robot_interfaces_mission::srv::SubmitStationOrderBatch>(
            "/v2/submit_station_order_batch",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationOrderBatch::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::SubmitStationOrderBatch::Response>
                    response) { SubmitStationOrderBatch(request, response); });
    v2_cancel_station_order_batch_srv_ =
        create_service<robot_interfaces_mission::srv::CancelStationOrderBatch>(
            "/v2/cancel_station_order_batch",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::CancelStationOrderBatch::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::CancelStationOrderBatch::Response>
                    response) { CancelStationOrderBatch(request, response); });
    v2_submit_station_sequence_srv_ =
        create_service<robot_interfaces_mission::srv::SubmitStationSequenceTask>(
            "/v2/submit_station_sequence_task",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>
                    response) { SubmitStationSequenceTask(request, response); });
    v2_estimate_station_transport_srv_ =
        create_service<robot_interfaces_mission::srv::EstimateStationTransport>(
            "/v2/estimate_station_transport",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::EstimateStationTransport::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::EstimateStationTransport::Response>
                    response) { EstimateStationTransport(request, response); });
    v2_estimate_station_sequence_srv_ =
        create_service<robot_interfaces_mission::srv::EstimateStationSequence>(
            "/v2/estimate_station_sequence",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::EstimateStationSequence::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::EstimateStationSequence::Response>
                    response) { EstimateStationSequence(request, response); });
    return_to_dock_srv_ = create_service<std_srvs::srv::Trigger>(
        "/return_to_dock",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          RequestDockReturn("manual return-to-dock request", response);
        });
    LoadProfile(nullptr);
    LoadStations(std::shared_ptr<robot_interfaces_mission::srv::ListStations::Response>{});
    timer_ = create_wall_timer(200ms, [this]() { Tick(); });
  }

 private:
  using QueuedMission = robot_tasks::QueuedMission;
  using DockRuntimeState = robot_tasks::DockingRuntimeState;
  using StationConfirmation = robot_tasks::StationConfirmationRuntimeState;
  using FulfillmentOrder = robot_tasks::FulfillmentOrderRuntimeState;

  using RemoteTaskState = robot_tasks::RemoteFleetTaskState;

  struct StationTransportExecutionRecord {
    robot_tasks::StationTransportRuntimeState state;
    robot_tasks::MissionProfile pickup_profile;
    robot_tasks::MissionProfile dropoff_profile;
  };

  struct StationTransportDispatchContext {
    std::string mission_id;
    robot_tasks::StationTransportLeg leg = robot_tasks::StationTransportLeg::kNone;
    std::uint64_t dispatch_token = 0;
  };

  bool RunnerBusy() const {
    if (mission_active_) {
      return true;
    }
    const auto it = station_transport_executions_.find(active_station_transport_id_);
    return it != station_transport_executions_.end() &&
           robot_tasks::IsStationTransportBusy(it->second.state.phase);
  }

  std::string ActiveMissionId() const {
    if (!active_station_transport_id_.empty()) {
      return active_station_transport_id_;
    }
    return profile_.has_value() ? profile_->mission_id : "";
  }

  bool HasPendingStationTransport() const {
    return std::any_of(
        station_transport_executions_.begin(), station_transport_executions_.end(),
        [](const auto& item) {
          return !robot_tasks::IsStationTransportTerminal(item.second.state.phase);
        });
  }

  void RequestPendingStationTransportStartAfterPayloadCleared() {
    if (!payload_loaded_ && queue_running_ && HasPendingStationTransport()) {
      queue_start_requested_ = true;
    }
  }

  StationTransportExecutionRecord* FindActiveStationTransport() {
    const auto it = station_transport_executions_.find(active_station_transport_id_);
    return it == station_transport_executions_.end() ? nullptr : &it->second;
  }

  const StationTransportExecutionRecord* FindActiveStationTransport() const {
    const auto it = station_transport_executions_.find(active_station_transport_id_);
    return it == station_transport_executions_.end() ? nullptr : &it->second;
  }

  void LoadProfile(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const auto profile = robot_tasks::LoadMissionProfile(mission_file_);
    if (!profile.has_value()) {
      profile_.reset();
      SetState("ERROR", "failed to load mission profile: " + mission_file_, false);
      if (response != nullptr) {
        response->success = false;
        response->message = last_message_;
      }
      return;
    }
    profile_ = *profile;
    SetState(
        "READY",
        "loaded mission " + profile_->mission_id + " with " +
            std::to_string(profile_->waypoints.size()) + " waypoint(s)",
        true);
    if (response != nullptr) {
      response->success = true;
      response->message = last_message_;
    }
  }

  void StartMission(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (RunnerBusy()) {
      if (response != nullptr) {
        response->success = false;
        response->message = "mission already active";
      }
      return;
    }
    if (!mission_queue_.empty()) {
      queue_running_ = true;
      StartNextQueuedMission(response);
      return;
    }
    DispatchMission(response, "mission profile sent");
  }

  template <typename RequestT, typename ResponseT>
  void EnqueueMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto profile = robot_tasks::LoadMissionProfile(request->mission_file);
    if (!profile.has_value()) {
      response->success = false;
      response->message = "failed to load mission profile: " + request->mission_file;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission",
            MissionIdActiveOrQueued(profile->mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateMissionForQueue(*profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto admission = AdmitQueuedMission(
        QueuedMission{*profile, request->mission_file, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, false, "",
        "queued mission " + profile->mission_id + " priority " +
            std::to_string(request->priority));
    response->success = admission.success;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void CancelQueuedMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto mission_id = SanitizeId(request->mission_id);
    if (mission_id.empty()) {
      response->success = false;
      response->message = "mission_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    robot_tasks::QueuedMissionCancelRequest cancel_request;
    cancel_request.queue = mission_queue_;
    cancel_request.mission_id = mission_id;
    cancel_request.cancel_active = request->cancel_active;
    cancel_request.mission_active = RunnerBusy();
    cancel_request.active_mission_id = ActiveMissionId();
    const auto decision = robot_tasks::PlanQueuedMissionCancel(cancel_request);
    mission_queue_ = decision.remaining_queue;
    for (const auto& released_mission_id : decision.release_mission_ids) {
      ReleaseAllResourcesForMission(released_mission_id);
      station_transport_executions_.erase(released_mission_id);
    }
    if (decision.active_cancel_requested) {
      RequestActiveMissionCancel("active mission cancellation requested by id");
    }

    response->success = decision.success;
    response->message = decision.message;
    response->queue_size = decision.queue_size;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  bool MissionBelongsToWorkflowOrder(
      const std::string& mission_id, const std::string& order_id) const {
    return robot_tasks::MissionBelongsToWorkflowOrder(mission_id, order_id);
  }

  bool MissionBelongsToStationOrderBatch(
      const std::string& mission_id, const std::string& batch_id) const {
    return robot_tasks::MissionBelongsToStationOrderBatch(mission_id, batch_id);
  }

  template <typename RequestT, typename ResponseT>
  void CancelWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto decision = robot_tasks::PlanWorkflowCancel(
        robot_tasks::WorkflowCancelRequest{
            order_id, request->cancel_active, mission_queue_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()}});
    mission_queue_ = decision.remaining_queue;
    response->canceled_mission_ids = decision.canceled_mission_ids;
    response->active_cancel_requested = decision.active_cancel_requested;
    for (const auto& mission_id : response->canceled_mission_ids) {
      if (decision.active_cancel_requested && mission_id == ActiveMissionId()) {
        continue;
      }
      ReleaseAllResourcesForMission(mission_id);
      station_transport_executions_.erase(mission_id);
    }

    if (decision.active_cancel_requested) {
      RequestActiveMissionCancel("workflow cancellation requested for " + order_id);
    }

    response->queue_size = static_cast<int>(mission_queue_.size());
    response->success = decision.success;
    response->message = decision.message;
    if (response->success && !RunnerBusy()) {
      SetState("QUEUED", response->message, true);
    }
  }

  template <typename RequestT, typename ResponseT>
  void PauseWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto decision = robot_tasks::PlanWorkflowPause(
        robot_tasks::WorkflowPauseRequest{
            order_id, request->pause_active, mission_queue_, paused_workflow_orders_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()}});
    paused_workflow_orders_ = decision.paused_order_ids;
    response->affected_mission_ids = decision.affected_mission_ids;
    response->active_pause_requested = decision.active_pause_requested;

    if (decision.active_pause_requested) {
      paused_ = decision.runner_paused;
      SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
      SetState("PAUSED", "workflow pause requested for " + order_id, true);
    }

    if (decision.success) {
      if (decision.clear_queue_start_requested) {
        queue_start_requested_ = false;
      }
      response->success = true;
      response->message = decision.message;
    } else {
      response->success = false;
      response->message = decision.message;
    }
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (response->success && !RunnerBusy()) {
      SetState("WORKFLOW_PAUSED", response->message, true);
    }
  }

  template <typename RequestT, typename ResponseT>
  void ResumeWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto decision = robot_tasks::PlanWorkflowResume(
        robot_tasks::WorkflowResumeRequest{
            order_id, request->resume_active, mission_queue_, paused_workflow_orders_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()}});
    paused_workflow_orders_ = decision.paused_order_ids;
    response->affected_mission_ids = decision.affected_mission_ids;
    response->active_resume_requested = decision.active_resume_requested;

    if (decision.active_resume_requested) {
      paused_ = decision.runner_paused;
      SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
      SetState("RUNNING", "workflow resume requested for " + order_id, true);
    }

    if (decision.success) {
      queue_running_ = queue_running_ || decision.queue_running;
      queue_start_requested_ = queue_start_requested_ || decision.queue_start_requested;
      response->success = true;
      response->message = decision.message;
    } else {
      response->success = false;
      response->message = decision.message;
    }
    response->queue_size = static_cast<int>(mission_queue_.size());
  }

  template <typename RequestT, typename ResponseT>
  void GetWorkflowStatus(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) const {
    const auto order_id = SanitizeId(request->order_id);
    response->order_id = order_id;
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->state = "INVALID";
      return;
    }

    const auto snapshot = robot_tasks::BuildWorkflowStatusSnapshot(
        robot_tasks::WorkflowStatusRequest{
            order_id, mission_queue_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()},
            paused_workflow_orders_, mission_events_, current_state_});
    response->queued_mission_ids = snapshot.queued_mission_ids;
    response->running_mission_id = snapshot.running_mission_id;
    response->finished_mission_ids = snapshot.finished_mission_ids;
    response->paused = snapshot.paused;
    response->total_steps = snapshot.total_steps;
    response->finished_steps = snapshot.finished_steps;
    response->queued_steps = snapshot.queued_steps;
    response->state = snapshot.state;
    response->success = snapshot.success;
    response->message = snapshot.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReprioritizeQueuedMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto mission_id = SanitizeId(request->mission_id);
    if (mission_id.empty()) {
      response->success = false;
      response->message = "mission_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    robot_tasks::QueuedMissionReprioritizeRequest reprioritize_request;
    reprioritize_request.queue = mission_queue_;
    reprioritize_request.mission_id = mission_id;
    reprioritize_request.priority = request->priority;
    reprioritize_request.mission_active = RunnerBusy();
    const auto decision = robot_tasks::PlanQueuedMissionReprioritize(reprioritize_request);
    mission_queue_ = decision.remaining_queue;
    response->success = decision.success;
    response->queue_size = decision.queue_size;
    response->message = decision.message;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void ReprioritizeBatch(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::vector<std::string> mission_ids;
    mission_ids.reserve(request->mission_ids.size());
    for (const auto& mission_id : request->mission_ids) {
      mission_ids.push_back(SanitizeId(mission_id));
    }
    robot_tasks::QueuedMissionBatchReprioritizeRequest reprioritize_request;
    reprioritize_request.queue = mission_queue_;
    reprioritize_request.mission_ids = mission_ids;
    reprioritize_request.priorities = request->priorities;
    reprioritize_request.mission_active = RunnerBusy();
    const auto decision = robot_tasks::PlanQueuedMissionBatchReprioritize(reprioritize_request);
    mission_queue_ = decision.remaining_queue;
    response->updated_mission_ids = decision.updated_mission_ids;
    response->success = decision.success;
    response->queue_size = decision.queue_size;
    response->message = decision.message;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void PreemptMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto profile = robot_tasks::LoadMissionProfile(request->mission_file);
    if (!profile.has_value()) {
      response->success = false;
      response->message = "failed to load mission profile: " + request->mission_file;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission",
            MissionIdActiveOrQueued(profile->mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateMissionForQueue(*profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const bool had_active_mission = RunnerBusy();
    const auto admission = AdmitQueuedMission(
        QueuedMission{*profile, request->mission_file, request->priority,
                      next_queue_sequence_++},
        true, true, "preempting active mission for " + profile->mission_id,
        had_active_mission ? "preempt mission queued and active cancel requested"
                           : "preempt mission queued to start",
        false);
    response->success = admission.success;
    response->mission_id = profile->mission_id;
    response->message = admission.message;
    response->queue_size = admission.queue_size;
  }

  template <typename RequestT, typename ResponseT>
  void EstimateMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::optional<robot_tasks::MissionProfile> profile;
    if (request->mission_file.empty()) {
      if (!profile_.has_value()) {
        LoadProfile(nullptr);
      }
      profile = profile_;
    } else {
      profile = robot_tasks::LoadMissionProfile(request->mission_file);
    }
    if (!profile.has_value()) {
      response->success = false;
      response->message = "failed to load mission profile for estimate";
      return;
    }
    FillEstimateResponse(
        *profile, request->nominal_speed_mps, request->battery_voltage, response.get());
  }

  template <typename ResponseT>
  void RejectSubmitOrder(
      const std::shared_ptr<ResponseT>& response,
      const std::string& message) const {
    response->accepted = false;
    response->reject_reason = message;
    response->message = message;
    response->queue_size = static_cast<int>(mission_queue_.size());
  }

  template <typename ResponseT>
  void FillSubmitOrderResponse(
      const std::shared_ptr<ResponseT>& response,
      const bool success, const std::string& message, const std::string& mission_id,
      const std::vector<std::string>& mission_ids, const int queue_size) const {
    response->accepted = success;
    response->reject_reason = success ? "" : message;
    response->message = message;
    response->mission_id =
        !mission_id.empty() ? mission_id : (mission_ids.empty() ? "" : mission_ids.front());
    response->mission_ids = mission_ids;
    if (response->mission_ids.empty() && !response->mission_id.empty()) {
      response->mission_ids.push_back(response->mission_id);
    }
    response->queue_size = queue_size;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto route_result = robot_tasks::TickSubmitOrderBehaviorTree(
        robot_tasks::SubmitOrderBehaviorTreeInput{robot_tasks::SubmitOrderInput{
            request->order_id, request->order_type, request->priority, request->payload_json,
            request->tags}});
    if (!route_result.success || !route_result.route.has_value()) {
      RejectSubmitOrder(
          response,
          route_result.message.empty() ? "unhandled order_type route" : route_result.message);
      return;
    }
    const auto& route = *route_result.route;

    if (route.kind == robot_tasks::SubmitOrderKind::kTransport) {
      auto typed_request = std::make_shared<robot_interfaces_business::srv::SubmitTransportOrder::Request>();
      typed_request->order_id = route.order_id;
      typed_request->frame_id = route.frame_id;
      typed_request->pickup_x = route.pickup_x;
      typed_request->pickup_y = route.pickup_y;
      typed_request->pickup_yaw = route.pickup_yaw;
      typed_request->dropoff_x = route.dropoff_x;
      typed_request->dropoff_y = route.dropoff_y;
      typed_request->dropoff_yaw = route.dropoff_yaw;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitTransportOrder::Response>();
      SubmitTransportOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kStationTransport) {
      auto typed_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
      typed_request->order_id = route.order_id;
      typed_request->pickup_station = route.pickup_station;
      typed_request->dropoff_station = route.dropoff_station;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
      SubmitStationTransportOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kStationOrderBatch) {
      auto typed_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationOrderBatch::Request>();
      typed_request->batch_id = route.batch_id;
      typed_request->order_ids = route.order_ids;
      typed_request->pickup_station_ids = route.pickup_station_ids;
      typed_request->dropoff_station_ids = route.dropoff_station_ids;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      typed_request->continue_on_error = route.continue_on_error;
      auto typed_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationOrderBatch::Response>();
      SubmitStationOrderBatch(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kStationSequence) {
      auto typed_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>();
      typed_request->order_id = route.order_id;
      typed_request->station_ids = route.station_ids;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>();
      SubmitStationSequenceTask(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kFleetStation) {
      auto typed_request =
          std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Request>();
      typed_request->task_id = route.task_id;
      typed_request->required_capability = route.required_capability;
      typed_request->pickup_station = route.pickup_station;
      typed_request->dropoff_station = route.dropoff_station;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Response>();
      SubmitFleetStationTask(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kBusiness) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitBusinessOrder::Request>();
      typed_request->business_order_id = route.business_order_id;
      typed_request->business_type = route.business_type;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitBusinessOrder::Response>();
      SubmitBusinessOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kScenarioTask) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Request>();
      typed_request->scenario_id = route.scenario_id;
      typed_request->task_id = route.task_id;
      typed_request->order_id = route.order_id;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Response>();
      SubmitScenarioTask(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kScenarioWorkflow) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Request>();
      typed_request->scenario_id = route.scenario_id;
      typed_request->workflow_id = route.workflow_id;
      typed_request->order_id = route.order_id;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Response>();
      SubmitScenarioWorkflow(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kVda5050) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitVda5050Order::Request>();
      typed_request->order_id = route.order_id;
      typed_request->serial_number = route.serial_number;
      typed_request->pickup_node_id = route.pickup_node_id;
      typed_request->dropoff_node_id = route.dropoff_node_id;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitVda5050Order::Response>();
      SubmitVda5050Order(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kPickingWave) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitPickingWave::Request>();
      typed_request->wave_id = route.wave_id;
      typed_request->sku_id = route.sku_id;
      typed_request->tote_id = route.tote_id;
      typed_request->pick_station = route.pick_station;
      typed_request->dropoff_station = route.dropoff_station;
      typed_request->quantity = route.quantity;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitPickingWave::Response>();
      SubmitPickingWave(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kReplenishment) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitReplenishmentOrder::Request>();
      typed_request->order_id = route.order_id;
      typed_request->sku_id = route.sku_id;
      typed_request->tote_id = route.tote_id;
      typed_request->source_station = route.source_station;
      typed_request->line_station = route.line_station;
      typed_request->quantity = route.quantity;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitReplenishmentOrder::Response>();
      SubmitReplenishmentOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kMilkRun) {
      auto typed_request = std::make_shared<robot_interfaces_business::srv::SubmitMilkRun::Request>();
      typed_request->run_id = route.run_id;
      typed_request->tote_id = route.tote_id;
      typed_request->station_ids = route.station_ids;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitMilkRun::Response>();
      SubmitMilkRun(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    RejectSubmitOrder(response, "unhandled order_type route");
  }

  template <typename RequestT, typename ResponseT>
  void SubmitTransportOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto profile = CreateTransportOrderProfile(*request, response);
    if (!profile.has_value()) {
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "transport order", "transport order",
            MissionIdActiveOrQueued(profile->mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateMissionForQueue(*profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "transport order", "transport order", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto admission = AdmitQueuedMission(
        QueuedMission{*profile, "transport_order:" + profile->mission_id, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, false, "",
        "queued transport order " + profile->mission_id + " priority " +
            std::to_string(request->priority));
    response->success = admission.success;
    response->mission_id = profile->mission_id;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename ResponseT>
  void ListStations(const std::shared_ptr<ResponseT>& response) {
    const auto catalog = LoadStations(response);
    if (!catalog.has_value()) {
      return;
    }
    const auto projection = robot_tasks::BuildStationListProjection(*catalog);
    response->success = true;
    response->message = projection.message;
    response->station_ids = projection.station_ids;
    response->frame_ids = projection.frame_ids;
    response->x = projection.x;
    response->y = projection.y;
    response->yaw = projection.yaw;
  }

  template <typename ResponseT>
  void ListStationRoutes(const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return;
    }
    const auto projection = robot_tasks::BuildStationRouteListProjection(*catalog);
    response->success = true;
    response->message = projection.message;
    response->from_station = projection.from_station;
    response->to_station = projection.to_station;
    response->distance_m = projection.distance_m;
    response->bidirectional = projection.bidirectional;
    response->enabled = projection.enabled;
  }

  template <typename RequestT, typename ResponseT>
  void BlockStationRoute(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto from = SanitizeId(request->from_station);
    const auto to = SanitizeId(request->to_station);
    const auto lock_id = RouteLockId(from, to);
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->resource_id = lock_id;
      return;
    }
    const auto reason = SanitizeId(request->reason.empty() ? "manual" : request->reason);
    const auto decision = robot_tasks::PlanTrafficBlock(
        robot_tasks::TrafficBlockRequest{
            from, to, reason,
            robot_tasks::FindStation(*catalog, from) != nullptr &&
                robot_tasks::FindStation(*catalog, to) != nullptr,
            robot_tasks::FindRoutePath(*catalog, from, to).has_value()},
        route_locks_);
    if (decision.mutate_state) {
      route_locks_[decision.resource_id] = decision.owner_id;
    }
    response->success = decision.success;
    response->message = decision.message;
    response->resource_id = decision.resource_id;
  }

  template <typename RequestT, typename ResponseT>
  void UnblockStationRoute(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto from = SanitizeId(request->from_station);
    const auto to = SanitizeId(request->to_station);
    const auto decision = robot_tasks::PlanTrafficUnblock(from, to, route_locks_);
    if (decision.mutate_state) {
      route_locks_.erase(decision.resource_id);
    }
    response->success = decision.success;
    response->message = decision.message;
    response->resource_id = decision.resource_id;
  }

  template <typename ResponseT>
  void ListTrafficReservations(const std::shared_ptr<ResponseT>& response) const {
    response->success = true;
    response->message = "loaded " + std::to_string(route_locks_.size()) + " traffic reservation(s)";
    for (const auto& [resource_id, owner_id] : route_locks_) {
      response->resource_ids.push_back(resource_id);
      response->owner_ids.push_back(owner_id);
      response->reservation_types.push_back(
          robot_tasks::IsTrafficBlockOwner(owner_id) ? "blocked" : "mission");
    }
  }

  template <typename RequestT, typename ResponseT>
  void ClearTrafficReservation(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto decision =
        robot_tasks::PlanTrafficReservationClear(request->resource_id, route_locks_);
    if (decision.erase_resource) {
      route_locks_.erase(decision.resource_id);
    }
    if (!decision.release_mission_id.empty()) {
      ReleaseRouteLocksForMission(decision.release_mission_id);
    }
    response->success = decision.success;
    response->message = decision.message;
    response->owner_id = decision.owner_id;
  }

  template <typename ResponseT>
  void DetectTrafficDeadlock(const std::shared_ptr<ResponseT>& response) const {
    response->success = true;
    const auto decision = robot_tasks::PlanTrafficDeadlockDetection(route_locks_);
    response->deadlocked = decision.deadlocked;
    response->mission_ids = decision.mission_ids;
    response->conflict_resource_ids = decision.conflict_resource_ids;
    response->conflict_station_ids = decision.conflict_station_ids;
    response->descriptions = decision.descriptions;
    response->message = decision.message;
  }

  std::optional<robot_tasks::FleetCatalog> LoadFleet(std::string* message) const {
    auto catalog = robot_tasks::LoadFleetCatalog(fleet_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load fleet catalog: " + fleet_file_;
      }
      return std::nullopt;
    }
    ApplyRuntimeFleetState(*catalog);
    return catalog;
  }

  void ApplyRuntimeFleetState(robot_tasks::FleetCatalog& catalog) const {
    robot_tasks::ApplyRuntimeFleetState(catalog, runtime_fleet_state_);
  }

  std::string JoinIds(const std::vector<std::string>& ids) const {
    std::string result;
    for (const auto& id : ids) {
      if (!result.empty()) {
        result += ",";
      }
      result += id;
    }
    return result;
  }

  template <typename RequestT, typename ResponseT>
  void ListFleetRobots(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFleet(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildFleetRobotListProjection(
        *catalog,
        robot_tasks::FleetRobotListRequest{request->include_disabled, request->capability});
    response->success = projection.success;
    response->robot_ids = projection.robot_ids;
    response->enabled = projection.enabled;
    response->states = projection.states;
    response->battery_voltage = projection.battery_voltage;
    response->current_station_ids = projection.current_station_ids;
    response->capabilities = projection.capabilities;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void UpdateFleetRobotState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto robot_id = SanitizeId(request->robot_id);
    response->robot_id = robot_id;
    auto base_catalog = robot_tasks::LoadFleetCatalog(fleet_file_);
    if (!base_catalog.has_value()) {
      response->success = false;
      response->message = "failed to load fleet catalog: " + fleet_file_;
      return;
    }
    std::optional<robot_tasks::StationCatalog> stations;
    if (request->update_current_station_id) {
      stations = robot_tasks::LoadStationCatalog(station_file_);
      if (!stations.has_value()) {
        response->success = false;
        response->message = "failed to load station catalog: " + station_file_;
        return;
      }
    }

    const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
        robot_tasks::FleetRuntimeUpdateRequest{
            robot_id, request->enabled, SanitizeId(request->state),
            request->battery_voltage, SanitizeId(request->current_station_id),
            request->update_enabled, request->update_state,
            request->update_battery_voltage, request->update_current_station_id},
        *base_catalog, stations.has_value() ? &*stations : nullptr, runtime_fleet_state_);
    response->success = decision.success;
    response->message = decision.message;
    if (!decision.success) {
      return;
    }
    runtime_fleet_state_[robot_id] = decision.runtime_state;
    response->enabled = decision.effective_robot.enabled;
    response->state = decision.effective_robot.state;
    response->battery_voltage = decision.effective_robot.battery_voltage;
    response->current_station_id = decision.effective_robot.current_station_id;
  }

  template <typename RequestT, typename ResponseT>
  void UpdateRemoteTaskState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto task_id = SanitizeId(request->task_id);
    const auto state = SanitizeId(request->state);
    const auto decision = robot_tasks::PlanRemoteFleetTaskStateUpdate(
        robot_tasks::RemoteFleetTaskUpdateRequest{task_id, state, request->message},
        remote_tasks_);
    if (decision.mutate_state) {
      remote_tasks_[task_id] = decision.task;
      RecordMissionEventFor(
          decision.event_mission_id, decision.event_state, decision.event_previous_state,
          decision.event_message, decision.event_recoverable);
    }
    response->success = decision.success;
    response->task_id = task_id;
    response->assigned_robot_id = decision.task.assigned_robot_id;
    response->state = decision.task.state;
    response->message = decision.response_message;
  }

  std::optional<robot_tasks::FleetRobotTaskScore> SelectFleetStationTaskCandidate(
      const std::string& required_capability, const std::string& pickup_station,
      const std::string& dropoff_station, std::string* selected_robot_id,
      std::string* message) const {
    const auto catalog = LoadFleet(message);
    if (!catalog.has_value()) {
      return std::nullopt;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      if (message != nullptr) {
        *message = "failed to load station catalog: " + station_file_;
      }
      return std::nullopt;
    }
    robot_tasks::FleetRobotTaskScore selected_score;
    const auto* selected = robot_tasks::SelectFleetRobotForStationTask(
        *catalog, *stations, required_capability, pickup_station, dropoff_station, {},
        &selected_score);
    if (selected == nullptr) {
      if (message != nullptr) {
        *message = "no available fleet robot with capability: " + required_capability;
      }
      return std::nullopt;
    }
    if (selected_robot_id != nullptr) {
      *selected_robot_id = selected->id;
    }
    if (selected_score.robot_id.empty()) {
      selected_score.robot_id = selected->id;
      selected_score.battery_voltage = selected->battery_voltage;
    }
    return selected_score;
  }

  template <typename RequestT, typename ResponseT>
  void PreviewFleetStationTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto required_capability =
        request->required_capability.empty() ? "station_transport" : request->required_capability;
    std::string message;
    std::string selected_robot_id;
    const auto score = SelectFleetStationTaskCandidate(
        required_capability, request->pickup_station, request->dropoff_station,
        &selected_robot_id, &message);
    if (!score.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    response->success = true;
    response->assigned_robot_id = selected_robot_id;
    response->local_robot = selected_robot_id == robot_id_;
    response->approach_distance_m = score->approach_distance_m;
    response->task_distance_m = score->task_distance_m;
    response->estimated_total_distance_m = score->total_distance_m;
    response->battery_voltage = score->battery_voltage;
    response->message = "preview selected " + selected_robot_id +
                        " estimated_total_distance_m=" +
                        std::to_string(score->total_distance_m);
  }

  template <typename RequestT, typename ResponseT>
  void SubmitFleetStationTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto required_capability =
        request->required_capability.empty() ? "station_transport" : request->required_capability;
    std::string message;
    std::string selected_robot_id;
    const auto selected_score = SelectFleetStationTaskCandidate(
        required_capability, request->pickup_station, request->dropoff_station,
        &selected_robot_id, &message);
    if (!selected_score.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    response->assigned_robot_id = selected_robot_id;
    const auto sanitized_task_id = SanitizeId(request->task_id);
    const auto task_id = sanitized_task_id.empty()
                             ? "fleet_task_" + std::to_string(next_fleet_task_sequence_++)
                             : sanitized_task_id;
    const auto dispatch_decision = robot_tasks::PlanFleetStationTaskDispatch(
        robot_tasks::FleetStationTaskDispatchRequest{
            task_id, selected_robot_id, robot_id_, request->pickup_station,
            request->dropoff_station, current_state_, selected_score->total_distance_m},
        remote_tasks_);
    if (!dispatch_decision.success) {
      response->success = false;
      response->task_id = task_id;
      response->queue_size = static_cast<int>(mission_queue_.size());
      response->message = dispatch_decision.response_message;
      return;
    }
    if (dispatch_decision.dispatch_remote) {
      remote_tasks_[task_id] = dispatch_decision.remote_task;
      RecordMissionEventFor(
          dispatch_decision.event_mission_id, dispatch_decision.event_state,
          dispatch_decision.event_previous_state, dispatch_decision.event_message,
          dispatch_decision.event_recoverable);
      response->success = true;
      response->task_id = task_id;
      response->mission_id = dispatch_decision.mission_id;
      response->queue_size = static_cast<int>(mission_queue_.size());
      response->estimated_total_distance_m = selected_score->total_distance_m;
      response->message = dispatch_decision.response_message;
      return;
    }

    auto station_request =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
    station_request->order_id = dispatch_decision.station_order_id;
    station_request->pickup_station = request->pickup_station;
    station_request->dropoff_station = request->dropoff_station;
    station_request->priority = request->priority;
    station_request->start_if_idle = request->start_if_idle;
    station_request->preempt_current = request->preempt_current;
    auto station_response =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
    SubmitStationTransportOrder(station_request, station_response);

    response->success = station_response->success;
    response->task_id = task_id;
    response->mission_id = station_response->mission_id;
    response->queue_size = station_response->queue_size;
    response->estimated_total_distance_m = selected_score->total_distance_m;
    response->message = station_response->success
                            ? "assigned fleet task " + task_id + " to " + selected_robot_id +
                                  " estimated_total_distance_m=" +
                                  std::to_string(selected_score->total_distance_m) +
                                  ": " + station_response->message
                            : "fleet task rejected by station order chain: " +
                                  station_response->message;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitVda5050Order(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanVda5050StationOrder(
        robot_tasks::Vda5050StationOrderRequest{
            SanitizeId(request->order_id), SanitizeId(request->serial_number),
            SanitizeId(request->pickup_node_id), SanitizeId(request->dropoff_node_id),
            request->priority, request->start_if_idle, request->preempt_current});
    response->order_id = decision.order_id;
    response->serial_number = decision.serial_number;
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      return;
    }

    auto station_request =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
    station_request->order_id = decision.station_order_id;
    station_request->pickup_station = decision.pickup_station;
    station_request->dropoff_station = decision.dropoff_station;
    station_request->priority = decision.priority;
    station_request->start_if_idle = decision.start_if_idle;
    station_request->preempt_current = decision.preempt_current;
    auto station_response =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
    SubmitStationTransportOrder(station_request, station_response);

    response->success = station_response->success;
    response->mission_id = station_response->mission_id;
    response->queue_size = station_response->queue_size;
    response->message = station_response->success
                            ? decision.message + ": " +
                                  station_response->message
                            : "vda5050 order rejected by station order chain: " +
                                  station_response->message;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitBusinessOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto business_order_id = SanitizeId(request->business_order_id);
    const auto business_type = SanitizeId(request->business_type);
    response->queue_size = static_cast<int>(mission_queue_.size());

    std::string message;
    const auto business_catalog = LoadBusinessOrders(&message);
    const auto decision = robot_tasks::PlanBusinessOrderSubmit(
        robot_tasks::BusinessOrderSubmitRequest{
            business_order_id, business_type, request->priority, request->start_if_idle,
            request->preempt_current},
        business_catalog.has_value() ? &*business_catalog : nullptr,
        message);
    response->business_order_id = decision.business_order_id;
    response->business_type = decision.business_type;
    response->routed_service = decision.routed_service;
    response->routed_scenario_id = decision.routed_scenario_id;
    response->routed_workflow_id = decision.routed_workflow_id;
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      return;
    }

    auto workflow_request =
        std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Request>();
    workflow_request->scenario_id = decision.routed_scenario_id;
    workflow_request->workflow_id = decision.routed_workflow_id;
    workflow_request->order_id = decision.workflow_order_id;
    workflow_request->priority = decision.priority;
    workflow_request->start_if_idle = decision.start_if_idle;
    workflow_request->preempt_current = decision.preempt_current;
    auto workflow_response =
        std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Response>();
    SubmitScenarioWorkflow(workflow_request, workflow_response);

    response->success = workflow_response->success;
    response->mission_ids = workflow_response->mission_ids;
    response->queued_count = workflow_response->queued_count;
    response->queue_size = workflow_response->queue_size;
    response->message = workflow_response->success
                            ? decision.message + ": " + workflow_response->message
                            : "business order rejected by scenario workflow chain: " +
                                  workflow_response->message;
  }

  std::optional<robot_tasks::BusinessOrderCatalog> LoadBusinessOrders(
      std::string* message) const {
    const auto catalog = robot_tasks::LoadBusinessOrderCatalog(business_order_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load business order catalog: " + business_order_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  template <typename ResponseT>
  void ListBusinessOrderTypes(const std::shared_ptr<ResponseT>& response) const {
    std::string message;
    const auto catalog = LoadBusinessOrders(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    for (const auto& order_type : catalog->order_types) {
      response->business_types.push_back(order_type.business_type);
      response->scenario_ids.push_back(order_type.scenario_id);
      response->workflow_ids.push_back(order_type.workflow_id);
      response->labels.push_back(order_type.label);
    }
    response->success = true;
    response->message =
        "loaded " + std::to_string(response->business_types.size()) +
        " business order type(s)";
  }

  template <typename RequestT, typename ResponseT>
  void GetOperatorSnapshot(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) const {
    std::string message;
    const auto fleet = LoadFleet(&message);
    const auto facilities = LoadFacilities(&message);
    const auto business_orders = LoadBusinessOrders(&message);
    const auto projection = robot_tasks::BuildOperatorSnapshotProjection(
        robot_tasks::OperatorSnapshotRequest{
            current_state_,
            previous_state_,
            ActiveMissionId(),
            last_message_,
            recoverable_,
            RunnerBusy(),
            paused_,
            mission_queue_,
            paused_workflow_orders_,
            fleet.has_value() ? &*fleet : nullptr,
            &remote_tasks_,
            facilities.has_value() ? &*facilities : nullptr,
            &facility_reservations_,
            business_orders.has_value() ? &*business_orders : nullptr,
            mission_events_,
            request->event_limit});

    response->success = projection.success;
    response->mission_state = projection.mission_state;
    response->previous_mission_state = projection.previous_mission_state;
    response->active_mission_id = projection.active_mission_id;
    response->mission_message = projection.mission_message;
    response->recoverable = projection.recoverable;
    response->mission_active = projection.mission_active;
    response->paused = projection.paused;
    response->queue_size = projection.queue_size;
    response->queued_mission_ids = projection.queued_mission_ids;
    response->queued_priorities = projection.queued_priorities;
    response->paused_workflow_order_ids = projection.paused_workflow_order_ids;
    response->fleet_robot_ids = projection.fleet_robot_ids;
    response->fleet_enabled = projection.fleet_enabled;
    response->fleet_states = projection.fleet_states;
    response->fleet_battery_voltage = projection.fleet_battery_voltage;
    response->fleet_current_station_ids = projection.fleet_current_station_ids;
    response->fleet_capabilities = projection.fleet_capabilities;
    response->remote_task_ids = projection.remote_task_ids;
    response->remote_task_robot_ids = projection.remote_task_robot_ids;
    response->remote_task_states = projection.remote_task_states;
    response->facility_resource_ids = projection.facility_resource_ids;
    response->facility_resource_types = projection.facility_resource_types;
    response->facility_station_ids = projection.facility_station_ids;
    response->facility_available = projection.facility_available;
    response->facility_status = projection.facility_status;
    response->business_types = projection.business_types;
    response->business_scenario_ids = projection.business_scenario_ids;
    response->business_workflow_ids = projection.business_workflow_ids;
    response->recent_event_stamp = projection.recent_event_stamp;
    response->recent_event_mission_ids = projection.recent_event_mission_ids;
    response->recent_event_states = projection.recent_event_states;
    response->recent_event_messages = projection.recent_event_messages;
    response->message = projection.message;
  }

  std::optional<robot_tasks::ScenarioCatalog> LoadScenarios(std::string* message) const {
    const auto catalog = robot_tasks::LoadScenarioCatalog(scenario_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load scenario catalog: " + scenario_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  template <typename RequestT, typename ResponseT>
  void ListScenarioTasks(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto scenario_filter = SanitizeId(request->scenario_id);
    for (const auto& task : catalog->tasks) {
      if (!scenario_filter.empty() && task.scenario_id != scenario_filter) {
        continue;
      }
      response->scenario_ids.push_back(task.scenario_id);
      response->task_ids.push_back(task.task_id);
      response->labels.push_back(task.label);
      response->task_types.push_back(task.task_type.empty() ? "station_transport" : task.task_type);
      response->pickup_station_ids.push_back(task.pickup_station_id);
      response->dropoff_station_ids.push_back(task.dropoff_station_id);
      response->station_sequences_csv.push_back(JoinIds(task.station_sequence_ids));
      response->required_capabilities.push_back(task.required_capability);
      response->resource_ids.push_back(task.resource_id);
      response->resource_types.push_back(task.resource_type);
      response->actions.push_back(task.action);
      response->hold_after_actions.push_back(task.hold_after_action);
      response->default_priorities.push_back(task.default_priority);
    }
    response->success = true;
    response->message =
        "loaded " + std::to_string(response->task_ids.size()) + " scenario task(s)";
  }

  template <typename RequestT, typename ResponseT>
  void SubmitScenarioTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    const auto scenario_id = SanitizeId(request->scenario_id);
    const auto task_id = SanitizeId(request->task_id);
    const auto decision = robot_tasks::PlanScenarioTaskSubmit(
        robot_tasks::ScenarioTaskSubmitRequest{
            scenario_id, task_id, SanitizeId(request->order_id), request->priority,
            request->start_if_idle, request->preempt_current},
        catalog.has_value() ? &*catalog : nullptr,
        message);
    response->scenario_id = decision.scenario_id;
    response->task_id = decision.task_id;
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      return;
    }

    if (decision.dispatch_kind == robot_tasks::ScenarioTaskDispatchKind::kFacilityAction) {
      auto facility_request =
          std::make_shared<robot_interfaces_facility::srv::ExecuteFacilityAction::Request>();
      facility_request->request_id = decision.order_id;
      facility_request->resource_id = decision.resource_id;
      facility_request->resource_type = decision.resource_type;
      facility_request->action = decision.action;
      facility_request->priority = decision.priority;
      facility_request->start_if_idle = decision.start_if_idle;
      facility_request->preempt_current = decision.preempt_current;
      facility_request->hold_after_action = decision.hold_after_action;
      auto facility_response =
          std::make_shared<robot_interfaces_facility::srv::ExecuteFacilityAction::Response>();
      ExecuteFacilityAction(facility_request, facility_response);

      response->success = facility_response->success;
      response->mission_id = facility_response->mission_id;
      response->queue_size = facility_response->queue_size;
      response->message = facility_response->success
                              ? "queued scenario facility task " + decision.scenario_id + "/" +
                                    decision.task_id + ": " + facility_response->message
                              : "scenario facility task rejected by facility action chain: " +
                                    facility_response->message;
      return;
    }

    if (decision.dispatch_kind == robot_tasks::ScenarioTaskDispatchKind::kStationSequence) {
      auto sequence_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>();
      sequence_request->order_id = decision.order_id;
      sequence_request->station_ids = decision.station_ids;
      sequence_request->priority = decision.priority;
      sequence_request->start_if_idle = decision.start_if_idle;
      sequence_request->preempt_current = decision.preempt_current;
      auto sequence_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>();
      SubmitStationSequenceTask(sequence_request, sequence_response);

      response->success = sequence_response->success;
      response->mission_id =
          sequence_response->mission_ids.empty() ? "" : sequence_response->mission_ids.front();
      response->queue_size = sequence_response->queue_size;
      response->message = sequence_response->success
                              ? "queued scenario station sequence task " + decision.scenario_id +
                                    "/" + decision.task_id + ": " + sequence_response->message
                              : "scenario station sequence task rejected by sequence chain: " +
                                    sequence_response->message;
      return;
    }

    auto fleet_request =
        std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Request>();
    fleet_request->task_id = decision.order_id;
    fleet_request->required_capability = decision.required_capability;
    fleet_request->pickup_station = decision.pickup_station;
    fleet_request->dropoff_station = decision.dropoff_station;
    fleet_request->priority = decision.priority;
    fleet_request->start_if_idle = decision.start_if_idle;
    fleet_request->preempt_current = decision.preempt_current;
    auto fleet_response =
        std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Response>();
    SubmitFleetStationTask(fleet_request, fleet_response);

    response->success = fleet_response->success;
    response->mission_id = fleet_response->mission_id;
    response->assigned_robot_id = fleet_response->assigned_robot_id;
    response->queue_size = fleet_response->queue_size;
    response->message = fleet_response->success
                            ? "queued scenario task " + decision.scenario_id + "/" +
                                  decision.task_id + ": " + fleet_response->message
                            : "scenario task rejected by fleet task chain: " +
                                  fleet_response->message;
  }

  template <typename RequestT, typename ResponseT>
  void ListScenarioWorkflows(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto scenario_filter = SanitizeId(request->scenario_id);
    for (const auto& workflow : catalog->workflows) {
      if (!scenario_filter.empty() && workflow.scenario_id != scenario_filter) {
        continue;
      }
      response->scenario_ids.push_back(workflow.scenario_id);
      response->workflow_ids.push_back(workflow.workflow_id);
      response->labels.push_back(workflow.label);
      response->task_ids_csv.push_back(JoinIds(workflow.task_ids));
      response->default_priorities.push_back(workflow.default_priority);
    }
    response->success = true;
    response->message =
        "loaded " + std::to_string(response->workflow_ids.size()) + " scenario workflow(s)";
  }

  template <typename RequestT, typename ResponseT>
  void SubmitScenarioWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    const auto scenario_id = SanitizeId(request->scenario_id);
    const auto workflow_id = SanitizeId(request->workflow_id);
    const auto decision = robot_tasks::PlanScenarioWorkflowSubmit(
        robot_tasks::ScenarioWorkflowSubmitRequest{
            scenario_id, workflow_id, SanitizeId(request->order_id), request->priority,
            request->start_if_idle, request->preempt_current},
        catalog.has_value() ? &*catalog : nullptr,
        message);
    response->scenario_id = decision.scenario_id;
    response->workflow_id = decision.workflow_id;
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    for (std::size_t index = 0; index < decision.steps.size(); ++index) {
      const auto& step = decision.steps[index];
      auto task_request = std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Request>();
      task_request->scenario_id = step.scenario_id;
      task_request->task_id = step.task_id;
      task_request->order_id = step.order_id;
      task_request->priority = step.priority;
      task_request->start_if_idle = step.start_if_idle;
      task_request->preempt_current = step.preempt_current;
      auto task_response =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Response>();
      SubmitScenarioTask(task_request, task_response);
      if (!task_response->success) {
        response->success = false;
        response->queued_count = static_cast<int>(response->mission_ids.size());
        response->queue_size = task_response->queue_size;
        response->message = "scenario workflow rejected at step " +
                            std::to_string(static_cast<int>(index) + 1) + ": " +
                            task_response->message;
        return;
      }
      response->mission_ids.push_back(task_response->mission_id);
      response->queue_size = task_response->queue_size;
    }
    response->success = true;
    response->queued_count = static_cast<int>(response->mission_ids.size());
    response->queue_size = static_cast<int>(mission_queue_.size());
    response->message = "queued scenario workflow " + decision.scenario_id + "/" +
                        decision.workflow_id +
                        " with " + std::to_string(response->queued_count) + " task(s)";
  }

  std::optional<robot_tasks::FacilityCatalog> LoadFacilities(std::string* message) const {
    const auto catalog = robot_tasks::LoadFacilityCatalog(facility_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load facility catalog: " + facility_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  bool ResourceAvailable(const robot_tasks::FacilityResource& resource) const {
    return robot_tasks::FacilityResourceAvailable(resource, facility_reservations_);
  }

  std::string ResourceStatus(const robot_tasks::FacilityResource& resource) const {
    return robot_tasks::FacilityResourceStatus(resource, facility_reservations_);
  }

  template <typename RequestT, typename ResponseT>
  void ListFacilityResources(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildFacilityResourceListProjection(
        *catalog, facility_reservations_,
        robot_tasks::FacilityResourceListRequest{
            request->include_disabled, request->resource_type});
    response->success = projection.success;
    response->resource_ids = projection.resource_ids;
    response->resource_types = projection.resource_types;
    response->station_ids = projection.station_ids;
    response->enabled = projection.enabled;
    response->available = projection.available;
    response->holder_ids = projection.holder_ids;
    response->status = projection.status;
    response->message = projection.message;
  }

  void ReleaseResourcesForHolder(const std::string& holder_id) {
    robot_tasks::ReleaseResourcesForHolder(facility_reservations_, holder_id);
  }

  template <typename RequestT, typename ResponseT>
  void ReserveFacilityResource(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto* resource = robot_tasks::FindFacilityResource(*catalog, request->resource_id);
    if (resource == nullptr) {
      response->success = false;
      response->message = "unknown facility resource: " + request->resource_id;
      return;
    }
    const auto holder_id =
        SanitizeId(request->holder_id.empty() ? request->resource_id : request->holder_id);
    const auto result = robot_tasks::ReserveFacilityResource(
        facility_reservations_, *resource, holder_id, SanitizeId(request->mission_id),
        request->release_existing_for_holder);
    response->success = result.success;
    response->resource_id = result.resource_id;
    response->holder_id = result.holder_id;
    response->status = result.status;
    response->message = result.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReleaseFacilityResource(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto result = robot_tasks::ReleaseFacilityResource(
        facility_reservations_, request->resource_id, SanitizeId(request->holder_id));
    response->success = result.success;
    response->resource_id = result.resource_id;
    response->status = result.status;
    response->message = result.message;
  }

  const robot_tasks::FacilityResource* FindFacilityByType(
      const robot_tasks::FacilityCatalog& catalog, const std::string& resource_id,
      const std::string& resource_type, std::string* message) const {
    const auto* resource = robot_tasks::FindFacilityResource(catalog, resource_id);
    if (resource == nullptr) {
      if (message != nullptr) {
        *message = "unknown " + resource_type + " resource: " + resource_id;
      }
      return nullptr;
    }
    if (resource->type != resource_type) {
      if (message != nullptr) {
        *message = resource_id + " is not a " + resource_type;
      }
      return nullptr;
    }
    return resource;
  }

  template <typename ResponseT>
  void FillLiftStateResponse(
      const robot_tasks::FacilityResource& resource,
      const std::shared_ptr<ResponseT>& response) const {
    const auto projection = robot_tasks::BuildFacilityResourceStateProjection(
        resource, facility_reservations_, "lift");
    response->success = true;
    response->lift_id = projection.resource_id;
    response->station_id = projection.station_id;
    response->available = projection.available;
    response->holder_id = projection.holder_id;
    response->status = projection.status;
    response->message = projection.message;
  }

  template <typename ResponseT>
  void FillDoorStateResponse(
      const robot_tasks::FacilityResource& resource,
      const std::shared_ptr<ResponseT>& response) const {
    const auto projection = robot_tasks::BuildFacilityResourceStateProjection(
        resource, facility_reservations_, "door");
    response->success = true;
    response->door_id = projection.resource_id;
    response->station_id = projection.station_id;
    response->available = projection.available;
    response->holder_id = projection.holder_id;
    response->status = projection.status;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void RequestLiftSession(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    const auto lift_id = SanitizeId(request->lift_id);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    const auto* lift = FindFacilityByType(*catalog, lift_id, "elevator", &message);
    if (lift == nullptr) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    const auto session_id =
        robot_tasks::BuildLiftSessionId(SanitizeId(request->session_id), lift->id);
    const auto holder_id =
        robot_tasks::BuildLiftSessionHolderId(SanitizeId(request->requester_id), session_id);
    const auto result = robot_tasks::ReserveLiftSessionResource(
        facility_reservations_, *lift, session_id, holder_id,
        request->release_existing_for_requester);
    response->success = result.success;
    response->session_id = result.session_id;
    response->lift_id = result.lift_id;
    response->status = result.status;
    response->message = result.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReleaseLiftSession(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto lift_id = SanitizeId(request->lift_id);
    const auto session_id = SanitizeId(request->session_id);
    response->lift_id = lift_id;
    response->session_id = session_id;
    const auto result =
        robot_tasks::ReleaseLiftSessionResource(facility_reservations_, lift_id, session_id);
    response->success = result.success;
    response->status = result.status;
    response->message = result.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetLiftState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    const auto lift_id = SanitizeId(request->lift_id);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    const auto* lift = FindFacilityByType(*catalog, lift_id, "elevator", &message);
    if (lift == nullptr) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    FillLiftStateResponse(*lift, response);
  }

  template <typename RequestT, typename ResponseT>
  void GetDoorState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    const auto door_id = SanitizeId(request->door_id);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      response->door_id = door_id;
      return;
    }
    const auto* door = FindFacilityByType(*catalog, door_id, "door", &message);
    if (door == nullptr) {
      response->success = false;
      response->message = message;
      response->door_id = door_id;
      return;
    }
    FillDoorStateResponse(*door, response);
  }

  std::optional<robot_tasks::DockCatalog> LoadDocks(std::string* message) const {
    const auto catalog = robot_tasks::LoadDockCatalog(dock_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load dock catalog: " + dock_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  const robot_tasks::Dock* ResolveDock(
      const robot_tasks::DockCatalog& catalog, const std::string& requested_dock_id,
      std::string* message) const {
    if (!requested_dock_id.empty()) {
      const auto* dock = robot_tasks::FindDock(catalog, requested_dock_id);
      if (dock == nullptr && message != nullptr) {
        *message = "unknown dock: " + requested_dock_id;
      }
      return dock;
    }
    const auto* dock = robot_tasks::FindFirstEnabledDock(catalog);
    if (dock == nullptr && message != nullptr) {
      *message = "no enabled dock";
    }
    return dock;
  }

  template <typename RequestT, typename ResponseT>
  void ListDocks(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadDocks(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildDockListProjection(
        *catalog, dock_states_, facility_reservations_, request->include_disabled);
    response->dock_ids = projection.dock_ids;
    response->station_ids = projection.station_ids;
    response->approach_station_ids = projection.approach_station_ids;
    response->charger_resource_ids = projection.charger_resource_ids;
    response->enabled = projection.enabled;
    response->available = projection.available;
    response->states = projection.states;
    response->mission_ids = projection.mission_ids;
    response->success = true;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetDockState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadDocks(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto* dock = ResolveDock(*catalog, SanitizeId(request->dock_id), &message);
    if (dock == nullptr) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildDockStateProjection(
        *dock, dock_states_, facility_reservations_);
    response->dock_id = projection.dock_id;
    response->station_id = projection.station_id;
    response->approach_station_id = projection.approach_station_id;
    response->charger_resource_id = projection.charger_resource_id;
    response->enabled = projection.enabled;
    response->available = projection.available;
    response->state = projection.state;
    response->mission_id = projection.mission_id;
    response->success = true;
    response->message = "dock " + dock->id + " state=" + response->state;
  }

  bool ValidateDockReferences(
      const robot_tasks::Dock& dock, const robot_tasks::FacilityCatalog& facilities,
      const robot_tasks::StationCatalog& stations, std::string* message) const {
    const auto* charger = robot_tasks::FindFacilityResource(facilities, dock.charger_resource_id);
    if (charger == nullptr || charger->type != "charger") {
      if (message != nullptr) {
        *message = "dock charger resource not found or not charger: " + dock.charger_resource_id;
      }
      return false;
    }
    if (robot_tasks::FindStation(stations, dock.approach_station_id) == nullptr) {
      if (message != nullptr) {
        *message = "dock approach station not found: " + dock.approach_station_id;
      }
      return false;
    }
    return true;
  }

  template <typename RequestT, typename ResponseT>
  void RequestDocking(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto docks = LoadDocks(&message);
    const auto facilities = LoadFacilities(&message);
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!docks.has_value() || !facilities.has_value() || !stations.has_value()) {
      response->success = false;
      response->message = docks.has_value() && facilities.has_value()
                              ? "failed to load station catalog: " + station_file_
                              : message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* dock = ResolveDock(*docks, SanitizeId(request->dock_id), &message);
    if (dock == nullptr) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    response->dock_id = dock->id;
    const auto dock_projection = robot_tasks::BuildDockStateProjection(
        *dock, dock_states_, facility_reservations_);
    const bool dock_references_valid = ValidateDockReferences(*dock, *facilities, *stations, &message);
    const auto dock_gate = robot_tasks::PlanDockingRequestGate(
        robot_tasks::DockingRequestGateRequest{
            dock->id, dock_projection.enabled, dock_projection.available, dock_projection.state,
            dock_references_valid, message});
    if (!dock_gate.accepted) {
      response->success = false;
      response->state = dock_gate.state;
      response->message = dock_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto request_id = SanitizeId(request->request_id);
    const std::string mission_id =
        request_id.empty() ? "docking_request_" + dock->id + "_" +
                                 std::to_string(next_docking_request_sequence_++)
                           : "docking_request_" + request_id;
    if (MissionIdActiveOrQueued(mission_id)) {
      const auto duplicate_gate = robot_tasks::PlanDockingRequestGate(
          robot_tasks::DockingRequestGateRequest{
              dock->id, true, true, dock_projection.state, true, "", mission_id, true});
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->state = duplicate_gate.state;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* approach_station = robot_tasks::FindStation(*stations, dock->approach_station_id);
    auto profile = robot_tasks::BuildStationWaypointProfile(mission_id, *approach_station);
    const auto preflight = ValidateMissionForQueue(profile);
    const auto preflight_gate = robot_tasks::PlanDockingRequestGate(
        robot_tasks::DockingRequestGateRequest{
            dock->id, true, true, dock_projection.state, true, "", mission_id, false, preflight});
    if (!preflight_gate.accepted) {
      response->success = false;
      response->mission_id = preflight_gate.mission_id;
      response->state = preflight_gate.state;
      response->message = preflight_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    facility_reservations_[dock->charger_resource_id] =
        robot_tasks::ResourceReservation{mission_id, mission_id, "APPROACHING", true};
    facility_resource_by_mission_[mission_id] = dock->charger_resource_id;
    dock_states_[dock->id] =
        DockRuntimeState{dock->id, mission_id, "APPROACHING", request->simulate_contact_success, 0};
    dock_by_mission_[mission_id] = dock->id;
    const auto admission = AdmitQueuedMission(
        QueuedMission{profile, "docking:" + dock->id, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for docking " + mission_id,
        "queued docking request " + mission_id + " for dock " + dock->id);
    response->success = admission.success;
    response->mission_id = mission_id;
    response->state = "APPROACHING";
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void RequestUndocking(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadDocks(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto* dock = ResolveDock(*catalog, SanitizeId(request->dock_id), &message);
    if (dock == nullptr) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanUndocking(
        robot_tasks::UndockingRequest{
            dock->id, dock->charger_resource_id, SanitizeId(request->request_id),
            request->release_charger},
        dock_states_, facility_reservations_);
    robot_tasks::ApplyUndockingDecision(
        decision, dock->charger_resource_id, dock_states_, facility_reservations_,
        facility_resource_by_mission_);
    RecordMissionEventFor(
        decision.mission_id, decision.state, decision.previous_state, decision.message, true);
    response->success = decision.success;
    response->dock_id = dock->id;
    response->state = decision.state;
    response->message = decision.message;
  }

  robot_tasks::PayloadWorkflowState CurrentPayloadWorkflowState() const {
    robot_tasks::PayloadWorkflowState state;
    state.loaded = payload_loaded_;
    state.payload_id = payload_id_;
    state.state = payload_state_;
    state.last_action = payload_last_action_;
    state.message = payload_message_;
    state.weight_kg = payload_weight_kg_;
    state.capacity_kg = payload_capacity_kg_;
    state.top_module_type = top_module_type_;
    return state;
  }

  void ApplyPayloadWorkflowDecision(
      const robot_tasks::PayloadWorkflowDecision& decision) {
    if (!decision.mutate_state) {
      return;
    }
    payload_loaded_ = decision.loaded;
    payload_id_ = decision.payload_id;
    payload_weight_kg_ = decision.weight_kg;
    payload_state_ = decision.state;
    payload_last_action_ = decision.last_action;
    payload_message_ = decision.message;
  }

  void RecordPayloadWorkflowEvent(
      const robot_tasks::PayloadWorkflowDecision& decision) {
    if (decision.event_mission_id.empty()) {
      return;
    }
    RecordMissionEventFor(
        decision.event_mission_id, decision.event_state,
        decision.event_previous_state, decision.event_message,
        decision.event_recoverable);
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmLoad(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto requested_payload_id = SanitizeId(request->payload_id);
    const auto station_id = SanitizeId(request->station_id);

    auto* transport = FindActiveStationTransport();
    if (transport != nullptr) {
      if (paused_) {
        response->success = false;
        response->message =
            "station transport is paused; resume it before confirming load";
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        response->weight_kg = payload_weight_kg_;
        return;
      }
      const auto transport_decision = robot_tasks::PlanStationTransportConfirmLoad(
          transport->state, station_id, requested_payload_id);
      if (!transport_decision.accepted) {
        PublishPayloadState();
        response->success = false;
        response->message = transport_decision.message;
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        response->weight_kg = payload_weight_kg_;
        return;
      }

      const auto payload_decision = robot_tasks::PlanConfirmLoad(
          CurrentPayloadWorkflowState(), requested_payload_id, station_id,
          request->weight_kg);
      if (!payload_decision.success) {
        PublishPayloadState();
        response->success = false;
        response->message = payload_decision.message;
        response->payload_id = payload_decision.payload_id;
        response->state = payload_decision.state;
        response->loaded = payload_decision.loaded;
        response->weight_kg = payload_decision.weight_kg;
        return;
      }

      ApplyPayloadWorkflowDecision(payload_decision);
      RecordPayloadWorkflowEvent(payload_decision);
      transport->state = transport_decision.next_state;
      PublishPayloadState();

      std::string dispatch_message;
      const bool dispatched = DispatchStationTransportLeg(
          transport->state.mission_id, robot_tasks::StationTransportLeg::kDropoff,
          &dispatch_message);
      response->success = dispatched;
      response->message =
          payload_decision.message + "; " +
          (dispatched ? dispatch_message : "dropoff dispatch failed: " + dispatch_message);
      response->payload_id = payload_id_;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      response->weight_kg = payload_weight_kg_;
      return;
    }

    const auto payload_id = requested_payload_id.empty() ? "payload" : requested_payload_id;
    const auto decision = robot_tasks::PlanConfirmLoad(
        CurrentPayloadWorkflowState(), payload_id, station_id, request->weight_kg);
    if (!decision.success) {
      PublishPayloadState();
      response->success = false;
      response->message = decision.message;
      response->payload_id = decision.payload_id;
      response->state = decision.state;
      response->loaded = decision.loaded;
      response->weight_kg = decision.weight_kg;
      return;
    }
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    response->success = true;
    response->message = decision.message;
    response->payload_id = decision.payload_id;
    response->state = decision.state;
    response->loaded = decision.loaded;
    response->weight_kg = decision.weight_kg;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmUnload(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto requested_payload_id = SanitizeId(request->payload_id);
    const auto station_id = SanitizeId(request->station_id);

    auto* transport = FindActiveStationTransport();
    if (transport != nullptr) {
      if (paused_) {
        response->success = false;
        response->message =
            "station transport is paused; resume it before confirming unload";
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        return;
      }
      const auto transport_decision = robot_tasks::PlanStationTransportConfirmUnload(
          transport->state, station_id, requested_payload_id);
      if (!transport_decision.accepted) {
        response->success = false;
        response->message = transport_decision.message;
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        return;
      }

      const auto payload_decision = robot_tasks::PlanConfirmUnload(
          CurrentPayloadWorkflowState(), requested_payload_id, station_id);
      if (!payload_decision.success) {
        response->success = false;
        response->message = payload_decision.message;
        response->payload_id = payload_decision.payload_id;
        response->state = payload_decision.state;
        response->loaded = payload_decision.loaded;
        return;
      }

      const std::string completed_mission_id = transport->state.mission_id;
      ApplyPayloadWorkflowDecision(payload_decision);
      RecordPayloadWorkflowEvent(payload_decision);
      transport->state = transport_decision.next_state;
      PublishPayloadState();
      CompleteStationTransportSuccess(completed_mission_id, transport_decision.message);

      response->success = true;
      response->message = payload_decision.message + "; station transport completed";
      response->payload_id = requested_payload_id;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      return;
    }

    const auto decision = robot_tasks::PlanConfirmUnload(
        CurrentPayloadWorkflowState(), requested_payload_id, station_id);
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      response->payload_id = decision.payload_id;
      response->state = decision.state;
      response->loaded = decision.loaded;
      return;
    }
    const auto response_payload_id =
        decision.mutate_state && !decision.event_mission_id.empty()
            ? decision.event_mission_id.substr(std::string("payload_").size())
            : decision.payload_id;
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    RequestPendingStationTransportStartAfterPayloadCleared();
    response->success = true;
    response->message = decision.message;
    response->payload_id = response_payload_id;
    response->state = decision.state;
    response->loaded = decision.loaded;
  }

  template <typename RequestT, typename ResponseT>
  void SetPayloadLoaded(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto* transport = FindActiveStationTransport();
    if (transport != nullptr &&
        transport->state.phase == robot_tasks::StationTransportPhase::kNeedsOperator &&
        !request->loaded) {
      const auto decision = robot_tasks::PlanSetPayloadLoaded(
          CurrentPayloadWorkflowState(), false, "", 0.0, request->message);
      if (!decision.success) {
        response->success = false;
        response->message = decision.message;
        response->payload_id = decision.payload_id;
        response->state = decision.state;
        response->loaded = decision.loaded;
        response->weight_kg = decision.weight_kg;
        return;
      }

      const std::string mission_id = transport->state.mission_id;
      ApplyPayloadWorkflowDecision(decision);
      RecordPayloadWorkflowEvent(decision);
      transport->state.payload_loaded = false;
      transport->state.payload_id.clear();
      transport->state.phase = robot_tasks::StationTransportPhase::kCanceled;
      PublishPayloadState();
      ReleaseAllResourcesForMission(mission_id);
      FinishStationTransportTerminal(
          mission_id, "CANCELED",
          "operator cleared payload after station transport failure", false);

      response->success = true;
      response->message = "payload cleared and failed station transport canceled";
      response->payload_id = payload_id_;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      response->weight_kg = payload_weight_kg_;
      return;
    }

    if (transport != nullptr ||
        (request->loaded && HasPendingStationTransport())) {
      response->success = false;
      response->message =
          "set_payload_loaded cannot bypass an active or queued station transport";
      response->payload_id = payload_id_;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      response->weight_kg = payload_weight_kg_;
      return;
    }
    const auto payload_id = SanitizeId(request->payload_id.empty() ? "payload" : request->payload_id);
    const auto decision = robot_tasks::PlanSetPayloadLoaded(
        CurrentPayloadWorkflowState(), request->loaded, payload_id,
        request->weight_kg, request->message);
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      response->payload_id = decision.payload_id;
      response->state = decision.state;
      response->loaded = decision.loaded;
      response->weight_kg = decision.weight_kg;
      return;
    }
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    RequestPendingStationTransportStartAfterPayloadCleared();
    response->success = true;
    response->message = decision.message;
    response->payload_id = decision.payload_id;
    response->state = decision.state;
    response->loaded = decision.loaded;
    response->weight_kg = decision.weight_kg;
  }

  template <typename RequestT, typename ResponseT>
  void ExecuteTopModuleAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto action = SanitizeId(request->action.empty() ? "inspect" : request->action);
    const auto decision =
        robot_tasks::PlanTopModuleAction(CurrentPayloadWorkflowState(), action);
    if (!decision.success) {
      response->success = false;
      response->action = action;
      response->state = decision.state;
      response->loaded = decision.loaded;
      response->payload_id = decision.payload_id;
      response->message = decision.message;
      return;
    }
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    response->success = true;
    response->message = decision.message;
    response->action = action;
    response->state = decision.state;
    response->loaded = decision.loaded;
    response->payload_id = decision.payload_id;
  }

  robot_tasks::MissionLocalizationState CurrentLocalizationState() const {
    robot_tasks::MissionLocalizationState state;
    state.localized = localization_localized_;
    state.state = localization_state_;
    state.station_id = localization_station_id_;
    state.message = localization_message_;
    state.covariance_xy = localization_covariance_xy_;
    state.paused_mission = localization_paused_mission_;
    return state;
  }

  void ApplyLocalizationDecision(
      const robot_tasks::MissionLocalizationDecision& decision) {
    localization_localized_ = decision.localization.localized;
    localization_state_ = decision.localization.state;
    localization_station_id_ = decision.localization.station_id;
    localization_message_ = decision.localization.message;
    localization_covariance_xy_ = decision.localization.covariance_xy;
    localization_paused_mission_ = decision.localization.paused_mission;

    if (decision.event.emit) {
      RecordMissionEventFor(
          decision.event.mission_id, decision.event.state, decision.event.previous_state,
          decision.event.message, decision.event.recoverable);
    }
    if (decision.side_effects.request_pause) {
      paused_ = true;
      SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
    }
    if (decision.side_effects.request_resume) {
      paused_ = false;
      SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
    }
    if (decision.side_effects.set_runner_state) {
      SetState(
          decision.side_effects.runner_state, decision.side_effects.runner_message,
          decision.side_effects.runner_recoverable);
    }
  }

  void HandleLocalizationPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    const auto decision = robot_tasks::PlanLocalizationPoseUpdate(
        robot_tasks::MissionLocalizationPoseUpdateRequest{
            CurrentLocalizationState(), pose.pose.covariance[0], pose.pose.covariance[7],
            localization_covariance_threshold_, localization_pause_on_lost_, mission_active_,
            paused_});
    ApplyLocalizationDecision(decision);
    PublishLocalizationHealth();
  }

  template <typename RequestT, typename ResponseT>
  void RequestRelocalization(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanRelocalizationRequest(
        robot_tasks::MissionRelocalizationRequest{CurrentLocalizationState(), request->reason});
    ApplyLocalizationDecision(decision);
    PublishLocalizationHealth();
    response->success = true;
    response->message = localization_message_;
    response->state = localization_state_;
    response->localized = localization_localized_;
  }

  template <typename RequestT, typename ResponseT>
  void SetInitialPoseFromStation(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto station_id = SanitizeId(request->station_id);
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->station_id = station_id;
      response->state = localization_state_;
      response->localized = localization_localized_;
      return;
    }
    const auto* station = robot_tasks::FindStation(*stations, station_id);
    if (station == nullptr) {
      response->success = false;
      response->message = "unknown station: " + station_id;
      response->station_id = station_id;
      response->state = localization_state_;
      response->localized = localization_localized_;
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = station->frame_id.empty() ? "map" : station->frame_id;
    pose.pose.pose.position.x = station->x;
    pose.pose.pose.position.y = station->y;
    pose.pose.pose.orientation = QuaternionFromYaw(station->yaw);
    pose.pose.covariance[0] = 0.05;
    pose.pose.covariance[7] = 0.05;
    pose.pose.covariance[35] = 0.05;
    initial_pose_pub_->publish(pose);
    const auto decision = robot_tasks::PlanInitialPoseRecovery(
        robot_tasks::MissionInitialPoseRecoveryRequest{
            CurrentLocalizationState(), station->id, mission_active_, paused_, 0.05});
    ApplyLocalizationDecision(decision);
    PublishLocalizationHealth();
    response->success = true;
    response->message = localization_message_;
    response->station_id = station->id;
    response->state = localization_state_;
    response->localized = localization_localized_;
  }

  robot_tasks::MissionSafetyState CurrentMissionSafetyState() const {
    robot_tasks::MissionSafetyState state;
    state.obstacle_blocked = safety_obstacle_blocked_;
    state.blockage_paused_mission = safety_blockage_paused_mission_;
    state.runtime_speed_limit_mps = safety_runtime_speed_limit_mps_;
    state.zone_speed_limit_mps = safety_zone_speed_limit_mps_;
    state.stop_reason = safety_stop_reason_;
    state.blockage_id = safety_blockage_id_;
    state.mission_id = safety_mission_id_;
    state.message = safety_message_;
    return state;
  }

  double EffectiveSafetySpeedLimit() const {
    return robot_tasks::EffectiveSafetySpeedLimit(CurrentMissionSafetyState());
  }

  std::string SafetyStateName() const {
    return robot_tasks::MissionSafetyStateName(CurrentMissionSafetyState());
  }

  void ApplyMissionSafetyDecision(
      const robot_tasks::MissionSafetyDecision& decision) {
    if (decision.mutate_state) {
      safety_obstacle_blocked_ = decision.safety.obstacle_blocked;
      safety_blockage_paused_mission_ = decision.safety.blockage_paused_mission;
      safety_runtime_speed_limit_mps_ = decision.safety.runtime_speed_limit_mps;
      safety_zone_speed_limit_mps_ = decision.safety.zone_speed_limit_mps;
      safety_stop_reason_ = decision.safety.stop_reason;
      safety_blockage_id_ = decision.safety.blockage_id;
      safety_mission_id_ = decision.safety.mission_id;
      safety_message_ = decision.safety.message;
    }
    if (decision.event.emit) {
      RecordMissionEventFor(
          decision.event.mission_id, decision.event.state, decision.event.previous_state,
          decision.event.message, decision.event.recoverable);
    }
    if (decision.side_effects.request_pause) {
      paused_ = true;
      SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
    }
    if (decision.side_effects.request_resume) {
      paused_ = false;
      SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
    }
    if (decision.side_effects.set_runner_state) {
      SetState(
          decision.side_effects.runner_state, decision.side_effects.runner_message,
          decision.side_effects.runner_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void SetDynamicSpeedLimit(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanSetDynamicSpeedLimit(
        robot_tasks::MissionSafetySpeedLimitRequest{
            CurrentMissionSafetyState(), request->speed_limit_mps, request->reason});
    ApplyMissionSafetyDecision(decision);
    if (decision.mutate_state) {
      PublishSafetyState();
    }

    response->success = decision.success;
    response->message = decision.response_message;
    response->runtime_speed_limit_mps = decision.response_runtime_speed_limit_mps;
    response->effective_speed_limit_mps = decision.response_effective_speed_limit_mps;
  }

  template <typename RequestT, typename ResponseT>
  void ReportObstacleBlockage(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto blockage_id = SanitizeId(request->blockage_id.empty() ? "obstacle" : request->blockage_id);
    const auto decision = robot_tasks::PlanReportObstacleBlockage(
        robot_tasks::MissionSafetyReportBlockageRequest{
            CurrentMissionSafetyState(), blockage_id, request->reason,
            profile_.has_value() ? profile_->mission_id : "", previous_state_,
            request->pause_active, safety_obstacle_pause_on_blockage_, mission_active_, paused_});
    ApplyMissionSafetyDecision(decision);
    if (decision.mutate_state) {
      PublishSafetyState();
    }

    response->success = decision.success;
    response->message = decision.response_message;
    response->state = decision.response_state;
    response->blockage_id = decision.response_blockage_id;
  }

  template <typename RequestT, typename ResponseT>
  void ClearObstacleBlockage(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanClearObstacleBlockage(
        robot_tasks::MissionSafetyClearBlockageRequest{
            CurrentMissionSafetyState(), SanitizeId(request->blockage_id),
            request->resolution, current_state_, request->resume_active, mission_active_,
            paused_});
    ApplyMissionSafetyDecision(decision);
    if (decision.mutate_state) {
      PublishSafetyState();
    }

    response->success = decision.success;
    response->message = decision.response_message;
    response->state = decision.response_state;
  }

  template <typename RequestT, typename ResponseT>
  void ExecuteFacilityAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto facilities = LoadFacilities(&message);
    if (!facilities.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto requested_resource_id = SanitizeId(request->resource_id);
    const auto requested_resource_type = SanitizeId(request->resource_type);
    const auto action = SanitizeId(request->action.empty() ? "use" : request->action);
    const auto selection = robot_tasks::SelectFacilityActionResource(
        *facilities, facility_reservations_, requested_resource_id, requested_resource_type);
    const auto* resource = selection.resource;
    if (resource == nullptr) {
      response->success = false;
      response->message = selection.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    response->resource_id = resource->id;
    response->action = action;
    if (!robot_tasks::FacilityActionSupported(resource->type, action)) {
      response->success = false;
      response->status = "rejected";
      response->message = "unsupported facility action " + action + " for " + resource->type;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* station = robot_tasks::FindStation(*stations, resource->station_id);
    if (station == nullptr) {
      response->success = false;
      response->message = "facility station not found: " + resource->station_id;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto request_id = SanitizeId(request->request_id);
    const auto mission_id =
        robot_tasks::BuildFacilityActionMissionId(request_id, resource->id, action);
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            mission_id, "facility action", "facility action",
            MissionIdActiveOrQueued(mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->status = "duplicate";
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    auto profile = robot_tasks::BuildStationWaypointProfile(mission_id, *station);
    const auto preflight = ValidateMissionForQueue(profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            mission_id, "facility action", "facility action", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->status = "rejected";
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    robot_tasks::ReserveFacilityActionResource(
        facility_reservations_, facility_resource_by_mission_, *resource, mission_id,
        request->hold_after_action);
    const auto admission = AdmitQueuedMission(
        QueuedMission{
            profile, "facility_action:" + resource->id + ":" + action, request->priority,
            next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for facility action " + mission_id,
        "queued facility action " + action + " for " + resource->id + " at " + station->id);
    response->success = admission.success;
    response->mission_id = mission_id;
    response->status = "reserved";
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void RequestStationConfirmation(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto station_id = SanitizeId(request->station_id);
    const auto action = SanitizeId(request->action.empty() ? "confirm" : request->action);
    auto confirmation_id = SanitizeId(request->confirmation_id);
    if (confirmation_id.empty()) {
      confirmation_id = "station_confirm_" + station_id + "_" + action;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    const auto station_known =
        stations.has_value() && robot_tasks::FindStation(*stations, station_id) != nullptr;
    robot_tasks::StationConfirmationRequest workflow_request;
    workflow_request.confirmation_id = confirmation_id;
    workflow_request.station_id = station_id;
    workflow_request.action = action;
    workflow_request.mission_id = SanitizeId(request->mission_id);
    workflow_request.operator_hint = request->operator_hint;
    workflow_request.timeout_sec = request->timeout_sec;
    workflow_request.timeout_policy =
        SanitizeId(request->timeout_policy.empty() ? "manual" : request->timeout_policy);
    workflow_request.created_at_sec = now().seconds();
    workflow_request.station_known = station_known;
    const auto decision = robot_tasks::PlanStationConfirmationRequest(
        workflow_request, station_confirmations_);
    if (decision.mutate_state) {
      station_confirmations_[decision.confirmation_id] = decision.confirmation;
      RecordMissionEventFor(
          decision.event_mission_id, decision.event_state, decision.event_previous_state,
          decision.event_message, decision.event_recoverable);
    }
    response->success = decision.success;
    response->confirmation_id = decision.confirmation_id;
    response->state = decision.state;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetPendingConfirmations(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response)
      const {
    const auto station_filter = SanitizeId(request->station_id_filter);
    const auto projection = robot_tasks::BuildPendingStationConfirmationsProjection(
        station_confirmations_, station_filter);
    response->confirmation_ids = projection.confirmation_ids;
    response->station_ids = projection.station_ids;
    response->actions = projection.actions;
    response->mission_ids = projection.mission_ids;
    response->operator_hints = projection.operator_hints;
    response->states = projection.states;
    response->success = projection.success;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmStationAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    ResolveStationConfirmation(
        SanitizeId(request->confirmation_id), "CONFIRMED",
        SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id),
        request->note.empty() ? "station action confirmed" : request->note, response);
  }

  template <typename RequestT, typename ResponseT>
  void RejectStationAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    ResolveStationConfirmation(
        SanitizeId(request->confirmation_id), "REJECTED",
        SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id),
        request->reason.empty() ? "station action rejected" : request->reason, response);
  }

  template <typename ResponseT>
  void ResolveStationConfirmation(
      const std::string& confirmation_id, const std::string& state,
      const std::string& operator_id, const std::string& note,
      const std::shared_ptr<ResponseT>& response) {
    const auto decision = ResolveStationConfirmationForRunner(
        confirmation_id, state, operator_id, note);
    response->success = decision.success;
    response->confirmation_id = decision.confirmation_id;
    response->state = decision.state;
    response->message = decision.message;
  }

  void ExpireStationConfirmations() {
    const auto current_time = now().seconds();
    for (auto& [confirmation_id, confirmation] : station_confirmations_) {
      if (confirmation.state != "PENDING" || confirmation.timeout_sec <= 0) {
        continue;
      }
      if (current_time - confirmation.created_at_sec < confirmation.timeout_sec) {
        continue;
      }
      ApplyStationConfirmationTimeout(confirmation_id, confirmation);
    }
  }

  void ApplyStationConfirmationTimeout(
      const std::string& confirmation_id, StationConfirmation& confirmation) {
    confirmation.confirmation_id = confirmation_id;
    const auto decision =
        robot_tasks::PlanStationConfirmationTimeout(confirmation, now().seconds());
    if (!decision.timed_out) {
      return;
    }
    if (decision.update_created_at) {
      confirmation.created_at_sec = decision.new_created_at_sec;
    }
    if (decision.update_state) {
      confirmation.state = decision.state;
    }
    RecordMissionEventFor(
        decision.event_mission_id, decision.event_state, decision.event_previous_state,
        decision.event_message, decision.event_recoverable);
    if (decision.request_dock_return) {
      RequestDockReturn("station confirmation timeout " + confirmation_id, nullptr);
    }
    if (decision.set_runner_state) {
      SetState(
          decision.runner_state, decision.runner_message, decision.runner_recoverable);
    }
  }

  void AddStationConfirmation(
      const std::string& confirmation_id, const std::string& station_id,
      const std::string& action, const std::string& mission_id,
      const std::string& operator_hint) {
    robot_tasks::StationConfirmationRequest workflow_request;
    workflow_request.confirmation_id = confirmation_id;
    workflow_request.station_id = station_id;
    workflow_request.action = action;
    workflow_request.mission_id = mission_id;
    workflow_request.operator_hint = operator_hint;
    workflow_request.created_at_sec = now().seconds();
    workflow_request.station_known = true;
    const auto decision = robot_tasks::PlanStationConfirmationRequest(
        workflow_request, station_confirmations_);
    if (!decision.mutate_state) {
      return;
    }
    station_confirmations_[decision.confirmation_id] = decision.confirmation;
    RecordMissionEventFor(
        decision.event_mission_id, decision.event_state, decision.event_previous_state,
        decision.event_message, decision.event_recoverable);
  }

  bool ResolveFulfillmentConfirmation(
      const std::string& confirmation_id, const std::string& state,
      const std::string& operator_id, const std::string& note) {
    const auto decision = ResolveStationConfirmationForRunner(
        confirmation_id, state, operator_id, note);
    return decision.success;
  }

  robot_tasks::StationConfirmationResolveDecision ResolveStationConfirmationForRunner(
      const std::string& confirmation_id, const std::string& state,
      const std::string& operator_id, const std::string& note) {
    robot_tasks::StationConfirmationResolveRequest request;
    request.confirmation_id = confirmation_id;
    request.state = state;
    request.operator_id = operator_id;
    request.note = note;
    const auto decision =
        robot_tasks::PlanStationConfirmationResolve(request, station_confirmations_);
    if (decision.mutate_state) {
      station_confirmations_[decision.confirmation_id].state = decision.state;
      RecordMissionEventFor(
          decision.event_mission_id, decision.event_state, decision.event_previous_state,
          decision.event_message, decision.event_recoverable);
    }
    return decision;
  }

  std::optional<std::vector<std::string>> QueueFulfillmentStationSequence(
      const std::string& fulfillment_id, const std::vector<std::string>& station_ids,
      const int priority, const bool start_if_idle, std::string* message,
      int* queue_size) {
    if (station_ids.size() < 2U) {
      *message = "fulfillment station sequence requires at least two stations";
      *queue_size = static_cast<int>(mission_queue_.size());
      return std::nullopt;
    }
    auto sequence_request =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>();
    sequence_request->order_id = fulfillment_id;
    sequence_request->station_ids = station_ids;
    sequence_request->priority = priority;
    sequence_request->start_if_idle = start_if_idle;
    sequence_request->preempt_current = false;
    auto sequence_response =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>();
    SubmitStationSequenceTask(sequence_request, sequence_response);
    *message = sequence_response->message;
    *queue_size = sequence_response->queue_size;
    if (!sequence_response->success) {
      return std::nullopt;
    }
    return sequence_response->mission_ids;
  }

  bool FulfillmentExists(const std::string& fulfillment_id) const {
    return fulfillment_orders_.find(fulfillment_id) != fulfillment_orders_.end();
  }

  robot_tasks::FulfillmentSubmitDecision PlanFulfillmentSubmitForRunner(
      const std::string& fulfillment_id, const std::string& order_type,
      const std::size_t station_count, const bool require_two_stations) const {
    robot_tasks::FulfillmentSubmitRequest request;
    request.fulfillment_id = fulfillment_id;
    request.order_type = order_type;
    request.station_count = station_count;
    request.duplicate = FulfillmentExists(fulfillment_id);
    request.require_two_stations = require_two_stations;
    return robot_tasks::PlanFulfillmentSubmit(request);
  }

  template <typename ResponseT>
  bool ApplyFulfillmentSubmitDecision(
      const robot_tasks::FulfillmentSubmitDecision& decision, ResponseT* response) const {
    if (decision.success) {
      return true;
    }
    response->success = false;
    response->fulfillment_id = decision.fulfillment_id;
    response->state = decision.state;
    response->queue_size = static_cast<int>(mission_queue_.size());
    response->message = decision.message;
    return false;
  }

  FulfillmentOrder* FindFulfillment(
      const std::string& fulfillment_id, std::string* message) {
    const auto it = fulfillment_orders_.find(fulfillment_id);
    if (it == fulfillment_orders_.end()) {
      *message = "unknown fulfillment order: " + fulfillment_id;
      return nullptr;
    }
    return &it->second;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitPickingWave(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto wave_id = SanitizeId(request->wave_id);
    const auto fulfillment_id = "picking_wave_" + wave_id;
    SubmitFulfillmentOrder(
        fulfillment_id, "picking_wave", SanitizeId(request->sku_id), SanitizeId(request->tote_id),
        SanitizeId(request->pick_station), SanitizeId(request->dropoff_station),
        std::max(1, request->quantity), request->priority, request->start_if_idle,
        response.get());
  }

  template <typename RequestT, typename ResponseT>
  void SubmitReplenishmentOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    const auto fulfillment_id = "replenishment_" + order_id;
    SubmitFulfillmentOrder(
        fulfillment_id, "replenishment", SanitizeId(request->sku_id),
        SanitizeId(request->tote_id), SanitizeId(request->source_station),
        SanitizeId(request->line_station), std::max(1, request->quantity), request->priority,
        request->start_if_idle, response.get());
  }

  template <typename RequestT, typename ResponseT>
  void SubmitMilkRun(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto run_id = SanitizeId(request->run_id);
    const auto fulfillment_id = "milk_run_" + run_id;
    std::vector<std::string> station_ids;
    station_ids.reserve(request->station_ids.size());
    for (const auto& station_id : request->station_ids) {
      station_ids.push_back(SanitizeId(station_id));
    }
    const auto submit_decision = PlanFulfillmentSubmitForRunner(
        fulfillment_id, "milk_run", station_ids.size(), true);
    if (!ApplyFulfillmentSubmitDecision(submit_decision, response.get())) {
      return;
    }
    std::string message;
    int queue_size = static_cast<int>(mission_queue_.size());
    const auto mission_ids = QueueFulfillmentStationSequence(
        fulfillment_id, station_ids, request->priority, request->start_if_idle, &message,
        &queue_size);
    response->queue_size = queue_size;
    response->fulfillment_id = fulfillment_id;
    if (!mission_ids.has_value()) {
      response->success = false;
      response->state = "REJECTED";
      response->message = message;
      return;
    }
    FulfillmentOrder order;
    order.fulfillment_id = fulfillment_id;
    order.order_type = "milk_run";
    order.tote_id = SanitizeId(request->tote_id);
    order.state = "ROUTE_QUEUED";
    order.pick_station = station_ids.front();
    order.dropoff_station = station_ids.back();
    order.mission_ids = *mission_ids;
    for (const auto& station_id : station_ids) {
      const auto confirmation_id = fulfillment_id + "_" + station_id + "_stop";
      order.confirmation_ids.push_back(confirmation_id);
      AddStationConfirmation(
          confirmation_id, station_id, "milk_run_stop", fulfillment_id,
          "confirm milk run stop for " + order.tote_id);
    }
    fulfillment_orders_[fulfillment_id] = order;
    RecordMissionEventFor(
        fulfillment_id, "FULFILLMENT_ROUTE_QUEUED", "READY",
        "milk run queued with " + std::to_string(station_ids.size()) + " station(s)", true);
    response->success = true;
    response->state = order.state;
    response->mission_ids = order.mission_ids;
    response->message = "milk run queued: " + fulfillment_id;
  }

  template <typename ResponseT>
  void SubmitFulfillmentOrder(
      const std::string& fulfillment_id, const std::string& order_type,
      const std::string& sku_id, const std::string& tote_id,
      const std::string& pick_station, const std::string& dropoff_station,
      const int quantity, const int priority, const bool start_if_idle, ResponseT* response) {
    const auto submit_decision = PlanFulfillmentSubmitForRunner(
        fulfillment_id, order_type, 2, false);
    if (!ApplyFulfillmentSubmitDecision(submit_decision, response)) {
      return;
    }
    if (pick_station.empty() || dropoff_station.empty()) {
      response->success = false;
      response->fulfillment_id = fulfillment_id;
      response->state = "REJECTED";
      response->queue_size = static_cast<int>(mission_queue_.size());
      response->message = "pick station and dropoff station are required";
      return;
    }
    std::string message;
    int queue_size = static_cast<int>(mission_queue_.size());
    const auto mission_ids = QueueFulfillmentStationSequence(
        fulfillment_id, {pick_station, dropoff_station}, priority, start_if_idle, &message,
        &queue_size);
    response->queue_size = queue_size;
    response->fulfillment_id = fulfillment_id;
    if (!mission_ids.has_value()) {
      response->success = false;
      response->state = "REJECTED";
      response->message = message;
      return;
    }
    FulfillmentOrder order;
    order.fulfillment_id = fulfillment_id;
    order.order_type = order_type;
    order.sku_id = sku_id;
    order.tote_id = tote_id;
    order.state = "AWAITING_PICK";
    order.pick_station = pick_station;
    order.dropoff_station = dropoff_station;
    order.requested_quantity = quantity;
    order.mission_ids = *mission_ids;
    order.confirmation_ids = {fulfillment_id + "_pick", fulfillment_id + "_dropoff"};
    AddStationConfirmation(
        order.confirmation_ids[0], pick_station, "pick_" + sku_id, fulfillment_id,
        "pick " + std::to_string(quantity) + " " + sku_id + " into " + tote_id);
    AddStationConfirmation(
        order.confirmation_ids[1], dropoff_station, "dropoff_" + sku_id, fulfillment_id,
        "drop off " + tote_id + " at " + dropoff_station);
    fulfillment_orders_[fulfillment_id] = order;
    RecordMissionEventFor(
        fulfillment_id, "FULFILLMENT_ROUTE_QUEUED", "READY",
        order_type + " queued from " + pick_station + " to " + dropoff_station, true);
    response->success = true;
    response->state = order.state;
    response->mission_ids = order.mission_ids;
    response->message = order_type + " queued: " + fulfillment_id;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmPick(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->state = "UNKNOWN";
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanFulfillmentPick(
        robot_tasks::FulfillmentPickRequest{
            fulfillment_id, order->order_type, order->state, order->requested_quantity,
            request->picked_quantity});
    if (!decision.success) {
      response->success = false;
      response->state = decision.state;
      response->message = decision.message;
      return;
    }
    order->picked_quantity = decision.picked_quantity;
    order->short_quantity = decision.short_quantity;
    order->state = decision.state;
    if (decision.resolve_pick_confirmation && !order->confirmation_ids.empty()) {
      ResolveFulfillmentConfirmation(
          order->confirmation_ids[0], "CONFIRMED",
          SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id),
          request->note.empty() ? "pick confirmed" : request->note);
    }
    RecordMissionEventFor(
        fulfillment_id, decision.event_state, decision.event_previous_state,
        decision.event_message, decision.event_recoverable);
    response->success = true;
    response->state = decision.state;
    response->picked_quantity = order->picked_quantity;
    response->short_quantity = order->short_quantity;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmDropoff(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->state = "UNKNOWN";
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanFulfillmentDropoff(
        robot_tasks::FulfillmentDropoffRequest{
            fulfillment_id, order->order_type, order->state, order->short_quantity,
            order->confirmation_ids.size()});
    if (!decision.success) {
      response->success = false;
      response->state = decision.state;
      response->message = decision.message;
      return;
    }
    const auto operator_id = SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id);
    const auto note = request->note.empty() ? "dropoff confirmed" : request->note;
    if (decision.resolve_dropoff_confirmation) {
      ResolveFulfillmentConfirmation(order->confirmation_ids[1], "CONFIRMED", operator_id, note);
    } else if (decision.resolve_all_confirmations) {
      for (const auto& confirmation_id : order->confirmation_ids) {
        ResolveFulfillmentConfirmation(confirmation_id, "CONFIRMED", operator_id, note);
      }
    }
    order->state = decision.state;
    RecordMissionEventFor(
        fulfillment_id, decision.event_state, decision.event_previous_state, note,
        decision.event_recoverable);
    response->success = true;
    response->state = decision.state;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReportShortPick(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->state = "UNKNOWN";
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanFulfillmentShortPick(
        robot_tasks::FulfillmentShortPickRequest{
            fulfillment_id, order->order_type, order->state, order->requested_quantity,
            request->short_quantity});
    if (!decision.success) {
      response->success = false;
      response->state = decision.state;
      response->message = decision.message;
      return;
    }
    order->short_quantity = decision.short_quantity;
    order->picked_quantity = decision.picked_quantity;
    order->state = decision.state;
    const auto operator_id = SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id);
    RecordMissionEventFor(
        fulfillment_id, decision.event_state, decision.event_previous_state,
        "short pick " + std::to_string(order->short_quantity) + " reported by " +
            operator_id + ": " + request->reason,
        decision.event_recoverable);
    response->success = true;
    response->state = decision.state;
    response->short_quantity = order->short_quantity;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetOrderFulfillmentStatus(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    const auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildFulfillmentStatusProjection(*order);
    response->success = projection.success;
    response->message = projection.message;
    response->order_type = projection.order_type;
    response->sku_id = projection.sku_id;
    response->tote_id = projection.tote_id;
    response->state = projection.state;
    response->pick_station = projection.pick_station;
    response->dropoff_station = projection.dropoff_station;
    response->requested_quantity = projection.requested_quantity;
    response->picked_quantity = projection.picked_quantity;
    response->short_quantity = projection.short_quantity;
    response->mission_ids = projection.mission_ids;
    response->confirmation_ids = projection.confirmation_ids;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitStationTransportOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto profile = CreateStationTransportOrderProfile(*request, response);
    if (!profile.has_value()) {
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    if (MissionIdActiveOrQueued(profile->mission_id)) {
      const auto duplicate_gate = robot_tasks::PlanStationTransportSubmitGate(
          robot_tasks::StationTransportSubmitGateRequest{profile->mission_id, true});
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      const auto gate = robot_tasks::PlanStationTransportSubmitGate(
          robot_tasks::StationTransportSubmitGateRequest{
              profile->mission_id, false, false});
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateStationMissionForQueue(
        *profile, *catalog, request->pickup_station, request->dropoff_station);
    const auto route_path =
        robot_tasks::FindRoutePath(*catalog, request->pickup_station, request->dropoff_station);
    const auto gate = robot_tasks::PlanStationTransportSubmitGate(
        robot_tasks::StationTransportSubmitGateRequest{
            profile->mission_id, false, true, preflight,
            route_path, traffic_intersection_locks_enabled_});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    if (payload_loaded_) {
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message =
          "station transport pickup requires an empty robot payload";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    std::string station_pair_message;
    const auto stations = robot_tasks::ResolveStationPair(
        *catalog, request->pickup_station, request->dropoff_station,
        &station_pair_message);
    if (!stations.has_value()) {
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message = station_pair_message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    std::string route_lock_message;
    if (!ReserveRouteLocksForMission(
            profile->mission_id, gate.route_lock_ids, &route_lock_message)) {
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message = "station transport route lock rejected: " + route_lock_message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    StationTransportExecutionRecord execution;
    execution.state.mission_id = profile->mission_id;
    execution.state.pickup_station_id = stations->pickup.id;
    execution.state.dropoff_station_id = stations->dropoff.id;
    execution.pickup_profile =
        robot_tasks::BuildStationWaypointProfile(profile->mission_id, stations->pickup);
    execution.dropoff_profile =
        robot_tasks::BuildStationWaypointProfile(profile->mission_id, stations->dropoff);
    station_transport_executions_[profile->mission_id] = execution;

    const auto admission = AdmitQueuedMission(
        QueuedMission{
            execution.pickup_profile, "station_transport_order:" + profile->mission_id,
            request->priority,
            next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for station order " + profile->mission_id,
        "queued station transport order " + profile->mission_id + " priority " +
            std::to_string(request->priority));
    response->success = admission.success;
    response->mission_id = profile->mission_id;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    if (!admission.success) {
      station_transport_executions_.erase(profile->mission_id);
      ReleaseAllResourcesForMission(profile->mission_id);
    }
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void SubmitStationOrderBatch(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::vector<std::string> sanitized_order_ids;
    sanitized_order_ids.reserve(request->order_ids.size());
    for (const auto& order_id : request->order_ids) {
      sanitized_order_ids.push_back(SanitizeId(order_id));
    }

    robot_tasks::StationOrderBatchSubmitRequest plan_request;
    plan_request.batch_id = SanitizeId(request->batch_id);
    plan_request.original_order_ids = request->order_ids;
    plan_request.sanitized_order_ids = sanitized_order_ids;
    plan_request.pickup_station_ids = request->pickup_station_ids;
    plan_request.dropoff_station_ids = request->dropoff_station_ids;
    plan_request.priority = request->priority;
    plan_request.start_if_idle = request->start_if_idle;
    plan_request.preempt_current = request->preempt_current;
    plan_request.continue_on_error = request->continue_on_error;
    const auto plan = robot_tasks::PlanStationOrderBatchSubmit(plan_request);
    if (!plan.accepted) {
      response->success = false;
      response->message = plan.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    for (const auto& item : plan.items) {
      if (!item.accepted) {
        response->rejected_order_ids.push_back(item.reject_order_id);
        response->rejected_messages.push_back(item.reject_message);
        continue;
      }
      auto station_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
      station_request->order_id = item.station_order_id;
      station_request->pickup_station = item.pickup_station;
      station_request->dropoff_station = item.dropoff_station;
      station_request->priority = item.priority;
      station_request->start_if_idle = item.start_if_idle;
      station_request->preempt_current = item.preempt_current;

      auto station_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
      SubmitStationTransportOrder(station_request, station_response);
      response->queue_size = station_response->queue_size;
      if (station_response->success) {
        response->mission_ids.push_back(station_response->mission_id);
        continue;
      }
      response->rejected_order_ids.push_back(item.order_id);
      response->rejected_messages.push_back(station_response->message);
      if (!request->continue_on_error) {
        break;
      }
    }

    response->accepted_count = static_cast<int>(response->mission_ids.size());
    response->rejected_count = static_cast<int>(response->rejected_order_ids.size());
    response->queue_size = static_cast<int>(mission_queue_.size());
    const auto summary = robot_tasks::PlanStationOrderBatchSubmitSummary(
        plan_request.batch_id, request->continue_on_error, response->accepted_count,
        response->rejected_count);
    response->success = summary.success;
    response->message = summary.message;
  }

  template <typename RequestT, typename ResponseT>
  void CancelStationOrderBatch(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto batch_id = SanitizeId(request->batch_id);
    if (batch_id.empty()) {
      response->success = false;
      response->message = "batch_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    robot_tasks::StationOrderBatchCancelRequest cancel_request;
    cancel_request.queue = mission_queue_;
    cancel_request.batch_id = batch_id;
    cancel_request.cancel_active = request->cancel_active;
    cancel_request.mission_active = RunnerBusy();
    cancel_request.active_mission_id = ActiveMissionId();
    const auto decision = robot_tasks::PlanStationOrderBatchCancel(cancel_request);
    mission_queue_ = decision.remaining_queue;
    response->canceled_mission_ids = decision.canceled_mission_ids;
    response->active_cancel_requested = decision.active_cancel_requested;

    for (const auto& mission_id : decision.release_mission_ids) {
      ReleaseAllResourcesForMission(mission_id);
      station_transport_executions_.erase(mission_id);
    }

    if (decision.active_cancel_requested) {
      RequestActiveMissionCancel("station order batch cancellation requested for " + batch_id);
    }

    response->queue_size = decision.queue_size;
    response->success = decision.success;
    response->message = decision.message;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void EstimateStationTransport(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto profile = CreateStationEstimateProfile(*request, response);
    if (!profile.has_value()) {
      return;
    }
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return;
    }
    FillStationEstimateResponse(
        *profile, *catalog, request->pickup_station, request->dropoff_station,
        request->nominal_speed_mps, request->battery_voltage, response.get());
  }

  template <typename RequestT, typename ResponseT>
  void SubmitStationSequenceTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    std::string message;
    const auto profiles = BuildStationSequenceProfiles(*request, *catalog, &message);
    if (!profiles.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto conflict =
        robot_tasks::CheckStationSequenceConflicts(*profiles, ActiveOrQueuedMissionIds());
    if (!conflict.accepted) {
      response->success = false;
      response->message = conflict.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    for (const auto& leg : *profiles) {
      const auto preflight =
          ValidateStationMissionForQueue(leg.profile, *catalog, leg.pickup_id, leg.dropoff_id);
      const auto admission =
          robot_tasks::CheckStationSequenceLegPreflight(leg, preflight);
      if (!admission.accepted) {
        response->success = false;
        response->message = admission.message;
        response->queue_size = static_cast<int>(mission_queue_.size());
        return;
      }
    }

    const auto queued_missions = robot_tasks::BuildStationSequenceQueuedMissions(
        *profiles, request->priority, next_queue_sequence_);
    next_queue_sequence_ += queued_missions.size();
    response->mission_ids = robot_tasks::StationSequenceMissionIds(*profiles);
    const auto admission = AdmitQueuedMissions(
        queued_missions, request->start_if_idle, request->preempt_current,
        "preempting active mission for station sequence " + SanitizeId(request->order_id),
        robot_tasks::BuildStationSequenceQueuedMessage(queued_missions.size()));
    response->success = admission.success;
    response->queued_count = static_cast<int>(queued_missions.size());
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void EstimateStationSequence(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return;
    }

    std::string message;
    const auto profile = BuildStationSequenceEstimateProfile(*request, *catalog, &message);
    if (!profile.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    FillStationSequenceEstimateResponse(
        *profile, *catalog, request->station_ids, request->nominal_speed_mps,
        request->battery_voltage, response.get());
  }

  std::vector<std::string> ReservedResourceIds() const {
    return robot_tasks::ReservedResourceIds(facility_reservations_);
  }

  const robot_tasks::FacilityResource* SelectChargingResource(
      const robot_tasks::FacilityCatalog& catalog, const std::string& requested_charger_id,
      std::string* message) const {
    if (!requested_charger_id.empty()) {
      const auto* resource = robot_tasks::FindFacilityResource(catalog, requested_charger_id);
      if (resource == nullptr) {
        if (message != nullptr) {
          *message = "unknown charger resource: " + requested_charger_id;
        }
        return nullptr;
      }
      if (resource->type != "charger") {
        if (message != nullptr) {
          *message = "facility resource is not a charger: " + requested_charger_id;
        }
        return nullptr;
      }
      if (!ResourceAvailable(*resource)) {
        if (message != nullptr) {
          *message = "charger unavailable: " + requested_charger_id;
        }
        return nullptr;
      }
      return resource;
    }
    const auto* resource =
        robot_tasks::FindFirstAvailableResource(catalog, "charger", ReservedResourceIds());
    if (resource == nullptr && message != nullptr) {
      *message = "no available charger resource";
    }
    return resource;
  }

  template <typename RequestT, typename ResponseT>
  void RequestCharging(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto facilities = LoadFacilities(&message);
    if (!facilities.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* charger = SelectChargingResource(*facilities, request->charger_id, &message);
    if (charger == nullptr) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      const auto gate = robot_tasks::PlanChargingRequestGate(
          robot_tasks::ChargingRequestGateRequest{
              "", charger->id, charger->station_id, false,
              "failed to load station catalog: " + station_file_});
      response->success = false;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* station = robot_tasks::FindStation(*stations, charger->station_id);
    if (station == nullptr) {
      const auto gate = robot_tasks::PlanChargingRequestGate(
          robot_tasks::ChargingRequestGateRequest{
              "", charger->id, charger->station_id, true, "", false});
      response->success = false;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto sanitized_request_id = SanitizeId(request->request_id);
    const std::string mission_id =
        sanitized_request_id.empty()
            ? "charging_request_" + std::to_string(next_charging_request_sequence_++)
            : "charging_request_" + sanitized_request_id;
    if (MissionIdActiveOrQueued(mission_id)) {
      const auto gate = robot_tasks::PlanChargingRequestGate(
          robot_tasks::ChargingRequestGateRequest{
              mission_id, charger->id, charger->station_id, true, "", true, true});
      response->success = false;
      response->mission_id = gate.mission_id;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    auto profile = robot_tasks::BuildStationWaypointProfile(mission_id, *station);
    const auto preflight = ValidateMissionForQueue(profile);
    const auto gate = robot_tasks::PlanChargingRequestGate(
        robot_tasks::ChargingRequestGateRequest{
            mission_id, charger->id, charger->station_id, true, "", true, false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    facility_reservations_[charger->id] =
        robot_tasks::ResourceReservation{mission_id, mission_id, "reserved"};
    facility_resource_by_mission_[mission_id] = charger->id;
    const auto admission = AdmitQueuedMission(
        QueuedMission{profile, "charging_request:" + mission_id, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for charging request " + mission_id,
        "queued charging request " + mission_id + " using charger " + charger->id);
    response->success = admission.success;
    response->mission_id = mission_id;
    response->charger_id = charger->id;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void RequestOpportunityCharging(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    robot_tasks::OpportunityChargingPlanRequest plan_request;
    plan_request.requested_battery_voltage = request->battery_voltage;
    plan_request.current_battery_voltage = current_battery_voltage_;
    plan_request.requested_opportunity_threshold_voltage =
        request->opportunity_threshold_voltage;
    plan_request.default_opportunity_threshold_voltage = opportunity_charge_threshold_voltage_;
    plan_request.requested_critical_threshold_voltage = request->critical_threshold_voltage;
    plan_request.default_critical_threshold_voltage = critical_charge_threshold_voltage_;
    plan_request.requested_require_idle_queue = request->require_idle_queue;
    plan_request.default_require_idle_queue = opportunity_charge_require_idle_queue_;
    plan_request.mission_active = RunnerBusy();
    plan_request.queue_empty = mission_queue_.empty();
    plan_request.requested_priority = request->priority;
    plan_request.default_priority = opportunity_charge_priority_;
    plan_request.requested_preempt_current = request->preempt_current;
    const auto decision = robot_tasks::PlanOpportunityCharging(plan_request);

    response->evaluated_battery_voltage = decision.evaluated_battery_voltage;
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (!decision.success || !decision.queue_charging) {
      response->success = decision.success;
      response->charging_queued = false;
      response->message = decision.message;
      return;
    }

    auto charging_request =
        std::make_shared<robot_interfaces_mission::srv::RequestCharging::Request>();
    charging_request->request_id = request->request_id;
    charging_request->charger_id = request->charger_id;
    charging_request->priority = decision.priority;
    charging_request->start_if_idle = request->start_if_idle;
    charging_request->preempt_current = decision.preempt_current;

    auto charging_response =
        std::make_shared<robot_interfaces_mission::srv::RequestCharging::Response>();
    RequestCharging(charging_request, charging_response);
    response->success = charging_response->success;
    response->charging_queued = charging_response->success;
    response->message = decision.queued_message_prefix + charging_response->message;
    response->mission_id = charging_response->mission_id;
    response->charger_id = charging_response->charger_id;
    response->queue_size = charging_response->queue_size;
  }

  template <typename ResponseT>
  void FillEstimateResponse(
      const robot_tasks::MissionProfile& profile, const double requested_speed_mps,
      const double requested_battery_voltage, ResponseT* response) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    const auto path_distance_m = PreferredPathDistanceOverride(profile, std::nullopt);
    const auto preflight = robot_tasks::ValidateMissionPreflight(
        profile, zone_catalog, MakePreflightConfig(requested_speed_mps, requested_battery_voltage),
        path_distance_m);
    FillEstimateFields(
        profile, preflight, response,
        path_distance_m.has_value() ? "estimated mission cost using Nav2 path length"
                                    : "estimated mission cost");
  }

  template <typename ResponseT>
  void FillStationEstimateResponse(
      const robot_tasks::MissionProfile& profile, const robot_tasks::StationCatalog& catalog,
      const std::string& pickup_station, const std::string& dropoff_station,
      const double requested_speed_mps, const double requested_battery_voltage,
      ResponseT* response) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    const auto path_distance_m = TryComputeNav2PathDistance(profile);
    const auto preflight = robot_tasks::ValidateStationTransportPreflight(
        profile, catalog, pickup_station, dropoff_station, zone_catalog,
        MakePreflightConfig(requested_speed_mps, requested_battery_voltage), path_distance_m);
    FillEstimateFields(
        profile, preflight, response,
        path_distance_m.has_value() ? "estimated station transport Nav2 path cost"
                                    : "estimated station transport route cost");
  }

  template <typename ResponseT>
  void FillStationSequenceEstimateResponse(
      const robot_tasks::MissionProfile& profile, const robot_tasks::StationCatalog& catalog,
      const std::vector<std::string>& station_ids, const double requested_speed_mps,
      const double requested_battery_voltage, ResponseT* response) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    auto config = MakePreflightConfig(requested_speed_mps, requested_battery_voltage);
    robot_tasks::MissionPreflightResult preflight;
    preflight.allowed = true;
    preflight.message = "station sequence accepted";
    double total_route_distance_m = 0.0;

    for (std::size_t index = 1; index < station_ids.size(); ++index) {
      const auto route_distance =
          robot_tasks::EstimateRouteDistance(catalog, station_ids[index - 1], station_ids[index]);
      if (!route_distance.has_value()) {
        preflight.allowed = false;
        preflight.message =
            "no enabled station route from " + station_ids[index - 1] + " to " + station_ids[index];
        break;
      }
      total_route_distance_m += *route_distance;
    }
    const auto path_distance_m = TryComputeNav2PathDistance(profile);
    const double effective_distance_m =
        path_distance_m.has_value() ? *path_distance_m : total_route_distance_m;
    preflight.cost = robot_tasks::EstimateDistanceCost(
        effective_distance_m, static_cast<int>(profile.waypoints.size()),
        config.nominal_speed_mps, config.battery_voltage, config.battery_drop_per_meter,
        config.minimum_battery_voltage,
        path_distance_m.has_value() ? "planned_path" : "station_route_graph");
    if (preflight.allowed) {
      const auto zone_preflight =
          robot_tasks::ValidateMissionPreflight(profile, zone_catalog, config, effective_distance_m);
      if (!zone_preflight.allowed) {
        preflight.allowed = false;
        preflight.message = zone_preflight.message;
      }
      preflight.cost = zone_preflight.cost;
    }
    FillEstimateFields(
        profile, preflight, response,
        path_distance_m.has_value() ? "estimated station sequence Nav2 path cost"
                                    : "estimated station sequence route cost");
  }

  std::optional<double> PreferredPathDistanceOverride(
      const robot_tasks::MissionProfile& profile,
      const std::optional<double> fallback_distance_m) const {
    const auto nav2_distance_m = TryComputeNav2PathDistance(profile);
    if (nav2_distance_m.has_value()) {
      return nav2_distance_m;
    }
    return fallback_distance_m;
  }

  std::optional<double> TryComputeNav2PathDistance(
      const robot_tasks::MissionProfile& profile) const {
    if (!estimate_use_nav2_path_ || profile.waypoints.size() < 2 || !nav2_path_node_ ||
        !nav2_path_client_) {
      return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(nav2_path_mutex_);
    const auto server_timeout =
        std::chrono::milliseconds(std::max(0, estimate_nav2_server_timeout_ms_));
    if (!nav2_path_client_->wait_for_action_server(server_timeout)) {
      return std::nullopt;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nav2_path_node_);
    const auto result_timeout =
        std::chrono::milliseconds(std::max(1, estimate_nav2_result_timeout_ms_));
    double total_distance_m = 0.0;
    for (std::size_t index = 1; index < profile.waypoints.size(); ++index) {
      const auto stamp = nav2_path_node_->now();
      ComputePathToPose::Goal goal;
      goal.start = PoseStampedFromWaypoint(profile.waypoints[index - 1], profile.frame_id, stamp);
      goal.goal = PoseStampedFromWaypoint(profile.waypoints[index], profile.frame_id, stamp);
      goal.planner_id = estimate_nav2_planner_id_;
      goal.use_start = true;

      auto goal_future = nav2_path_client_->async_send_goal(goal);
      if (executor.spin_until_future_complete(goal_future, result_timeout) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }
      auto goal_handle = goal_future.get();
      if (!goal_handle) {
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }

      auto result_future = nav2_path_client_->async_get_result(goal_handle);
      if (executor.spin_until_future_complete(result_future, result_timeout) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        nav2_path_client_->async_cancel_goal(goal_handle);
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }

      const auto wrapped = result_future.get();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || wrapped.result == nullptr ||
          wrapped.result->error_code != ComputePathToPose::Result::NONE) {
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }
      total_distance_m += PathLength(wrapped.result->path);
    }
    executor.remove_node(nav2_path_node_);
    return total_distance_m;
  }

  double RequestedOrDefaultSpeed(const double requested_speed_mps) const {
    return requested_speed_mps > 0.0 ? requested_speed_mps : estimate_nominal_speed_mps_;
  }

  double RequestedOrDefaultBattery(const double requested_battery_voltage) const {
    return requested_battery_voltage > 0.0 ? requested_battery_voltage : dock_low_battery_voltage_ + 2.0;
  }

  template <typename ResponseT>
  void FillEstimateFields(
      const robot_tasks::MissionProfile& profile,
      const robot_tasks::MissionPreflightResult& preflight, ResponseT* response,
      const std::string& accepted_message) const {
    response->success = true;
    response->message =
        preflight.allowed ? accepted_message : accepted_message + "; preflight rejected: " + preflight.message;
    response->mission_id = profile.mission_id;
    response->waypoint_count = preflight.cost.waypoint_count;
    response->distance_m = preflight.cost.distance_m;
    response->eta_sec = preflight.cost.eta_sec;
    response->battery_drop_v = preflight.cost.battery_drop_v;
    response->projected_battery_voltage = preflight.cost.projected_battery_voltage;
    response->battery_sufficient = preflight.cost.battery_sufficient;
  }

  robot_tasks::MissionPreflightConfig MakePreflightConfig() const {
    return MakePreflightConfig(0.0, 0.0);
  }

  robot_tasks::MissionPreflightConfig MakePreflightConfig(
      const double requested_speed_mps, const double requested_battery_voltage) const {
    robot_tasks::MissionPreflightConfig config;
    config.enabled = preflight_enabled_;
    config.map_name = preflight_map_name_;
    config.default_speed_limit_mps = preflight_default_speed_limit_mps_;
    config.nominal_speed_mps = RequestedOrDefaultSpeed(requested_speed_mps);
    config.battery_voltage = requested_battery_voltage > 0.0
                                  ? requested_battery_voltage
                                  : (current_battery_voltage_ > 0.0
                                         ? current_battery_voltage_
                                         : dock_low_battery_voltage_ + 2.0);
    config.battery_drop_per_meter = estimate_battery_drop_per_meter_;
    config.minimum_battery_voltage = estimate_min_battery_voltage_;
    config.require_station_route = preflight_require_station_route_;
    return config;
  }

  robot_tasks::MissionPreflightResult ValidateMissionForQueue(
      const robot_tasks::MissionProfile& profile) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    return robot_tasks::ValidateMissionPreflight(
        profile, zone_catalog, MakePreflightConfig(),
        PreferredPathDistanceOverride(profile, std::nullopt));
  }

  robot_tasks::MissionPreflightResult ValidateStationMissionForQueue(
      const robot_tasks::MissionProfile& profile, const robot_tasks::StationCatalog& catalog,
      const std::string& pickup_station, const std::string& dropoff_station) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    return robot_tasks::ValidateStationTransportPreflight(
        profile, catalog, pickup_station, dropoff_station, zone_catalog, MakePreflightConfig(),
        TryComputeNav2PathDistance(profile));
  }

  template <typename ResponseT>
  std::optional<robot_tasks::StationCatalog> LoadStations(
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      if (response != nullptr) {
        response->success = false;
        response->message = "failed to load station catalog: " + station_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  template <typename RequestT, typename ResponseT>
  std::optional<robot_tasks::MissionProfile> CreateStationTransportOrderProfile(
      const RequestT& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return std::nullopt;
    }

    std::string message;
    const auto stations = robot_tasks::ResolveStationPair(
        *catalog, request.pickup_station, request.dropoff_station, &message);
    if (!stations.has_value()) {
      response->success = false;
      response->message = message;
      return std::nullopt;
    }

    const auto sanitized_order_id = SanitizeId(request.order_id);
    std::string mission_id;
    if (sanitized_order_id.empty()) {
      mission_id = "station_order_" + std::to_string(next_station_order_sequence_++);
    } else {
      mission_id = "station_order_" + sanitized_order_id;
    }
    auto profile = robot_tasks::BuildTwoWaypointProfile(
        mission_id, stations->pickup.frame_id, stations->pickup.x, stations->pickup.y,
        stations->pickup.yaw, stations->dropoff.x, stations->dropoff.y, stations->dropoff.yaw);

    if (!robot_tasks::ValidateMissionProfile(profile, &message)) {
      response->success = false;
      response->mission_id = profile.mission_id;
      response->message = "invalid station transport mission: " + message;
      return std::nullopt;
    }
    return profile;
  }

  template <typename RequestT, typename ResponseT>
  std::optional<robot_tasks::MissionProfile> CreateStationEstimateProfile(
      const RequestT& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return std::nullopt;
    }

    std::string message;
    const auto stations = robot_tasks::ResolveStationPair(
        *catalog, request.pickup_station, request.dropoff_station, &message);
    if (!stations.has_value()) {
      response->success = false;
      response->message = message;
      return std::nullopt;
    }

    return robot_tasks::BuildTwoWaypointProfile(
        "estimate_" + SanitizeId(request.pickup_station) + "_to_" +
            SanitizeId(request.dropoff_station),
        stations->pickup.frame_id, stations->pickup.x, stations->pickup.y, stations->pickup.yaw,
        stations->dropoff.x, stations->dropoff.y, stations->dropoff.yaw);
  }

  template <typename RequestT>
  std::optional<std::vector<robot_tasks::StationSequenceLeg>> BuildStationSequenceProfiles(
      const RequestT& request,
      const robot_tasks::StationCatalog& catalog, std::string* message) {
    const auto order_id = SanitizeId(request.order_id);
    const auto base_id = order_id.empty()
                             ? "station_sequence_" +
                                   std::to_string(next_station_sequence_order_sequence_++)
                             : "station_sequence_" + order_id;
    return robot_tasks::BuildStationSequenceProfiles(catalog, request.station_ids, base_id, message);
  }

  template <typename RequestT>
  std::optional<robot_tasks::MissionProfile> BuildStationSequenceEstimateProfile(
      const RequestT& request,
      const robot_tasks::StationCatalog& catalog, std::string* message) const {
    return robot_tasks::BuildStationSequenceEstimateProfile(catalog, request.station_ids, message);
  }

  template <typename RequestT, typename ResponseT>
  std::optional<robot_tasks::MissionProfile> CreateTransportOrderProfile(
      const RequestT& request,
      const std::shared_ptr<ResponseT>& response) {
    if (!std::isfinite(request.pickup_x) || !std::isfinite(request.pickup_y) ||
        !std::isfinite(request.pickup_yaw) || !std::isfinite(request.dropoff_x) ||
        !std::isfinite(request.dropoff_y) || !std::isfinite(request.dropoff_yaw)) {
      response->success = false;
      response->message = "transport order contains non-finite waypoint values";
      return std::nullopt;
    }

    robot_tasks::MissionProfile profile;
    const auto sanitized_order_id = SanitizeId(request.order_id);
    if (sanitized_order_id.empty()) {
      profile.mission_id = "transport_order_" + std::to_string(next_transport_order_sequence_++);
    } else {
      profile.mission_id = "transport_order_" + sanitized_order_id;
    }
    profile = robot_tasks::BuildTwoWaypointProfile(
        profile.mission_id, request.frame_id.empty() ? "map" : request.frame_id, request.pickup_x,
        request.pickup_y, request.pickup_yaw, request.dropoff_x, request.dropoff_y,
        request.dropoff_yaw);

    std::string message;
    if (!robot_tasks::ValidateMissionProfile(profile, &message)) {
      response->success = false;
      response->mission_id = profile.mission_id;
      response->message = "invalid transport order mission: " + message;
      return std::nullopt;
    }
    return profile;
  }

  bool MissionIdActiveOrQueued(const std::string& mission_id) const {
    const auto transport = station_transport_executions_.find(mission_id);
    if (transport != station_transport_executions_.end() &&
        !robot_tasks::IsStationTransportTerminal(transport->second.state.phase)) {
      return true;
    }
    if (RunnerBusy() && ActiveMissionId() == mission_id) {
      return true;
    }
    return robot_tasks::MissionIdQueued(mission_queue_, mission_id);
  }

  std::unordered_set<std::string> ActiveOrQueuedMissionIds() const {
    std::unordered_set<std::string> mission_ids;
    if (RunnerBusy() && !ActiveMissionId().empty()) {
      mission_ids.insert(ActiveMissionId());
    }
    for (const auto& mission : mission_queue_) {
      mission_ids.insert(mission.profile.mission_id);
    }
    for (const auto& item : station_transport_executions_) {
      if (!robot_tasks::IsStationTransportTerminal(item.second.state.phase)) {
        mission_ids.insert(item.first);
      }
    }
    return mission_ids;
  }

  static std::string RouteLockId(const std::string& from, const std::string& to) {
    return robot_tasks::RouteLockId(from, to);
  }

  std::vector<std::string> BuildRouteLockIds(const std::vector<std::string>& station_path) const {
    return robot_tasks::BuildRouteLockIds(station_path, traffic_intersection_locks_enabled_);
  }

  bool ReserveRouteLocksForMission(
      const std::string& mission_id, const std::vector<std::string>& route_lock_ids,
      std::string* message) {
    return robot_tasks::ReserveRouteLocksForMission(
        route_locks_, route_locks_by_mission_, mission_id, route_lock_ids, message);
  }

  void ReleaseRouteLocksForMission(const std::string& mission_id) {
    robot_tasks::ReleaseRouteLocksForMission(route_locks_, route_locks_by_mission_, mission_id);
  }

  void SortMissionQueue() {
    robot_tasks::SortMissionQueue(mission_queue_);
  }

  void ApplyQueuedMissionStartPolicy(
      const bool start_if_idle, const bool preempt_current,
      const std::string& active_cancel_message) {
    robot_tasks::QueuedMissionStartRequest request;
    request.start_if_idle = start_if_idle;
    request.preempt_current = preempt_current;
    request.mission_active = RunnerBusy();
    robot_tasks::QueuedMissionStartBehaviorTreeInput input;
    input.request = request;
    const auto start_result = robot_tasks::TickQueuedMissionStartBehaviorTree(input);
    const auto& decision = start_result.decision;
    if (decision.run_queue) {
      queue_running_ = true;
    }
    if (decision.request_queue_start) {
      queue_start_requested_ = true;
    }
    if (decision.cancel_active) {
      RequestActiveMissionCancel(active_cancel_message);
    }
  }

  robot_tasks::MissionQueueAdmissionDecision AdmitQueuedMissions(
      const std::vector<QueuedMission>& missions, const bool start_if_idle,
      const bool preempt_current, const std::string& active_cancel_message,
      const std::string& response_message, const bool set_state_when_idle = true) {
    for (const auto& mission : missions) {
      mission_queue_.push_back(mission);
    }
    SortMissionQueue();
    ApplyQueuedMissionStartPolicy(start_if_idle, preempt_current, active_cancel_message);

    robot_tasks::MissionQueueAdmissionRequest request;
    request.mission_active = RunnerBusy();
    request.queue_size = mission_queue_.size();
    request.message = response_message;
    request.set_state_when_idle = set_state_when_idle;
    return robot_tasks::PlanMissionQueueAdmission(request);
  }

  robot_tasks::MissionQueueAdmissionDecision AdmitQueuedMission(
      const QueuedMission& mission, const bool start_if_idle, const bool preempt_current,
      const std::string& active_cancel_message, const std::string& response_message,
      const bool set_state_when_idle = true) {
    return AdmitQueuedMissions(
        std::vector<QueuedMission>{mission}, start_if_idle, preempt_current,
        active_cancel_message, response_message, set_state_when_idle);
  }

  void ApplyMissionQueueAdmissionState(
      const robot_tasks::MissionQueueAdmissionDecision& decision) {
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  void StartNextQueuedMission(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    robot_tasks::MissionQueueBehaviorTreeInput input;
    input.queue = mission_queue_;
    input.paused_order_ids = paused_workflow_orders_;
    const auto queue_dispatch = robot_tasks::TickMissionQueueBehaviorTree(input);
    const auto& dispatch = queue_dispatch.dispatch;

    if (dispatch.queue_empty) {
      queue_running_ = false;
      if (response != nullptr) {
        response->success = false;
        response->message = "mission queue is empty";
      }
      return;
    }
    if (dispatch.all_paused) {
      queue_start_requested_ = false;
      if (response != nullptr) {
        response->success = false;
        response->message = "all queued workflow missions are paused";
      }
      SetState("WORKFLOW_PAUSED", "all queued workflow missions are paused", true);
      return;
    }
    if (!queue_dispatch.success || !dispatch.mission.has_value()) {
      if (response != nullptr) {
        response->success = false;
        response->message = dispatch.message.empty() ? "failed to dispatch queued mission"
                                                     : dispatch.message;
      }
      return;
    }

    const auto transport =
        station_transport_executions_.find(dispatch.mission->profile.mission_id);
    if (transport != station_transport_executions_.end() &&
        !robot_tasks::IsStationTransportTerminal(transport->second.state.phase)) {
      if (payload_loaded_) {
        queue_running_ = true;
        queue_start_requested_ = false;
        if (response != nullptr) {
          response->success = false;
          response->message =
              "station transport pickup requires an empty robot payload";
        }
        SetState(
            "QUEUED", "station transport waiting for the robot payload to be emptied",
            true);
        return;
      }

      mission_queue_ = dispatch.remaining_queue;
      profile_ = transport->second.pickup_profile;
      mission_file_ = dispatch.mission->mission_file;
      active_station_transport_id_ = transport->first;
      std::string dispatch_message;
      const bool sent = DispatchStationTransportLeg(
          transport->first, robot_tasks::StationTransportLeg::kPickup,
          &dispatch_message);
      if (response != nullptr) {
        response->success = sent;
        response->message = dispatch_message;
      }
      return;
    }

    mission_queue_ = dispatch.remaining_queue;
    profile_ = dispatch.mission->profile;
    mission_file_ = dispatch.mission->mission_file;
    DispatchMission(response, "queued mission profile sent");
  }

  std::optional<robot_tasks::MissionProfile> LoadDockProfile(std::string* message) const {
    const auto profile = robot_tasks::LoadMissionProfile(dock_mission_file_);
    if (!profile.has_value()) {
      if (message != nullptr) {
        *message = "failed to load dock mission profile: " + dock_mission_file_;
      }
      return std::nullopt;
    }
    if (message != nullptr) {
      *message = "loaded dock mission " + profile->mission_id;
    }
    return profile;
  }

  bool DockAlreadyRequestedOrRunning(const std::string& dock_mission_id) const {
    if (RunnerBusy() && ActiveMissionId() == dock_mission_id) {
      return true;
    }
    return std::any_of(
        mission_queue_.begin(), mission_queue_.end(), [&](const QueuedMission& mission) {
          return mission.profile.mission_id == dock_mission_id;
        });
  }

  void QueueDockMission(const robot_tasks::MissionProfile& dock_profile, const std::string& reason) {
    mission_queue_.push_back(
        QueuedMission{dock_profile, dock_mission_file_, dock_priority_, next_queue_sequence_++});
    SortMissionQueue();
    queue_running_ = true;
    queue_start_requested_ = true;
    SetState("QUEUED", reason + ": return to dock queued", true);
  }

  void RequestDockReturn(
      const std::string& reason,
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    std::string message;
    const auto dock_profile = LoadDockProfile(&message);
    robot_tasks::DockReturnRequest request;
    request.dock_profile_available = dock_profile.has_value();
    request.load_error_message = message;
    request.dock_already_requested_or_running =
        dock_profile.has_value() && DockAlreadyRequestedOrRunning(dock_profile->mission_id);
    request.mission_active = RunnerBusy();
    const auto decision = robot_tasks::PlanDockReturn(request);

    if (decision.set_error_state) {
      SetState("ERROR", decision.error_message, false);
      if (response != nullptr) {
        response->success = decision.response_success;
        response->message = decision.response_message;
      }
      return;
    }
    if (decision.branch == "already_requested") {
      if (response != nullptr) {
        response->success = decision.response_success;
        response->message = decision.response_message;
      }
      return;
    }

    if (decision.queue_dock_mission) {
      QueueDockMission(*dock_profile, reason);
      if (decision.cancel_active_mission) {
        RequestActiveMissionCancel(reason + ": canceling active mission before docking");
      }
      if (response != nullptr) {
        response->success = decision.response_success;
        response->message = decision.response_message;
      }
      return;
    }

    if (decision.dispatch_dock_mission) {
      profile_ = *dock_profile;
      mission_file_ = dock_mission_file_;
      queue_running_ = false;
      queue_start_requested_ = false;
      DispatchMission(response, "return to dock mission sent");
    }
  }

  void HandleChassisState(const robot_interfaces::msg::ChassisState& state) {
    current_battery_voltage_ = state.battery_voltage;
    robot_tasks::LowBatteryDockRequest request;
    request.enabled = return_to_dock_on_low_battery_;
    request.battery_voltage = state.battery_voltage;
    request.threshold_voltage = dock_low_battery_voltage_;
    request.request_already_sent = low_battery_dock_request_sent_;
    const auto decision = robot_tasks::PlanLowBatteryDock(request);
    low_battery_dock_request_sent_ = decision.low_battery_request_sent;
    if (decision.request_dock_return) {
      RequestDockReturn(decision.reason, nullptr);
    }
  }

  bool MatchesActiveStationTransportContext(
      const StationTransportDispatchContext& context) const {
    if (context.mission_id.empty() ||
        context.mission_id != active_station_transport_id_) {
      return false;
    }
    const auto it = station_transport_executions_.find(context.mission_id);
    return it != station_transport_executions_.end() &&
           it->second.state.active_leg == context.leg &&
           it->second.state.dispatch_token == context.dispatch_token;
  }

  void ApplyStationTransportPostActions(const bool mission_succeeded) {
    robot_tasks::MissionResultWorkflowDecision result;
    result.mission_succeeded = mission_succeeded;
    ApplyMissionResultPostActions(result, false, LoadDockProfile(nullptr));
  }

  void FinishStationTransportTerminal(
      const std::string& mission_id, const std::string& state,
      const std::string& message, const bool mission_succeeded) {
    active_station_transport_id_.clear();
    cancellation_requested_ = false;
    paused_ = false;
    current_goal_handle_.reset();
    mission_active_ = false;
    SetState(state, message, true);
    station_transport_executions_.erase(mission_id);
    ApplyStationTransportPostActions(mission_succeeded);
  }

  void CompleteStationTransportSuccess(
      const std::string& mission_id, const std::string& message) {
    ReleaseAllResourcesForMission(mission_id);
    FinishStationTransportTerminal(mission_id, "FINISHED", message, true);
  }

  void HandleStationTransportNavigationResult(
      const StationTransportDispatchContext& context, const bool succeeded,
      const bool canceled, const std::string& result_message) {
    if (!MatchesActiveStationTransportContext(context)) {
      RCLCPP_WARN(
          get_logger(), "ignoring stale station transport result for %s leg %s token %llu",
          context.mission_id.c_str(),
          robot_tasks::StationTransportLegName(context.leg),
          static_cast<unsigned long long>(context.dispatch_token));
      return;
    }

    auto it = station_transport_executions_.find(context.mission_id);
    const auto transition = robot_tasks::PlanStationTransportNavigationResult(
        it->second.state, context.leg, context.dispatch_token, succeeded, canceled);
    if (!transition.accepted) {
      RCLCPP_WARN(
          get_logger(), "station transport result rejected: %s",
          transition.message.c_str());
      return;
    }

    it->second.state = transition.next_state;
    mission_active_ = false;
    paused_ = false;
    current_goal_handle_.reset();
    cancellation_requested_ = false;

    const std::string message =
        result_message.empty() ? transition.message : result_message;
    if (transition.release_resources) {
      ReleaseAllResourcesForMission(context.mission_id);
    }

    switch (transition.next_state.phase) {
      case robot_tasks::StationTransportPhase::kWaitLoad:
        SetState(
            "WAIT_LOAD",
            "arrived at pickup station " +
                transition.next_state.pickup_station_id +
                "; waiting for load confirmation",
            true);
        return;
      case robot_tasks::StationTransportPhase::kWaitUnload:
        SetState(
            "WAIT_UNLOAD",
            "arrived at dropoff station " +
                transition.next_state.dropoff_station_id +
                "; waiting for unload confirmation",
            true);
        return;
      case robot_tasks::StationTransportPhase::kNeedsOperator:
        SetState("NEEDS_OPERATOR", message, false);
        return;
      case robot_tasks::StationTransportPhase::kFailed:
        FinishStationTransportTerminal(
            context.mission_id, "FAILED", message, false);
        return;
      case robot_tasks::StationTransportPhase::kCanceled:
        FinishStationTransportTerminal(
            context.mission_id, "CANCELED", message, false);
        return;
      default:
        SetState("ERROR", "unexpected station transport navigation result", false);
        return;
    }
  }

  void SendStationTransportGoal(
      const NavigateSequence::Goal& goal,
      const StationTransportDispatchContext& context) {
    auto options = rclcpp_action::Client<NavigateSequence>::SendGoalOptions();
    options.goal_response_callback =
        [this, context](GoalHandleNavigateSequence::SharedPtr goal_handle) {
          if (!MatchesActiveStationTransportContext(context)) {
            if (goal_handle) {
              action_client_->async_cancel_goal(goal_handle);
            }
            return;
          }
          if (!goal_handle) {
            HandleStationTransportNavigationResult(
                context, false, cancellation_requested_,
                "navigate sequence rejected station transport leg");
            return;
          }
          current_goal_handle_ = goal_handle;
          if (cancellation_requested_) {
            action_client_->async_cancel_goal(goal_handle);
            return;
          }
          const auto* transport = FindActiveStationTransport();
          SetState(
              transport == nullptr
                  ? "ERROR"
                  : robot_tasks::StationTransportPhaseName(
                        transport->state.phase),
              std::string("station transport ") +
                  robot_tasks::StationTransportLegName(context.leg) +
                  " navigation accepted",
              true);
        };
    options.result_callback =
        [this, context](const GoalHandleNavigateSequence::WrappedResult& result) {
          if (!MatchesActiveStationTransportContext(context)) {
            return;
          }
          const bool succeeded =
              result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              result.result && result.result->success;
          const bool canceled =
              result.code == rclcpp_action::ResultCode::CANCELED ||
              cancellation_requested_;
          HandleStationTransportNavigationResult(
              context, succeeded, canceled,
              result.result ? result.result->message : "");
        };
    options.feedback_callback =
        [this, context](
            GoalHandleNavigateSequence::SharedPtr,
            const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
          if (!MatchesActiveStationTransportContext(context)) {
            return;
          }
          current_goal_index_ = feedback->current_goal_index;
          if (feedback->state == "paused") {
            paused_ = true;
            SetState("PAUSED", feedback->message, true);
            return;
          }
          const auto* transport = FindActiveStationTransport();
          if (transport != nullptr && !paused_) {
            SetState(
                robot_tasks::StationTransportPhaseName(transport->state.phase),
                feedback->state + ": " + feedback->message, true);
          }
        };

    mission_active_ = true;
    autostart_sent_ = true;
    action_client_->async_send_goal(goal, options);
  }

  bool DispatchStationTransportLeg(
      const std::string& mission_id,
      const robot_tasks::StationTransportLeg leg, std::string* message) {
    const auto it = station_transport_executions_.find(mission_id);
    if (it == station_transport_executions_.end()) {
      if (message != nullptr) {
        *message = "station transport execution not found: " + mission_id;
      }
      return false;
    }

    const std::uint64_t dispatch_token = it->second.state.dispatch_token + 1U;
    const auto transition = robot_tasks::PlanStationTransportDispatch(
        it->second.state, leg, dispatch_token);
    if (!transition.accepted) {
      if (message != nullptr) {
        *message = transition.message;
      }
      return false;
    }

    it->second.state = transition.next_state;
    active_station_transport_id_ = mission_id;
    profile_ = leg == robot_tasks::StationTransportLeg::kPickup
                   ? it->second.pickup_profile
                   : it->second.dropoff_profile;
    paused_ = false;
    cancellation_requested_ = false;
    current_goal_handle_.reset();

    const StationTransportDispatchContext context{
        mission_id, leg, dispatch_token};
    SetState(
        robot_tasks::StationTransportPhaseName(it->second.state.phase),
        transition.message, true);

    if (!action_client_->wait_for_action_server(
            std::chrono::milliseconds(server_timeout_ms_))) {
      HandleStationTransportNavigationResult(
          context, false, false,
          "navigate sequence action server unavailable");
      if (message != nullptr) {
        *message = "navigate sequence action server unavailable";
      }
      return false;
    }

    NavigateSequence::Goal goal;
    goal.loop = false;
    for (const auto& waypoint : profile_->waypoints) {
      goal.goals.push_back(
          PoseStampedFromWaypoint(waypoint, profile_->frame_id, now()));
    }
    SendStationTransportGoal(goal, context);
    if (message != nullptr) {
      *message = transition.message;
    }
    return true;
  }

  void DispatchMission(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response,
                       const std::string& accepted_message) {
    paused_ = false;
    cancellation_requested_ = false;
    current_goal_handle_.reset();
    if (!profile_.has_value()) {
      LoadProfile(nullptr);
    }
    if (!profile_.has_value()) {
      if (response != nullptr) {
        response->success = false;
        response->message = last_message_;
      }
      return;
    }
    if (active_recovery_mission_id_ != profile_->mission_id) {
      active_recovery_mission_id_ = profile_->mission_id;
      current_failure_retry_count_ = 0;
    }
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms_))) {
      SetState("ERROR", "navigate sequence action server unavailable", true);
      if (response != nullptr) {
        response->success = false;
        response->message = last_message_;
      }
      return;
    }

    NavigateSequence::Goal goal;
    goal.loop = profile_->loop;
    for (const auto& waypoint : profile_->waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now();
      pose.header.frame_id = profile_->frame_id;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.orientation = QuaternionFromYaw(waypoint.yaw);
      goal.goals.push_back(pose);
    }
    SendMissionGoal(goal, response, accepted_message);
  }

  void SendMissionGoal(const NavigateSequence::Goal& goal,
                       const std::shared_ptr<std_srvs::srv::Trigger::Response>& response,
                       const std::string& accepted_message) {
    auto options = rclcpp_action::Client<NavigateSequence>::SendGoalOptions();
    options.goal_response_callback = [this](GoalHandleNavigateSequence::SharedPtr goal_handle) {
      if (!goal_handle) {
        mission_active_ = false;
        current_goal_handle_.reset();
        SetState("ERROR", "navigate sequence rejected mission profile", true);
        return;
      }
      current_goal_handle_ = goal_handle;
      SetState("RUNNING", "mission accepted by navigate sequence", true);
    };
    options.result_callback = [this](const GoalHandleNavigateSequence::WrappedResult& result) {
      const auto dock_profile = LoadDockProfile(nullptr);
      const std::string completed_mission_id = profile_.has_value() ? profile_->mission_id : "";
      const bool completed_dock =
          !completed_mission_id.empty() &&
          ((dock_profile.has_value() && completed_mission_id == dock_profile->mission_id) ||
           dock_by_mission_.find(completed_mission_id) != dock_by_mission_.end());
      mission_active_ = false;
      paused_ = false;
      current_goal_handle_.reset();
      robot_tasks::MissionResultWorkflowInput result_input;
      result_input.code = ToMissionActionResultCode(result.code);
      result_input.result_success = result.result && result.result->success;
      result_input.result_message = result.result ? result.result->message : "";
      result_input.cancellation_requested = cancellation_requested_;
      result_input.completed_dock = completed_dock;
      const auto mission_result = robot_tasks::TickMissionResultBehaviorTree(result_input);
      if (!mission_result.success) {
        SetState("ERROR", "mission result behavior tree failed", true);
        return;
      }
      const auto& result_decision = mission_result.decision;

      if (!ApplyMissionResultDecision(
              result_decision, completed_mission_id, completed_dock)) {
        return;
      }
      ApplyMissionResultPostActions(result_decision, completed_dock, dock_profile);
    };
    options.feedback_callback =
        [this](
            GoalHandleNavigateSequence::SharedPtr,
            const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
          current_goal_index_ = feedback->current_goal_index;
          if (feedback->state == "paused") {
            paused_ = true;
            SetState("PAUSED", feedback->message, true);
          } else if (paused_) {
            SetState("PAUSED", feedback->state + ": " + feedback->message, true);
          } else {
            SetState("RUNNING", feedback->state + ": " + feedback->message, true);
          }
        };

    mission_active_ = true;
    autostart_sent_ = true;
    SetState("STARTING", "sending mission profile to navigate sequence", true);
    action_client_->async_send_goal(goal, options);
    if (response != nullptr) {
      response->success = true;
      response->message = accepted_message;
    }
  }

  bool ApplyMissionResultDecision(
      const robot_tasks::MissionResultWorkflowDecision& result_decision,
      const std::string& completed_mission_id, const bool completed_dock) {
    if (result_decision.clear_cancellation_requested) {
      cancellation_requested_ = false;
    }
    if (result_decision.mark_docking_terminal) {
      MarkDockingMissionTerminal(
          completed_mission_id, result_decision.docking_terminal_state,
          result_decision.docking_terminal_message);
    }
    if (result_decision.release_all_resources) {
      ReleaseAllResourcesForMission(completed_mission_id);
    } else if (result_decision.release_route_locks) {
      ReleaseRouteLocksForMission(completed_mission_id);
    }

    if (result_decision.probe_docking_contact_failure) {
      if (HandleDockingContactFailure(completed_mission_id)) {
        HandleMissionFailure("docking contact failed", false);
        return false;
      }
    }
    if (result_decision.reset_failure_retry_count) {
      current_failure_retry_count_ = 0;
    }
    if (result_decision.mark_resource_occupied) {
      MarkResourceOccupiedForMission(completed_mission_id);
    }
    if (result_decision.complete_docking) {
      CompleteDockingMission(completed_mission_id);
    }

    if (result_decision.handle_failure) {
      HandleMissionFailure(result_decision.failure_message, completed_dock);
    } else {
      SetState(result_decision.state, result_decision.state_message, result_decision.recoverable);
    }

    return true;
  }

  void ApplyMissionResultPostActions(
      const robot_tasks::MissionResultWorkflowDecision& result_decision,
      const bool completed_dock,
      const std::optional<robot_tasks::MissionProfile>& dock_profile) {
    robot_tasks::MissionResultPostActionRequest request;
    request.mission_succeeded = result_decision.mission_succeeded;
    request.return_to_dock_after_mission = return_to_dock_after_mission_;
    request.completed_dock = completed_dock;
    request.mission_queue_empty = mission_queue_.empty();
    request.dock_profile_available = dock_profile.has_value();
    request.existing_queue_start_requested = queue_start_requested_;
    request.auto_start_queue = auto_start_queue_;
    request.queue_running = queue_running_;
    const auto post_actions = robot_tasks::PlanMissionResultPostActions(request);
    if (post_actions.queue_dock_after_result) {
      QueueDockMission(*dock_profile, "mission queue completed");
    }
    queue_start_requested_ = post_actions.queue.queue_start_requested;
    queue_running_ = post_actions.queue.queue_running;
  }

  void PauseMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (!mission_active_) {
      response->success = false;
      response->message = "no active mission profile to pause";
      return;
    }
    paused_ = true;
    SetState("PAUSED", "mission profile pause requested", true);
    SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
    response->success = true;
    response->message = "mission profile pause requested";
  }

  void ResumeMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (!mission_active_ || !paused_) {
      response->success = false;
      response->message = "no paused mission profile to resume";
      return;
    }
    paused_ = false;
    SetState("RUNNING", "mission profile resume requested", true);
    SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
    response->success = true;
    response->message = "mission profile resume requested";
  }

  void CancelMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (!RunnerBusy()) {
      response->success = false;
      response->message = "no active mission profile to cancel";
      return;
    }
    RequestActiveMissionCancel("mission profile cancellation requested");
    response->success = true;
    response->message = "mission profile cancellation requested";
  }

  template <typename ResponseT>
  void GetMissionRecoveryState(const std::shared_ptr<ResponseT>& response) const {
    response->success = true;
    response->message = "mission recovery state";
    response->enabled = failure_recovery_enabled_;
    response->policy = failure_recovery_policy_;
    response->retry_limit = std::max(0, failure_retry_limit_);
    response->retry_count = current_failure_retry_count_;
    response->last_failed_mission_id = last_failed_mission_id_;
    response->last_failure_message = last_failure_message_;
    response->last_recovery_action = last_recovery_action_;
  }

  template <typename RequestT, typename ResponseT>
  void ListMissionEvents(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) const {
    const auto limit = robot_tasks::NormalizeEventLimit(request->limit, 20U, 200U);
    const auto events = robot_tasks::SelectRecentMissionEvents(
        mission_events_, limit, request->state_filter, request->mission_id_filter);
    for (const auto& event : events) {
      response->stamp.push_back(event.stamp);
      response->mission_ids.push_back(event.mission_id);
      response->states.push_back(event.state);
      response->previous_states.push_back(event.previous_state);
      response->messages.push_back(event.message);
      response->recoverable.push_back(event.recoverable);
    }
    response->success = true;
    response->message = "loaded " + std::to_string(events.size()) + " mission event(s)";
  }

  void ResetMissionRecoveryState(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const auto* transport = FindActiveStationTransport();
    if (transport != nullptr &&
        transport->state.phase ==
            robot_tasks::StationTransportPhase::kNeedsOperator) {
      response->success = false;
      response->message =
          "clear the physical payload with /v2/set_payload_loaded before resetting recovery";
      return;
    }
    current_failure_retry_count_ = 0;
    active_recovery_mission_id_.clear();
    last_failed_mission_id_.clear();
    last_failure_message_.clear();
    last_recovery_action_.clear();
    if (current_state_ == "NEEDS_OPERATOR") {
      SetState("READY", "mission recovery state reset", true);
    }
    response->success = true;
    response->message = "mission recovery state reset";
  }

  void MarkResourceOccupiedForMission(const std::string& mission_id) {
    robot_tasks::MarkResourceOccupiedForMission(
        facility_reservations_, facility_resource_by_mission_, mission_id);
  }

  void ReleaseResourceForMission(const std::string& mission_id) {
    robot_tasks::ReleaseResourceForMission(
        facility_reservations_, facility_resource_by_mission_, mission_id);
  }

  void ReleaseAllResourcesForMission(const std::string& mission_id) {
    robot_tasks::TickMissionResourceCleanupBehaviorTree(
        robot_tasks::MissionResourceCleanupBehaviorTreeInput{
            facility_reservations_, facility_resource_by_mission_, route_locks_,
            route_locks_by_mission_, mission_id});
  }

  void CompleteDockingMission(const std::string& mission_id) {
    const auto transition = robot_tasks::CompleteDockingSubflow(
        dock_states_, dock_by_mission_, facility_reservations_, facility_resource_by_mission_,
        mission_id);
    if (!transition.changed) {
      return;
    }
    RecordMissionEventFor(
        mission_id, transition.event_state, transition.previous_state,
        transition.event_message, transition.event_recoverable);
  }

  bool HandleDockingContactFailure(const std::string& mission_id) {
    const auto transition = robot_tasks::StageDockingContactRetry(
        dock_states_, dock_by_mission_, facility_reservations_, facility_resource_by_mission_,
        mission_id);
    if (!transition.changed) {
      return false;
    }
    RecordMissionEventFor(
        mission_id, transition.event_state, transition.previous_state,
        transition.event_message, transition.event_recoverable);
    return true;
  }

  void MarkDockingMissionTerminal(
      const std::string& mission_id, const std::string& state, const std::string& message) {
    const auto transition = robot_tasks::MarkDockingSubflowTerminal(
        dock_states_, dock_by_mission_, mission_id, state, message);
    if (!transition.changed) {
      return;
    }
    RecordMissionEventFor(
        mission_id, transition.event_state, transition.previous_state,
        transition.event_message, transition.event_recoverable);
  }

  void HandleMissionFailure(const std::string& failure_message, const bool completed_dock) {
    const std::string mission_id = profile_.has_value() ? profile_->mission_id : "";
    last_failed_mission_id_ = mission_id;
    last_failure_message_ = failure_message;

    robot_tasks::MissionRecoveryConfig config;
    config.enabled = failure_recovery_enabled_;
    config.policy = failure_recovery_policy_;
    config.retry_limit = std::max(0, failure_retry_limit_);

    robot_tasks::MissionRecoveryBehaviorTreeInput input;
    input.config = config;
    input.retry_count = current_failure_retry_count_;
    input.is_dock_mission = completed_dock;
    input.mission_id = mission_id;
    input.failure_message = failure_message;
    input.mission_file = mission_file_;
    input.recovery_priority = failure_recovery_priority_;
    input.next_queue_sequence = next_queue_sequence_;
    if (profile_.has_value()) {
      input.active_profile = *profile_;
    }

    const auto recovery = robot_tasks::TickMissionRecoveryBehaviorTree(input);
    current_failure_retry_count_ = recovery.retry_count;
    next_queue_sequence_ = recovery.next_queue_sequence;
    last_recovery_action_ = recovery.decision.action;

    if (recovery.retry_mission.has_value()) {
      mission_queue_.push_back(*recovery.retry_mission);
      SortMissionQueue();
      queue_running_ = recovery.queue_running;
      queue_start_requested_ = recovery.queue_start_requested;
      SetState(recovery.state, recovery.state_message, recovery.recoverable);
      return;
    }
    if (recovery.request_dock_return) {
      SetState(recovery.state, recovery.state_message, recovery.recoverable);
      RequestDockReturn("mission failure recovery for " + mission_id, nullptr);
      return;
    }
    if (recovery.stop_queue) {
      queue_running_ = false;
      queue_start_requested_ = false;
      SetState(recovery.state, recovery.state_message, recovery.recoverable);
      return;
    }
    SetState(recovery.state, recovery.state_message, recovery.recoverable);
  }

  void RequestActiveMissionCancel(const std::string& message) {
    auto* transport = FindActiveStationTransport();
    if (transport != nullptr && !mission_active_) {
      const std::string mission_id = transport->state.mission_id;
      const auto transition =
          robot_tasks::PlanStationTransportCancel(transport->state);
      if (!transition.accepted) {
        SetState("ERROR", transition.message, false);
        return;
      }
      transport->state = transition.next_state;
      if (transition.release_resources) {
        ReleaseAllResourcesForMission(mission_id);
        FinishStationTransportTerminal(
            mission_id, "CANCELED", transition.message, false);
      } else {
        SetState("NEEDS_OPERATOR", transition.message, false);
      }
      return;
    }

    cancellation_requested_ = true;
    paused_ = false;
    SetState("CANCELING", message, true);
    if (current_goal_handle_) {
      action_client_->async_cancel_goal(current_goal_handle_);
    }
    SendControlRequest<std_srvs::srv::Trigger>(cancel_client_, cancel_service_name_);
  }

  template <typename ServiceT>
  void SendControlRequest(const typename rclcpp::Client<ServiceT>::SharedPtr& client,
                          const std::string& service_name) {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "control service %s is not ready", service_name.c_str());
      return;
    }
    client->async_send_request(std::make_shared<typename ServiceT::Request>());
  }

  void Tick() {
    robot_tasks::MissionRuntimeBehaviorTreeInput input;
    input.tick.autostart = autostart_;
    input.tick.autostart_sent = autostart_sent_;
    input.tick.mission_active = RunnerBusy();
    input.tick.queue_start_requested = queue_start_requested_;
    const auto runtime_tick = robot_tasks::TickMissionRuntimeBehaviorTree(input);
    if (!runtime_tick.success) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "mission runtime behavior tree failed");
    }
    if (runtime_tick.success && runtime_tick.decision.start_default_mission) {
      StartMission(nullptr);
    }
    if (runtime_tick.success && runtime_tick.decision.start_queued_mission &&
        !RunnerBusy()) {
      if (runtime_tick.decision.clear_queue_start_request) {
        queue_start_requested_ = false;
      }
      StartNextQueuedMission(nullptr);
    }
    PublishState();
    PublishPayloadState();
    PublishLocalizationHealth();
    PublishSafetyState();
    ExpireStationConfirmations();
  }

  void SetState(const std::string& state, const std::string& message, const bool recoverable) {
    previous_state_ = current_state_;
    current_state_ = state;
    last_message_ = message;
    recoverable_ = recoverable;
    RecordMissionEvent();
    PublishState();
  }

  void RecordMissionEventFor(const std::string& mission_id, const std::string& state,
                             const std::string& previous_state, const std::string& message,
                             const bool recoverable) {
    robot_tasks::AppendMissionEvent(
        mission_events_,
        robot_tasks::MakeMissionEvent(
            std::to_string(now().seconds()), mission_id, state, previous_state, message,
            recoverable),
        500U);
  }

  void RecordMissionEvent() {
    robot_tasks::AppendMissionEvent(
        mission_events_,
        robot_tasks::MakeMissionEvent(
            std::to_string(now().seconds()),
            profile_.has_value() ? profile_->mission_id : "", current_state_, previous_state_,
            last_message_, recoverable_),
        500U);
  }

  void PublishState() {
    robot_interfaces::msg::RobotState state;
    state.header.stamp = now();
    state.state = current_state_;
    state.previous_state = previous_state_;
    state.active_goal_id = profile_.has_value() ? profile_->mission_id : "";
    state.message = last_message_;
    state.recoverable = recoverable_;
    state_pub_->publish(state);
  }

  void PublishPayloadState() {
    robot_interfaces::msg::PayloadState state;
    state.header.stamp = now();
    state.loaded = payload_loaded_;
    state.payload_id = payload_id_;
    state.weight_kg = payload_weight_kg_;
    state.capacity_kg = payload_capacity_kg_;
    state.top_module_type = top_module_type_;
    state.state = payload_state_;
    state.last_action = payload_last_action_;
    state.message = payload_message_;
    payload_state_pub_->publish(state);
  }

  void PublishLocalizationHealth() {
    robot_interfaces::msg::LocalizationHealth state;
    state.header.stamp = now();
    state.localized = localization_localized_;
    state.state = localization_state_;
    state.covariance_xy = localization_covariance_xy_;
    state.covariance_threshold = localization_covariance_threshold_;
    state.station_id = localization_station_id_;
    state.message = localization_message_;
    localization_health_pub_->publish(state);
  }

  void PublishSafetyState() {
    robot_interfaces::msg::SafetyState state;
    state.header.stamp = now();
    state.obstacle_blocked = safety_obstacle_blocked_;
    state.safety_stop = safety_obstacle_blocked_;
    state.state = SafetyStateName();
    state.runtime_speed_limit_mps = safety_runtime_speed_limit_mps_;
    state.zone_speed_limit_mps = safety_zone_speed_limit_mps_;
    state.effective_speed_limit_mps = EffectiveSafetySpeedLimit();
    state.stop_reason = safety_stop_reason_;
    state.blockage_id = safety_blockage_id_;
    state.mission_id = safety_mission_id_;
    state.message = safety_message_;
    safety_state_pub_->publish(state);
  }

  std::string mission_file_;
  std::string dock_mission_file_;
  std::string station_file_;
  std::string facility_file_;
  std::string dock_file_;
  std::string fleet_file_;
  std::string scenario_file_;
  std::string business_order_file_;
  std::string robot_id_;
  std::string top_module_type_;
  std::string preflight_zones_file_;
  std::string preflight_map_name_;
  std::string action_name_;
  std::string pause_service_name_;
  std::string resume_service_name_;
  std::string cancel_service_name_;
  bool auto_start_queue_ = true;
  bool return_to_dock_after_mission_ = false;
  bool return_to_dock_on_low_battery_ = true;
  bool autostart_ = false;
  bool autostart_sent_ = false;
  bool mission_active_ = false;
  bool paused_ = false;
  bool cancellation_requested_ = false;
  bool queue_running_ = false;
  bool queue_start_requested_ = false;
  bool low_battery_dock_request_sent_ = false;
  bool recoverable_ = true;
  bool failure_recovery_enabled_ = true;
  bool preflight_enabled_ = true;
  bool preflight_require_station_route_ = true;
  bool estimate_use_nav2_path_ = true;
  bool opportunity_charge_require_idle_queue_ = true;
  int current_goal_index_ = 0;
  int dock_priority_ = 100;
  int server_timeout_ms_ = 5000;
  int estimate_nav2_server_timeout_ms_ = 50;
  int estimate_nav2_result_timeout_ms_ = 1500;
  int failure_retry_limit_ = 1;
  int failure_recovery_priority_ = 90;
  int opportunity_charge_priority_ = 75;
  double dock_low_battery_voltage_ = 22.0;
  double opportunity_charge_threshold_voltage_ = 23.2;
  double critical_charge_threshold_voltage_ = 22.0;
  double estimate_nominal_speed_mps_ = 0.35;
  double estimate_battery_drop_per_meter_ = 0.015;
  double estimate_min_battery_voltage_ = 22.0;
  double preflight_default_speed_limit_mps_ = 0.6;
  double current_battery_voltage_ = 0.0;
  double payload_capacity_kg_ = 30.0;
  double payload_weight_kg_ = 0.0;
  double localization_covariance_threshold_ = 0.5;
  double localization_covariance_xy_ = 0.0;
  double safety_runtime_speed_limit_mps_ = 0.0;
  double safety_zone_speed_limit_mps_ = 0.6;
  std::uint64_t next_queue_sequence_ = 0;
  std::uint64_t next_transport_order_sequence_ = 0;
  std::uint64_t next_station_order_sequence_ = 0;
  std::uint64_t next_station_sequence_order_sequence_ = 0;
  std::uint64_t next_charging_request_sequence_ = 0;
  std::uint64_t next_docking_request_sequence_ = 0;
  std::uint64_t next_fleet_task_sequence_ = 0;
  std::string current_state_ = "INITIALIZING";
  std::string previous_state_ = "UNKNOWN";
  std::string last_message_ = "initializing";
  std::string failure_recovery_policy_ = "retry_then_dock";
  std::string estimate_nav2_compute_path_action_ = "/compute_path_to_pose";
  std::string estimate_nav2_planner_id_;
  std::string active_recovery_mission_id_;
  std::string last_failed_mission_id_;
  std::string last_failure_message_;
  std::string last_recovery_action_;
  std::string payload_id_;
  std::string payload_state_ = "EMPTY";
  std::string payload_last_action_ = "none";
  std::string payload_message_ = "payload empty";
  std::string localization_state_ = "UNKNOWN";
  std::string localization_station_id_;
  std::string localization_message_ = "waiting for localization";
  std::string safety_stop_reason_;
  std::string safety_blockage_id_;
  std::string safety_mission_id_;
  std::string safety_message_ = "safety ok";
  int current_failure_retry_count_ = 0;
  bool payload_loaded_ = false;
  bool localization_localized_ = false;
  bool localization_pause_on_lost_ = true;
  bool localization_paused_mission_ = false;
  bool safety_obstacle_blocked_ = false;
  bool safety_obstacle_pause_on_blockage_ = true;
  bool safety_blockage_paused_mission_ = false;
  bool traffic_intersection_locks_enabled_ = true;
  std::optional<robot_tasks::MissionProfile> profile_;
  std::vector<QueuedMission> mission_queue_;
  std::unordered_map<std::string, StationTransportExecutionRecord>
      station_transport_executions_;
  std::string active_station_transport_id_;
  std::vector<robot_tasks::MissionEvent> mission_events_;
  std::unordered_set<std::string> paused_workflow_orders_;
  std::unordered_map<std::string, robot_tasks::RuntimeFleetRobotState> runtime_fleet_state_;
  std::unordered_map<std::string, RemoteTaskState> remote_tasks_;
  robot_tasks::FacilityReservationMap facility_reservations_;
  robot_tasks::MissionResourceMap facility_resource_by_mission_;
  std::unordered_map<std::string, StationConfirmation> station_confirmations_;
  std::unordered_map<std::string, FulfillmentOrder> fulfillment_orders_;
  robot_tasks::DockingStateMap dock_states_;
  robot_tasks::DockingMissionMap dock_by_mission_;
  robot_tasks::RouteLockMap route_locks_;
  robot_tasks::RouteLockReservationMap route_locks_by_mission_;
  GoalHandleNavigateSequence::SharedPtr current_goal_handle_;
  rclcpp_action::Client<NavigateSequence>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr nav2_path_node_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr nav2_path_client_;
  mutable std::mutex nav2_path_mutex_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
  rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<robot_interfaces::msg::PayloadState>::SharedPtr payload_state_pub_;
  rclcpp::Publisher<robot_interfaces::msg::LocalizationHealth>::SharedPtr
      localization_health_pub_;
  rclcpp::Publisher<robot_interfaces::msg::SafetyState>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<robot_interfaces::msg::ChassisState>::SharedPtr chassis_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::GetMissionRecoveryState>::SharedPtr
      v2_recovery_state_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ListMissionEvents>::SharedPtr
      v2_list_mission_events_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_recovery_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::EnqueueMission>::SharedPtr v2_enqueue_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::CancelQueuedMission>::SharedPtr
      v2_cancel_queued_srv_;
  rclcpp::Service<robot_interfaces_business::srv::CancelWorkflow>::SharedPtr
      v2_cancel_workflow_srv_;
  rclcpp::Service<robot_interfaces_business::srv::PauseWorkflow>::SharedPtr
      v2_pause_workflow_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ResumeWorkflow>::SharedPtr
      v2_resume_workflow_srv_;
  rclcpp::Service<robot_interfaces_business::srv::GetWorkflowStatus>::SharedPtr
      v2_get_workflow_status_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ReprioritizeQueuedMission>::SharedPtr
      v2_reprioritize_queued_mission_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ReprioritizeBatch>::SharedPtr
      v2_reprioritize_batch_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::PreemptMission>::SharedPtr v2_preempt_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::EstimateMissionCost>::SharedPtr
      v2_estimate_mission_srv_;
  rclcpp::Service<robot_interfaces_business::srv::SubmitOrder>::SharedPtr
      v2_submit_unified_order_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ListStations>::SharedPtr v2_list_stations_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ListStationRoutes>::SharedPtr
      v2_list_station_routes_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::BlockStationRoute>::SharedPtr
      v2_block_station_route_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::UnblockStationRoute>::SharedPtr
      v2_unblock_station_route_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ListTrafficReservations>::SharedPtr
      v2_list_traffic_reservations_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ClearTrafficReservation>::SharedPtr
      v2_clear_traffic_reservation_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::DetectTrafficDeadlock>::SharedPtr
      v2_detect_traffic_deadlock_srv_;
  rclcpp::Service<robot_interfaces_fleet::srv::ListFleetRobots>::SharedPtr
      v2_list_fleet_robots_srv_;
  rclcpp::Service<robot_interfaces_fleet::srv::UpdateFleetRobotState>::SharedPtr
      v2_update_fleet_robot_state_srv_;
  rclcpp::Service<robot_interfaces_fleet::srv::UpdateRemoteTaskState>::SharedPtr
      v2_update_remote_task_state_srv_;
  rclcpp::Service<robot_interfaces_fleet::srv::PreviewFleetStationTask>::SharedPtr
      v2_preview_fleet_station_task_srv_;
  rclcpp::Service<robot_interfaces_fleet::srv::SubmitFleetStationTask>::SharedPtr
      v2_submit_fleet_station_task_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ListBusinessOrderTypes>::SharedPtr
      v2_list_business_order_types_srv_;
  rclcpp::Service<robot_interfaces_business::srv::GetOperatorSnapshot>::SharedPtr
      v2_get_operator_snapshot_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ListScenarioTasks>::SharedPtr
      v2_list_scenario_tasks_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ListScenarioWorkflows>::SharedPtr
      v2_list_scenario_workflows_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ListFacilityResources>::SharedPtr
      v2_list_facility_resources_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ReserveFacilityResource>::SharedPtr
      v2_reserve_facility_resource_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ReleaseFacilityResource>::SharedPtr
      v2_release_facility_resource_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::RequestLiftSession>::SharedPtr
      v2_request_lift_session_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ReleaseLiftSession>::SharedPtr
      v2_release_lift_session_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::GetLiftState>::SharedPtr
      v2_get_lift_state_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::GetDoorState>::SharedPtr
      v2_get_door_state_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::ListDocks>::SharedPtr v2_list_docks_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestDocking>::SharedPtr
      v2_request_docking_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestUndocking>::SharedPtr
      v2_request_undocking_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::GetDockState>::SharedPtr
      v2_get_dock_state_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ConfirmLoad>::SharedPtr
      v2_confirm_load_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ConfirmUnload>::SharedPtr
      v2_confirm_unload_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::SetPayloadLoaded>::SharedPtr
      v2_set_payload_loaded_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ExecuteTopModuleAction>::SharedPtr
      v2_execute_top_module_action_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::RequestRelocalization>::SharedPtr
      v2_request_relocalization_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::SetInitialPoseFromStation>::SharedPtr
      v2_set_initial_pose_from_station_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>::SharedPtr
      v2_set_dynamic_speed_limit_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::ReportObstacleBlockage>::SharedPtr
      v2_report_obstacle_blockage_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::ClearObstacleBlockage>::SharedPtr
      v2_clear_obstacle_blockage_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestCharging>::SharedPtr
      v2_request_charging_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestOpportunityCharging>::SharedPtr
      v2_request_opportunity_charging_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ExecuteFacilityAction>::SharedPtr
      v2_execute_facility_action_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::RequestStationConfirmation>::SharedPtr
      v2_request_station_confirmation_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::GetPendingConfirmations>::SharedPtr
      v2_get_pending_confirmations_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::ConfirmStationAction>::SharedPtr
      v2_confirm_station_action_srv_;
  rclcpp::Service<robot_interfaces_facility::srv::RejectStationAction>::SharedPtr
      v2_reject_station_action_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ConfirmPick>::SharedPtr v2_confirm_pick_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ConfirmDropoff>::SharedPtr
      v2_confirm_dropoff_srv_;
  rclcpp::Service<robot_interfaces_business::srv::ReportShortPick>::SharedPtr
      v2_report_short_pick_srv_;
  rclcpp::Service<robot_interfaces_business::srv::GetOrderFulfillmentStatus>::SharedPtr
      v2_get_order_fulfillment_status_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::SubmitStationTransportOrder>::SharedPtr
      v2_submit_station_order_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::SubmitStationOrderBatch>::SharedPtr
      v2_submit_station_order_batch_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::CancelStationOrderBatch>::SharedPtr
      v2_cancel_station_order_batch_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::SubmitStationSequenceTask>::SharedPtr
      v2_submit_station_sequence_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::EstimateStationTransport>::SharedPtr
      v2_estimate_station_transport_srv_;
  rclcpp::Service<robot_interfaces_mission::srv::EstimateStationSequence>::SharedPtr
      v2_estimate_station_sequence_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_to_dock_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
