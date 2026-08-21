from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_file = LaunchConfiguration("mission_file")
    dock_mission_file = LaunchConfiguration("dock_mission_file")
    station_file = LaunchConfiguration("station_file")
    facility_file = LaunchConfiguration("facility_file")
    dock_file = LaunchConfiguration("dock_file")
    fleet_file = LaunchConfiguration("fleet_file")
    scenario_file = LaunchConfiguration("scenario_file")
    business_order_file = LaunchConfiguration("business_order_file")
    robot_id = LaunchConfiguration("robot_id")
    preflight_zones_file = LaunchConfiguration("preflight_zones_file")
    preflight_map_name = LaunchConfiguration("preflight_map_name")
    autostart = LaunchConfiguration("autostart")
    server_timeout_ms = LaunchConfiguration("server_timeout_ms")
    action_name = LaunchConfiguration("action_name")
    pause_service_name = LaunchConfiguration("pause_service_name")
    resume_service_name = LaunchConfiguration("resume_service_name")
    cancel_service_name = LaunchConfiguration("cancel_service_name")
    auto_start_queue = LaunchConfiguration("auto_start_queue")
    return_to_dock_after_mission = LaunchConfiguration("return_to_dock_after_mission")
    return_to_dock_on_low_battery = LaunchConfiguration("return_to_dock_on_low_battery")
    dock_priority = LaunchConfiguration("dock_priority")
    dock_low_battery_voltage = LaunchConfiguration("dock_low_battery_voltage")
    opportunity_charge_threshold_voltage = LaunchConfiguration(
        "opportunity_charge_threshold_voltage"
    )
    critical_charge_threshold_voltage = LaunchConfiguration("critical_charge_threshold_voltage")
    opportunity_charge_priority = LaunchConfiguration("opportunity_charge_priority")
    opportunity_charge_require_idle_queue = LaunchConfiguration(
        "opportunity_charge_require_idle_queue"
    )
    estimate_nominal_speed_mps = LaunchConfiguration("estimate_nominal_speed_mps")
    estimate_battery_drop_per_meter = LaunchConfiguration("estimate_battery_drop_per_meter")
    estimate_min_battery_voltage = LaunchConfiguration("estimate_min_battery_voltage")
    estimate_use_nav2_path = LaunchConfiguration("estimate_use_nav2_path")
    estimate_nav2_compute_path_action = LaunchConfiguration("estimate_nav2_compute_path_action")
    estimate_nav2_planner_id = LaunchConfiguration("estimate_nav2_planner_id")
    estimate_nav2_server_timeout_ms = LaunchConfiguration("estimate_nav2_server_timeout_ms")
    estimate_nav2_result_timeout_ms = LaunchConfiguration("estimate_nav2_result_timeout_ms")
    failure_recovery_enabled = LaunchConfiguration("failure_recovery_enabled")
    failure_recovery_policy = LaunchConfiguration("failure_recovery_policy")
    failure_retry_limit = LaunchConfiguration("failure_retry_limit")
    failure_recovery_priority = LaunchConfiguration("failure_recovery_priority")
    preflight_enabled = LaunchConfiguration("preflight_enabled")
    preflight_require_station_route = LaunchConfiguration("preflight_require_station_route")
    preflight_default_speed_limit_mps = LaunchConfiguration("preflight_default_speed_limit_mps")
    traffic_intersection_locks_enabled = LaunchConfiguration("traffic_intersection_locks_enabled")
    payload_capacity_kg = LaunchConfiguration("payload_capacity_kg")
    top_module_type = LaunchConfiguration("top_module_type")
    localization_covariance_threshold = LaunchConfiguration("localization_covariance_threshold")
    localization_pause_on_lost = LaunchConfiguration("localization_pause_on_lost")
    safety_zone_speed_limit_mps = LaunchConfiguration("safety_zone_speed_limit_mps")
    safety_obstacle_pause_on_blockage = LaunchConfiguration("safety_obstacle_pause_on_blockage")
    default_mission_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "missions", "demo_patrol.yaml"]
    )
    default_dock_mission_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "missions", "dock_return.yaml"]
    )
    default_station_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "stations", "warehouse_stations.yaml"]
    )
    default_facility_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "facilities", "warehouse_facilities.yaml"]
    )
    default_dock_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "docks", "docks.yaml"]
    )
    default_fleet_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "fleet", "warehouse_fleet.yaml"]
    )
    default_scenario_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "scenarios", "industry_scenarios.yaml"]
    )
    default_business_order_file = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "config", "business_orders", "business_orders.yaml"]
    )
    default_preflight_zones_file = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "map_zones.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mission_file", default_value=default_mission_file),
            DeclareLaunchArgument("dock_mission_file", default_value=default_dock_mission_file),
            DeclareLaunchArgument("station_file", default_value=default_station_file),
            DeclareLaunchArgument("facility_file", default_value=default_facility_file),
            DeclareLaunchArgument("dock_file", default_value=default_dock_file),
            DeclareLaunchArgument("fleet_file", default_value=default_fleet_file),
            DeclareLaunchArgument("scenario_file", default_value=default_scenario_file),
            DeclareLaunchArgument("business_order_file", default_value=default_business_order_file),
            DeclareLaunchArgument("robot_id", default_value="robot_1"),
            DeclareLaunchArgument("preflight_zones_file", default_value=default_preflight_zones_file),
            DeclareLaunchArgument("preflight_map_name", default_value="indoor_room"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("server_timeout_ms", default_value="5000"),
            DeclareLaunchArgument("action_name", default_value="/navigate_sequence"),
            DeclareLaunchArgument("pause_service_name", default_value="/pause_mission"),
            DeclareLaunchArgument("resume_service_name", default_value="/resume_mission"),
            DeclareLaunchArgument("cancel_service_name", default_value="/cancel_mission"),
            DeclareLaunchArgument("auto_start_queue", default_value="true"),
            DeclareLaunchArgument("return_to_dock_after_mission", default_value="false"),
            DeclareLaunchArgument("return_to_dock_on_low_battery", default_value="true"),
            DeclareLaunchArgument("dock_priority", default_value="100"),
            DeclareLaunchArgument("dock_low_battery_voltage", default_value="22.0"),
            DeclareLaunchArgument("opportunity_charge_threshold_voltage", default_value="23.2"),
            DeclareLaunchArgument("critical_charge_threshold_voltage", default_value="22.0"),
            DeclareLaunchArgument("opportunity_charge_priority", default_value="75"),
            DeclareLaunchArgument("opportunity_charge_require_idle_queue", default_value="true"),
            DeclareLaunchArgument("estimate_nominal_speed_mps", default_value="0.35"),
            DeclareLaunchArgument("estimate_battery_drop_per_meter", default_value="0.015"),
            DeclareLaunchArgument("estimate_min_battery_voltage", default_value="22.0"),
            DeclareLaunchArgument("estimate_use_nav2_path", default_value="true"),
            DeclareLaunchArgument(
                "estimate_nav2_compute_path_action", default_value="/compute_path_to_pose"
            ),
            DeclareLaunchArgument("estimate_nav2_planner_id", default_value=""),
            DeclareLaunchArgument("estimate_nav2_server_timeout_ms", default_value="50"),
            DeclareLaunchArgument("estimate_nav2_result_timeout_ms", default_value="1500"),
            DeclareLaunchArgument("failure_recovery_enabled", default_value="true"),
            DeclareLaunchArgument("failure_recovery_policy", default_value="retry_then_dock"),
            DeclareLaunchArgument("failure_retry_limit", default_value="1"),
            DeclareLaunchArgument("failure_recovery_priority", default_value="90"),
            DeclareLaunchArgument("preflight_enabled", default_value="true"),
            DeclareLaunchArgument("preflight_require_station_route", default_value="true"),
            DeclareLaunchArgument("preflight_default_speed_limit_mps", default_value="0.6"),
            DeclareLaunchArgument("traffic_intersection_locks_enabled", default_value="true"),
            DeclareLaunchArgument("payload_capacity_kg", default_value="30.0"),
            DeclareLaunchArgument("top_module_type", default_value="manual_load"),
            DeclareLaunchArgument("localization_covariance_threshold", default_value="0.5"),
            DeclareLaunchArgument("localization_pause_on_lost", default_value="true"),
            DeclareLaunchArgument("safety_zone_speed_limit_mps", default_value="0.6"),
            DeclareLaunchArgument("safety_obstacle_pause_on_blockage", default_value="true"),
            Node(
                package="robot_tasks",
                executable="mission_runner_node",
                parameters=[
                    {
                        "mission_file": mission_file,
                        "dock_mission_file": dock_mission_file,
                        "station_file": station_file,
                        "facility_file": facility_file,
                        "dock_file": dock_file,
                        "fleet_file": fleet_file,
                        "scenario_file": scenario_file,
                        "business_order_file": business_order_file,
                        "robot_id": robot_id,
                        "preflight_zones_file": preflight_zones_file,
                        "preflight_map_name": preflight_map_name,
                        "autostart": autostart,
                        "server_timeout_ms": server_timeout_ms,
                        "action_name": action_name,
                        "pause_service_name": pause_service_name,
                        "resume_service_name": resume_service_name,
                        "cancel_service_name": cancel_service_name,
                        "auto_start_queue": auto_start_queue,
                        "return_to_dock_after_mission": return_to_dock_after_mission,
                        "return_to_dock_on_low_battery": return_to_dock_on_low_battery,
                        "dock_priority": dock_priority,
                        "dock_low_battery_voltage": dock_low_battery_voltage,
                        "opportunity_charge_threshold_voltage": opportunity_charge_threshold_voltage,
                        "critical_charge_threshold_voltage": critical_charge_threshold_voltage,
                        "opportunity_charge_priority": opportunity_charge_priority,
                        "opportunity_charge_require_idle_queue": opportunity_charge_require_idle_queue,
                        "estimate_nominal_speed_mps": estimate_nominal_speed_mps,
                        "estimate_battery_drop_per_meter": estimate_battery_drop_per_meter,
                        "estimate_min_battery_voltage": estimate_min_battery_voltage,
                        "estimate_use_nav2_path": estimate_use_nav2_path,
                        "estimate_nav2_compute_path_action": estimate_nav2_compute_path_action,
                        "estimate_nav2_planner_id": estimate_nav2_planner_id,
                        "estimate_nav2_server_timeout_ms": estimate_nav2_server_timeout_ms,
                        "estimate_nav2_result_timeout_ms": estimate_nav2_result_timeout_ms,
                        "failure_recovery_enabled": failure_recovery_enabled,
                        "failure_recovery_policy": failure_recovery_policy,
                        "failure_retry_limit": failure_retry_limit,
                        "failure_recovery_priority": failure_recovery_priority,
                        "preflight_enabled": preflight_enabled,
                        "preflight_require_station_route": preflight_require_station_route,
                        "preflight_default_speed_limit_mps": preflight_default_speed_limit_mps,
                        "traffic_intersection_locks_enabled": traffic_intersection_locks_enabled,
                        "payload_capacity_kg": payload_capacity_kg,
                        "top_module_type": top_module_type,
                        "localization_covariance_threshold": localization_covariance_threshold,
                        "localization_pause_on_lost": localization_pause_on_lost,
                        "safety_zone_speed_limit_mps": safety_zone_speed_limit_mps,
                        "safety_obstacle_pause_on_blockage": safety_obstacle_pause_on_blockage,
                    }
                ],
                output="screen",
            ),
        ]
    )
