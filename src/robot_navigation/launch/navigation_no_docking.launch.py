from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def create_navigation_group(parameters, condition, leading_actions=None):
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    actions = []
    if leading_actions is not None:
        actions.extend(leading_actions)
    actions.extend(
        [
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "/nav2_cmd_vel")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "/nav2_cmd_vel")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
    return GroupAction(condition=condition, actions=actions)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    use_zone_filters = LaunchConfiguration("use_zone_filters")
    zone_filter_params_file = LaunchConfiguration("zone_filter_params_file")
    footprint_file = LaunchConfiguration("footprint_file")
    zone_filters_launch = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "launch", "zone_filters.launch.py"]
    )
    default_zone_filter_params = PathJoinSubstitution(
        [
            FindPackageShare("robot_navigation"),
            "maps",
            "filters",
            "indoor_room_nav2_costmap_filters.yaml",
        ]
    )
    default_footprint_file = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "robot_footprint.yaml"]
    )
    zone_filter_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zone_filters_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "log_level": log_level,
            }.items(),
        )
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_zone_filters", default_value="false"),
            DeclareLaunchArgument(
                "zone_filter_params_file", default_value=default_zone_filter_params
            ),
            DeclareLaunchArgument("footprint_file", default_value=default_footprint_file),
            create_navigation_group(
                [params_file, footprint_file], UnlessCondition(use_zone_filters)
            ),
            create_navigation_group(
                [params_file, zone_filter_params_file, footprint_file],
                IfCondition(use_zone_filters),
                zone_filter_actions,
            ),
        ]
    )
