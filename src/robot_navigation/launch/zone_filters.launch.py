from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keepout_mask = LaunchConfiguration("keepout_mask")
    speed_mask = LaunchConfiguration("speed_mask")
    keepout_mask_topic = LaunchConfiguration("keepout_mask_topic")
    speed_mask_topic = LaunchConfiguration("speed_mask_topic")
    keepout_filter_info_topic = LaunchConfiguration("keepout_filter_info_topic")
    speed_filter_info_topic = LaunchConfiguration("speed_filter_info_topic")
    mask_frame = LaunchConfiguration("mask_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    default_keepout_mask = PathJoinSubstitution(
        [
            FindPackageShare("robot_navigation"),
            "maps",
            "filters",
            "indoor_room_keepout_mask.yaml",
        ]
    )
    default_speed_mask = PathJoinSubstitution(
        [
            FindPackageShare("robot_navigation"),
            "maps",
            "filters",
            "indoor_room_speed_mask.yaml",
        ]
    )

    lifecycle_nodes = [
        "keepout_filter_mask_server",
        "speed_filter_mask_server",
        "keepout_costmap_filter_info_server",
        "speed_costmap_filter_info_server",
    ]
    common_arguments = ["--ros-args", "--log-level", log_level]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("keepout_mask", default_value=default_keepout_mask),
            DeclareLaunchArgument("speed_mask", default_value=default_speed_mask),
            DeclareLaunchArgument("keepout_mask_topic", default_value="/keepout_filter_mask"),
            DeclareLaunchArgument("speed_mask_topic", default_value="/speed_filter_mask"),
            DeclareLaunchArgument(
                "keepout_filter_info_topic", default_value="/costmap_filter_info/keepout"
            ),
            DeclareLaunchArgument(
                "speed_filter_info_topic", default_value="/costmap_filter_info/speed"
            ),
            DeclareLaunchArgument("mask_frame", default_value="map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            GroupAction(
                actions=[
                    SetParameter("use_sim_time", use_sim_time),
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="keepout_filter_mask_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": keepout_mask,
                                "topic_name": keepout_mask_topic,
                                "frame_id": mask_frame,
                            },
                        ],
                        arguments=common_arguments,
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="speed_filter_mask_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": speed_mask,
                                "topic_name": speed_mask_topic,
                                "frame_id": mask_frame,
                            },
                        ],
                        arguments=common_arguments,
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="costmap_filter_info_server",
                        name="keepout_costmap_filter_info_server",
                        output="screen",
                        parameters=[
                            {
                                "filter_info_topic": keepout_filter_info_topic,
                                "type": 0,
                                "mask_topic": keepout_mask_topic,
                                "base": 0.0,
                                "multiplier": 1.0,
                            },
                        ],
                        arguments=common_arguments,
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="costmap_filter_info_server",
                        name="speed_costmap_filter_info_server",
                        output="screen",
                        parameters=[
                            {
                                "filter_info_topic": speed_filter_info_topic,
                                "type": 2,
                                "mask_topic": speed_mask_topic,
                                "base": 0.0,
                                "multiplier": 0.01,
                            },
                        ],
                        arguments=common_arguments,
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_zone_filters",
                        output="screen",
                        parameters=[
                            {"autostart": autostart},
                            {"node_names": lifecycle_nodes},
                        ],
                        arguments=common_arguments,
                    ),
                ]
            ),
        ]
    )
