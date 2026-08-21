from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_zone_filters = LaunchConfiguration("use_zone_filters")
    zone_filter_params_file = LaunchConfiguration("zone_filter_params_file")
    footprint_file = LaunchConfiguration("footprint_file")
    localization_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
    )
    navigation_launch = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "launch", "navigation_no_docking.launch.py"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "nav2_basic.yaml"]
    )
    default_map = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "maps", "indoor_room.yaml"]
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("navigation_start_delay", default_value="5.0"),
            DeclareLaunchArgument("use_zone_filters", default_value="false"),
            DeclareLaunchArgument(
                "zone_filter_params_file", default_value=default_zone_filter_params
            ),
            DeclareLaunchArgument("footprint_file", default_value=default_footprint_file),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "map": map_file,
                    "params_file": params,
                    "use_sim_time": use_sim_time,
                    "use_composition": "False",
                    "autostart": "true",
                }.items(),
            ),
            TimerAction(
                period=navigation_start_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch),
                        launch_arguments={
                            "params_file": params,
                            "use_sim_time": use_sim_time,
                            "autostart": "true",
                            "use_zone_filters": use_zone_filters,
                            "zone_filter_params_file": zone_filter_params_file,
                            "footprint_file": footprint_file,
                        }.items(),
                    )
                ],
            ),
        ]
    )
