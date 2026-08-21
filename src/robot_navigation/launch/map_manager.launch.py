from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    maps_directory = LaunchConfiguration("maps_directory")
    zones_file = LaunchConfiguration("zones_file")
    active_map = LaunchConfiguration("active_map")
    publish_period_ms = LaunchConfiguration("publish_period_ms")
    default_speed_limit_mps = LaunchConfiguration("default_speed_limit_mps")
    default_maps_directory = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "maps"]
    )
    default_zones_file = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "map_zones.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("maps_directory", default_value=default_maps_directory),
            DeclareLaunchArgument("zones_file", default_value=default_zones_file),
            DeclareLaunchArgument("active_map", default_value="indoor_room"),
            DeclareLaunchArgument("publish_period_ms", default_value="1000"),
            DeclareLaunchArgument("default_speed_limit_mps", default_value="0.6"),
            Node(
                package="robot_navigation",
                executable="map_manager_node",
                parameters=[
                    {
                        "maps_directory": maps_directory,
                        "zones_file": zones_file,
                        "active_map": active_map,
                        "publish_period_ms": publish_period_ms,
                        "default_speed_limit_mps": default_speed_limit_mps,
                    }
                ],
                output="screen",
            ),
        ]
    )
