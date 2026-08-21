from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    aggregator_config = LaunchConfiguration("aggregator_config")

    default_config = PathJoinSubstitution(
        [FindPackageShare("robot_utils"), "config", "diagnostic_aggregator.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("aggregator_config", default_value=default_config),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                parameters=[aggregator_config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
