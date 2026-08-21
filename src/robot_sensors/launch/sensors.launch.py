from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_input = LaunchConfiguration("scan_input")
    imu_input = LaunchConfiguration("imu_input")
    scan_clockwise = LaunchConfiguration("scan_clockwise")
    use_laser_filters = LaunchConfiguration("use_laser_filters")
    laser_filter_config = LaunchConfiguration("laser_filter_config")
    default_laser_filter_config = PathJoinSubstitution(
        [FindPackageShare("robot_sensors"), "config", "laser_filter_chain.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_input", default_value="/sim/scan"),
            DeclareLaunchArgument("imu_input", default_value="/sim/imu"),
            DeclareLaunchArgument("scan_clockwise", default_value="false"),
            DeclareLaunchArgument("use_laser_filters", default_value="true"),
            DeclareLaunchArgument("laser_filter_config", default_value=default_laser_filter_config),
            Node(
                package="robot_sensors",
                executable="laser_filter_node",
                parameters=[
                    {
                        "input_topic": scan_input,
                        "output_topic": "/scan",
                        "clockwise": scan_clockwise,
                    }
                ],
                condition=UnlessCondition(use_laser_filters),
                output="screen",
            ),
            Node(
                package="robot_sensors",
                executable="laser_filter_node",
                parameters=[
                    {
                        "input_topic": scan_input,
                        "output_topic": "/scan_normalized",
                        "clockwise": scan_clockwise,
                        "enable_builtin_filter": False,
                    }
                ],
                condition=IfCondition(use_laser_filters),
                output="screen",
            ),
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                parameters=[laser_filter_config],
                remappings=[
                    ("scan", "/scan_normalized"),
                    ("scan_filtered", "/scan"),
                ],
                condition=IfCondition(use_laser_filters),
                output="screen",
            ),
            Node(
                package="robot_sensors",
                executable="imu_filter_node",
                parameters=[{"input_topic": imu_input, "output_topic": "/imu/data"}],
                output="screen",
            ),
        ]
    )
