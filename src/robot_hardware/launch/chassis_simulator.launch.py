from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bind_host = LaunchConfiguration("bind_host")
    port = LaunchConfiguration("port")
    battery_voltage = LaunchConfiguration("battery_voltage")
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("battery_voltage", default_value="24.0"),
            Node(
                package="robot_hardware",
                executable="chassis_simulator_node",
                parameters=[
                    {
                        "bind_host": bind_host,
                        "port": port,
                        "battery_voltage": battery_voltage,
                    }
                ],
                output="screen",
            ),
        ]
    )

