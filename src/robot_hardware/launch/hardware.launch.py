from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    dev = LaunchConfiguration("dev")
    baud = LaunchConfiguration("baud")
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")
    io_timeout_ms = LaunchConfiguration("io_timeout_ms")
    protocol = LaunchConfiguration("protocol")
    chassis_type = LaunchConfiguration("chassis_type")
    publish_tf = LaunchConfiguration("publish_tf")
    publish_odom = LaunchConfiguration("publish_odom")
    imu_source = LaunchConfiguration("imu_source")
    calibration_file = LaunchConfiguration("calibration_file")
    default_calibration_file = PathJoinSubstitution(
        [FindPackageShare("robot_hardware"), "config", "chassis_calibration.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("sensor_source", default_value="fake"),
            DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("ip", default_value="192.168.1.10"),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("io_timeout_ms", default_value="500"),
            DeclareLaunchArgument("protocol", default_value="text"),
            DeclareLaunchArgument("chassis_type", default_value="diff_drive"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_odom", default_value="true"),
            DeclareLaunchArgument("imu_source", default_value="auto"),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration_file),
            Node(
                package="robot_hardware",
                executable="chassis_driver_node",
                parameters=[
                    calibration_file,
                    {
                        "backend": backend,
                        "publish_tf": publish_tf,
                        "publish_odom": publish_odom,
                        "imu_source": imu_source,
                        "serial_device": dev,
                        "serial_baud": baud,
                        "udp_host": ip,
                        "udp_port": port,
                        "io_timeout_ms": io_timeout_ms,
                        "protocol": protocol,
                        "chassis_type": chassis_type,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_hardware",
                executable="odom_path_recorder_node",
                parameters=[{"input_topic": "/odom", "output_topic": "/odom/path"}],
                output="screen",
            ),
            Node(
                package="robot_sensors",
                executable="fake_scan_node",
                parameters=[{"topic": "/scan"}],
                output="screen",
            ),
        ]
    )
