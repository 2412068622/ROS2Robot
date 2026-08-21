from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    dev = LaunchConfiguration("dev")
    baud = LaunchConfiguration("baud")
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")
    io_timeout_ms = LaunchConfiguration("io_timeout_ms")
    protocol = LaunchConfiguration("protocol")
    chassis_type = LaunchConfiguration("chassis_type")
    geometry_file = LaunchConfiguration("geometry_file")
    calibration_file = LaunchConfiguration("calibration_file")
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            ["xacro ", robot_description_file, " geometry_file:=", geometry_file]
        )
    }
    cmd_vel_stack_launch = PathJoinSubstitution(
        [FindPackageShare("robot_teleop"), "launch", "cmd_vel_stack.launch.py"]
    )
    default_geometry_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "robot_geometry.yaml"]
    )
    default_calibration_file = PathJoinSubstitution(
        [FindPackageShare("robot_hardware"), "config", "chassis_calibration.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="hardware"),
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("ip", default_value="192.168.1.10"),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("io_timeout_ms", default_value="500"),
            DeclareLaunchArgument("protocol", default_value="text"),
            DeclareLaunchArgument("chassis_type", default_value="diff_drive"),
            DeclareLaunchArgument("use_twist_mux", default_value="true"),
            DeclareLaunchArgument("geometry_file", default_value=default_geometry_file),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration_file),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cmd_vel_stack_launch),
                launch_arguments={
                    "use_twist_mux": LaunchConfiguration("use_twist_mux"),
                    "use_sim_time": use_sim_time,
                    "default_source": "teleop",
                    "watchdog_timeout_ms": "500",
                }.items(),
            ),
            Node(
                package="robot_teleop",
                executable="virtual_rc_node",
                parameters=[{"manual_takeover": False}],
                output="screen",
            ),
            Node(
                package="robot_hardware",
                executable="chassis_driver_node",
                parameters=[
                    calibration_file,
                    {
                        "backend": backend,
                        "mode": mode,
                        "use_sim_time": use_sim_time,
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
            Node(
                package="robot_tasks",
                executable="navigate_sequence_server_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_utils",
                executable="system_monitor_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robot_utils"),
                            "config",
                            "diagnostic_aggregator.yaml",
                        ]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
            Node(
                package="robot_utils",
                executable="fault_supervisor_node",
                parameters=[
                    {"use_sim_time": use_sim_time, "auto_clear": True, "startup_grace_ms": 3000}
                ],
                output="screen",
            ),
        ]
    )
