from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    dev = LaunchConfiguration("dev")
    baud = LaunchConfiguration("baud")
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")
    protocol = LaunchConfiguration("protocol")
    chassis_type = LaunchConfiguration("chassis_type")
    controllers_file = LaunchConfiguration("controllers_file")
    geometry_file = LaunchConfiguration("geometry_file")
    calibration_file = LaunchConfiguration("calibration_file")

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                robot_description_file,
                " geometry_file:=",
                geometry_file,
                " calibration_file:=",
                calibration_file,
                " use_ros2_control_hardware:=true",
                " ros2_control_backend:=",
                backend,
                " ros2_control_serial_device:=",
                dev,
                " ros2_control_serial_baud:=",
                baud,
                " ros2_control_udp_host:=",
                ip,
                " ros2_control_udp_port:=",
                port,
                " ros2_control_protocol:=",
                protocol,
                " ros2_control_chassis_type:=",
                chassis_type,
            ]
        )
    }
    default_geometry_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "robot_geometry.yaml"]
    )
    default_calibration_file = PathJoinSubstitution(
        [FindPackageShare("robot_hardware"), "config", "chassis_calibration.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("ip", default_value="192.168.1.10"),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("protocol", default_value="text"),
            DeclareLaunchArgument("geometry_file", default_value=default_geometry_file),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration_file),
            DeclareLaunchArgument(
                "chassis_type",
                default_value="diff_drive",
                description=(
                    "Chassis kinematics for the two-wheel ros2_control hardware interface; "
                    "only diff_drive is supported"
                ),
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_hardware"), "config", "ros2_control_controllers.yaml"]
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_file],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
                output="screen",
            ),
        ]
    )
