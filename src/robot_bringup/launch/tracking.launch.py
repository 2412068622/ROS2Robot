from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller = LaunchConfiguration("controller")
    path_file = LaunchConfiguration("path_file")
    use_mock_chassis = LaunchConfiguration("use_mock_chassis")
    output_csv = LaunchConfiguration("output_csv")
    output_json = LaunchConfiguration("output_json")
    scenario_name = LaunchConfiguration("scenario_name")
    calibration_file = LaunchConfiguration("calibration_file")
    default_path = PathJoinSubstitution(
        [FindPackageShare("robot_experiments"), "paths", "demo_path.txt"]
    )
    cmd_vel_stack_launch = PathJoinSubstitution(
        [FindPackageShare("robot_teleop"), "launch", "cmd_vel_stack.launch.py"]
    )
    default_calibration_file = PathJoinSubstitution(
        [FindPackageShare("robot_hardware"), "config", "chassis_calibration.yaml"]
    )
    pure_pursuit_selected = PythonExpression(["'", controller, "' == 'pure_pursuit'"])
    stanley_selected = PythonExpression(["'", controller, "' == 'stanley'"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("controller", default_value="pure_pursuit"),
            DeclareLaunchArgument("path_file", default_value=default_path),
            DeclareLaunchArgument("use_mock_chassis", default_value="false"),
            DeclareLaunchArgument("scenario_name", default_value="tracking_benchmark"),
            DeclareLaunchArgument("output_csv", default_value="experiment_results.csv"),
            DeclareLaunchArgument("output_json", default_value="experiment_results.json"),
            DeclareLaunchArgument("use_twist_mux", default_value="true"),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration_file),
            Node(
                package="robot_path_tracking",
                executable="path_publisher_node",
                parameters=[{"path_file": path_file, "frame_id": "odom"}],
                output="screen",
            ),
            Node(
                package="robot_path_tracking",
                executable="pure_pursuit_controller_node",
                condition=IfCondition(pure_pursuit_selected),
                output="screen",
            ),
            Node(
                package="robot_path_tracking",
                executable="stanley_controller_node",
                condition=IfCondition(stanley_selected),
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cmd_vel_stack_launch),
                launch_arguments={
                    "use_twist_mux": LaunchConfiguration("use_twist_mux"),
                    "default_source": "tracking",
                    "watchdog_timeout_ms": "500",
                }.items(),
            ),
            Node(
                package="robot_hardware",
                executable="chassis_driver_node",
                condition=IfCondition(use_mock_chassis),
                parameters=[calibration_file, {"backend": "mock", "publish_tf": True}],
                output="screen",
            ),
            Node(
                package="robot_experiments",
                executable="experiment_runner_node",
                parameters=[
                    {
                        "controller_name": controller,
                        "scenario_name": scenario_name,
                        "path_file": path_file,
                        "output_csv": output_csv,
                        "output_json": output_json,
                    }
                ],
                output="screen",
            ),
        ]
    )
