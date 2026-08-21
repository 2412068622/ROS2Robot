from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_source = LaunchConfiguration("default_source")
    watchdog_timeout_ms = LaunchConfiguration("watchdog_timeout_ms")
    dynamic_angular_limit_radps = LaunchConfiguration("dynamic_angular_limit_radps")
    twist_mux_config = LaunchConfiguration("twist_mux_config")
    mux_output_topic = LaunchConfiguration("mux_output_topic")

    default_twist_mux_config = PathJoinSubstitution(
        [FindPackageShare("robot_teleop"), "config", "twist_mux.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_twist_mux", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("default_source", default_value="teleop"),
            DeclareLaunchArgument("watchdog_timeout_ms", default_value="500"),
            DeclareLaunchArgument("dynamic_angular_limit_radps", default_value="0.6"),
            DeclareLaunchArgument("twist_mux_config", default_value=default_twist_mux_config),
            DeclareLaunchArgument("mux_output_topic", default_value="/muxed_cmd_vel"),
            Node(
                package="twist_mux",
                executable="twist_mux",
                condition=IfCondition(use_twist_mux),
                remappings=[("/cmd_vel_out", mux_output_topic)],
                parameters=[twist_mux_config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_teleop",
                executable="cmd_vel_safety_gate_node",
                condition=IfCondition(use_twist_mux),
                parameters=[
                    {
                        "input_topic": mux_output_topic,
                        "default_source": default_source,
                        "watchdog_timeout_ms": watchdog_timeout_ms,
                        "dynamic_angular_limit_radps": dynamic_angular_limit_radps,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_teleop",
                executable="cmd_vel_mux_node",
                condition=UnlessCondition(use_twist_mux),
                parameters=[
                    {
                        "default_source": default_source,
                        "watchdog_timeout_ms": watchdog_timeout_ms,
                        "dynamic_angular_limit_radps": dynamic_angular_limit_radps,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
