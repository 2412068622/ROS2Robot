from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    geometry_file = LaunchConfiguration("geometry_file")
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "display.rviz"]
    )
    default_geometry_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "robot_geometry.yaml"]
    )

    robot_description = {
        "robot_description": Command(
            ["xacro ", robot_description_file, " geometry_file:=", geometry_file]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("geometry_file", default_value=default_geometry_file),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_rviz),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
