from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_slam_mode(context, *args, **kwargs):
    slam_mode = LaunchConfiguration("slam_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    if slam_mode == "cartographer":
        return [
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    PathJoinSubstitution([FindPackageShare("robot_navigation"), "config"]),
                    "-configuration_basename",
                    "cartographer.lua",
                ],
                output="screen",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    if slam_mode == "gmapping":
        return [
            Node(
                package="slam_gmapping_sim",
                executable="slam_gmapping_sim",
                name="slam_gmapping",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        ]

    slam_params = LaunchConfiguration("slam_params_file")
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "slam_params_file": slam_params,
                "use_sim_time": use_sim_time,
            }.items(),
        )
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "slam_toolbox.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("slam_mode", default_value="slam_toolbox"),
            DeclareLaunchArgument("slam_params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_slam_mode),
        ]
    )
