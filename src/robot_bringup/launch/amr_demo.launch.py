from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_map = LaunchConfiguration("use_map")
    auto_demo = LaunchConfiguration("auto_demo")
    labels_enabled = LaunchConfiguration("labels_enabled")
    label_scale = LaunchConfiguration("label_scale")
    geometry_file = LaunchConfiguration("geometry_file")
    simulation_physics_file = LaunchConfiguration("simulation_physics_file")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "rviz", "amr_sim.rviz"]
    )
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "launch", "sim.launch.py"]
    )
    mission_runner_launch = PathJoinSubstitution(
        [FindPackageShare("robot_tasks"), "launch", "mission_runner.launch.py"]
    )
    station_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "config", "amr_sim_stations.yaml"]
    )
    map_zones_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "config", "amr_sim_map_zones.yaml"]
    )
    map_file = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "maps", "indoor_room.yaml"]
    )
    default_geometry_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "robot_geometry.yaml"]
    )
    default_simulation_physics_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "config", "simulation_physics.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_map", default_value="true"),
            DeclareLaunchArgument("auto_demo", default_value="true"),
            # Gazebo 3D 文字网格朝向固定，从斜视角会侧/反且易重叠；默认关闭，
            # 文字统一由 RViz 的 TEXT_VIEW_FACING marker 显示（永远正对相机）。
            # 需要纯 Gazebo 演示无 RViz 时，可显式传 labels_enabled:=true。
            DeclareLaunchArgument("labels_enabled", default_value="false"),
            DeclareLaunchArgument("label_scale", default_value="0.55"),
            DeclareLaunchArgument("geometry_file", default_value=default_geometry_file),
            DeclareLaunchArgument(
                "simulation_physics_file", default_value=default_simulation_physics_file
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "gui": gui,
                    "labels_enabled": labels_enabled,
                    "label_scale": label_scale,
                    "geometry_file": geometry_file,
                    "simulation_physics_file": simulation_physics_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_runner_launch),
                launch_arguments={
                    "autostart": "false",
                    "server_timeout_ms": "3000",
                    "return_to_dock_on_low_battery": "false",
                    "station_file": station_file,
                    "preflight_zones_file": map_zones_file,
                }.items(),
            ),
            Node(
                package="robot_tasks",
                executable="navigate_sequence_server_node",
                parameters=[
                    {
                        "use_nav2_action": False,
                        "simulate_without_nav2": True,
                        "simulated_motion_enabled": True,
                        "simulated_cmd_vel_topic": "/virtual_rc/cmd_vel",
                        "simulated_odom_topic": "/model/mobile_robot/odometry",
                        "simulated_goal_tolerance_m": 0.28,
                        "simulated_goal_timeout_s": 90.0,
                        "simulated_max_linear_mps": 0.65,
                    }
                ],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="amr_demo_map_server",
                parameters=[{"yaml_filename": map_file, "use_sim_time": True}],
                condition=IfCondition(use_map),
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="amr_demo_map_lifecycle_manager",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["amr_demo_map_server"],
                    }
                ],
                condition=IfCondition(use_map),
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=IfCondition(use_map),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="amr_sim_demo_director_node",
                parameters=[{"enabled": True}],
                condition=IfCondition(auto_demo),
                output="screen",
            ),
        ]
    )
