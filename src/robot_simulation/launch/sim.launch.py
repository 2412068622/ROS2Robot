import atexit
import os
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _remove_generated_world(path):
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass


def _render_world(
    context,
    world_template,
    geometry_file,
    simulation_physics_file,
    simulation_media_directory,
):
    template_path = world_template.perform(context)
    geometry_path = geometry_file.perform(context)
    physics_path = simulation_physics_file.perform(context)
    media_path = simulation_media_directory.perform(context)
    for label, path in [
        ("world template", template_path),
        ("geometry file", geometry_path),
        ("simulation physics file", physics_path),
    ]:
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist or is not a file: {path}")
    if not os.path.isdir(media_path):
        raise RuntimeError(
            f"simulation media directory does not exist or is not a directory: {media_path}"
        )

    output = xacro.process_file(
        template_path,
        mappings={
            "geometry_file": geometry_path,
            "simulation_physics_file": physics_path,
            "simulation_media_directory": media_path,
        },
    )
    descriptor, generated_path = tempfile.mkstemp(
        prefix=f"indoor_room_{os.getpid()}_", suffix=".sdf"
    )
    os.close(descriptor)
    try:
        with open(generated_path, "w", encoding="utf-8") as stream:
            stream.write(output.toprettyxml(indent="  "))
        validation = subprocess.run(
            ["gz", "sdf", "-k", generated_path],
            check=False,
            capture_output=True,
            text=True,
        )
        if validation.returncode != 0:
            details = validation.stderr.strip() or validation.stdout.strip()
            raise RuntimeError(f"generated Gazebo SDF is invalid: {details}")
    except Exception:
        _remove_generated_world(generated_path)
        raise

    atexit.register(_remove_generated_world, generated_path)
    return [SetLaunchConfiguration("generated_world", generated_path)]


def _cleanup_world(context):
    generated_path = LaunchConfiguration("generated_world").perform(context)
    _remove_generated_world(generated_path)
    return []


def generate_launch_description():
    use_gui = LaunchConfiguration("gui")
    labels_enabled = LaunchConfiguration("labels_enabled")
    label_scale = LaunchConfiguration("label_scale")
    robot2_pose_bridge_enabled = LaunchConfiguration("robot2_pose_bridge_enabled")
    enable_robot2_fleet_demo_driver = LaunchConfiguration(
        "enable_robot2_fleet_demo_driver"
    )
    geometry_file = LaunchConfiguration("geometry_file")
    simulation_physics_file = LaunchConfiguration("simulation_physics_file")
    generated_world = LaunchConfiguration("generated_world")
    world_template = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "worlds", "indoor_room.sdf.xacro"]
    )
    default_geometry_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "robot_geometry.yaml"]
    )
    default_simulation_physics_file = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "config", "simulation_physics.yaml"]
    )
    simulation_media_directory = PathJoinSubstitution(
        [FindPackageShare("robot_simulation"), "media"]
    )
    sensors_launch = PathJoinSubstitution(
        [FindPackageShare("robot_sensors"), "launch", "sensors.launch.py"]
    )
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            ["xacro ", robot_description_file, " geometry_file:=", geometry_file]
        )
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("labels_enabled", default_value="true"),
            DeclareLaunchArgument("label_scale", default_value="0.55"),
            DeclareLaunchArgument("robot2_pose_bridge_enabled", default_value="true"),
            DeclareLaunchArgument(
                "enable_robot2_fleet_demo_driver", default_value="true"
            ),
            DeclareLaunchArgument("geometry_file", default_value=default_geometry_file),
            DeclareLaunchArgument(
                "simulation_physics_file", default_value=default_simulation_physics_file
            ),
            OpaqueFunction(
                function=_render_world,
                args=[
                    world_template,
                    geometry_file,
                    simulation_physics_file,
                    simulation_media_directory,
                ],
            ),
            RegisterEventHandler(
                OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_world)])
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", generated_world],
                condition=IfCondition(use_gui),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-s", "-r", generated_world],
                condition=UnlessCondition(use_gui),
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/robot_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/model/mobile_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                    "/model/mobile_robot_2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                    "/sim/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/robot_2/sim/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/sim/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/robot_2/sim/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/sim/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/robot_2/sim/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/world/indoor_room/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                    "/world/indoor_room/control@ros_gz_interfaces/srv/ControlWorld",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
            ),
            Node(
                package="robot_sensors",
                executable="laser_filter_node",
                name="robot_2_laser_filter_node",
                parameters=[
                    {
                        "input_topic": "/robot_2/sim/scan",
                        "output_topic": "/robot_2/scan",
                        "frame_id": "robot_2/lidar_link",
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_sensors",
                executable="imu_filter_node",
                name="robot_2_imu_filter_node",
                parameters=[
                    {
                        "input_topic": "/robot_2/sim/imu",
                        "output_topic": "/robot_2/imu/data",
                        "frame_id": "robot_2/imu_link",
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[robot_description, {"use_sim_time": True, "rate": 30}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace="robot_2",
                name="robot_2_joint_state_publisher",
                parameters=[robot_description, {"use_sim_time": True, "rate": 30}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="robot_2",
                name="robot_2_state_publisher",
                parameters=[
                    robot_description,
                    {"use_sim_time": True, "frame_prefix": "robot_2/"},
                ],
                output="screen",
            ),
            Node(
                package="robot_teleop",
                executable="cmd_vel_mux_node",
                parameters=[{"default_source": "teleop"}],
                output="screen",
            ),
            Node(
                package="robot_teleop",
                executable="virtual_rc_node",
                parameters=[{"manual_takeover": True}],
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="amr_sim_visualizer_node",
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="amr_sim_gazebo_entity_bridge_node",
                parameters=[
                    {
                        "labels_enabled": labels_enabled,
                        "label_scale": label_scale,
                        "robot2_pose_bridge_enabled": robot2_pose_bridge_enabled,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="gazebo_odom_tf_node",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="gazebo_odom_tf_node",
                name="robot_2_gazebo_odom_tf_node",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/model/mobile_robot_2/odometry",
                        "output_topic": "/robot_2/odom",
                        "odom_frame": "robot_2/odom",
                        "base_frame": "robot_2/base_footprint",
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="robot2_fleet_demo_driver_node",
                condition=IfCondition(enable_robot2_fleet_demo_driver),
                parameters=[
                    {
                        "use_sim_time": True,
                        "cmd_vel_topic": "/robot_2/cmd_vel",
                        "odom_topic": "/robot_2/odom",
                        "main_odom_topic": "/odom",
                        "linear_command_sign": 1.0,
                        "odom_yaw_sign": -1.0,
                        "angular_command_sign": -1.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_hardware",
                executable="odom_path_recorder_node",
                parameters=[
                    {
                        "input_topic": "/odom",
                        "output_topic": "/odom/path",
                        "publish_period_ms": 250,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_simulation",
                executable="gazebo_unpause_node",
                parameters=[
                    {
                        "service_name": "/world/indoor_room/control",
                        "max_attempts": 30,
                        "retry_period_s": 0.5,
                    }
                ],
                output="screen",
            ),
        ]
    )
