import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_testing.actions
import launch_ros.actions
import launch_ros.substitutions
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from launch_testing_ros import WaitForTopics


def _static_base_tf_node():
    return launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.075", "0", "0", "0", "base_footprint", "base_link"],
        output="screen",
    )


def _static_lidar_tf_node():
    return launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.18", "0", "0.27", "0", "0", "0", "base_link", "lidar_link"],
        output="screen",
    )


def _static_map_tf_node():
    return launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )


def _wait_for_topics(topics, timeout_sec=15.0):
    wait_for_topics = WaitForTopics(topics, timeout=timeout_sec)
    try:
        if not wait_for_topics.wait():
            raise AssertionError(
                "topics not received before Nav2 runtime check: "
                + ", ".join(wait_for_topics.topics_not_received())
            )
    finally:
        wait_for_topics.shutdown()


def _wait_for_message(node, executor, topic_name, msg_type, timeout_sec=15.0):
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    received = {"value": False}

    def _callback(_msg):
        received["value"] = True

    subscription = node.create_subscription(msg_type, topic_name, _callback, qos)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
            if received["value"]:
                return
        raise AssertionError(f"{topic_name} message not received")
    finally:
        node.destroy_subscription(subscription)


def _wait_for_service_available(node, executor, service_name, service_type, timeout_sec=30.0):
    client = node.create_client(service_type, service_name)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return
            executor.spin_once(timeout_sec=0.1)
        raise AssertionError(f"{service_name} service not available")
    finally:
        node.destroy_client(client)


def generate_test_description():
    static_base_tf = _static_base_tf_node()
    static_lidar_tf = _static_lidar_tf_node()
    static_map_tf = _static_map_tf_node()
    cmd_vel_mux = launch_ros.actions.Node(
        package="robot_teleop",
        executable="cmd_vel_mux_node",
        parameters=[{"default_source": "nav2", "watchdog_timeout_ms": 500}],
        output="screen",
    )
    chassis = launch_ros.actions.Node(
        package="robot_hardware",
        executable="chassis_driver_node",
        parameters=[{"backend": "mock", "publish_tf": True}],
        output="screen",
    )
    fake_scan = launch_ros.actions.Node(
        package="robot_sensors",
        executable="fake_scan_node",
        parameters=[{"topic": "/scan"}],
        output="screen",
    )
    nav_launch = launch.substitutions.PathJoinSubstitution(
        [launch_ros.substitutions.FindPackageShare("robot_navigation"), "launch", "nav.launch.py"]
    )
    nav2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return (
        launch.LaunchDescription(
            [
                static_base_tf,
                static_lidar_tf,
                static_map_tf,
                cmd_vel_mux,
                chassis,
                fake_scan,
                nav2,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "static_base_tf": static_base_tf,
            "static_lidar_tf": static_lidar_tf,
            "static_map_tf": static_map_tf,
            "cmd_vel_mux": cmd_vel_mux,
            "chassis": chassis,
            "fake_scan": fake_scan,
            "nav2": nav2,
        },
    )


class TestNav2MockRuntime(unittest.TestCase):
    def test_nav2_core_services_are_available(self):
        _wait_for_topics(
            [
                ("/scan", LaserScan),
                ("/odom", Odometry),
            ],
            timeout_sec=20.0,
        )
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("nav2_lifecycle_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            _wait_for_message(node, executor, "/map", OccupancyGrid, timeout_sec=30.0)
            for service_name in [
                "/map_server/get_state",
                "/amcl/get_state",
                "/planner_server/get_state",
                "/controller_server/get_state",
                "/behavior_server/get_state",
                "/bt_navigator/get_state",
            ]:
                _wait_for_service_available(
                    node, executor, service_name, GetState, timeout_sec=45.0
                )
        finally:
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)

    def test_nav2_mock_runtime_topics_remain_available(self):
        _wait_for_topics(
            [
                ("/scan", LaserScan),
                ("/odom", Odometry),
            ],
            timeout_sec=30.0,
        )
