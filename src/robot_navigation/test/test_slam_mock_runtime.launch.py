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
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
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


def _wait_for_lifecycle_state(service_name, expected_label, timeout_sec=30.0):
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("slam_lifecycle_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = node.create_client(GetState, service_name)
    deadline = time.monotonic() + timeout_sec
    last_label = "<service unavailable>"
    try:
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=0.2):
                executor.spin_once(timeout_sec=0.1)
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(
                node, future, executor=executor, timeout_sec=1.0
            )
            if future.done() and future.result() is not None:
                last_label = future.result().current_state.label
                if last_label == expected_label:
                    return
            executor.spin_once(timeout_sec=0.1)
        raise AssertionError(
            f"{service_name} did not reach {expected_label}; last state={last_label}"
        )
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)


def generate_test_description():
    static_base_tf = _static_base_tf_node()
    static_lidar_tf = _static_lidar_tf_node()
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
    slam_launch = launch.substitutions.PathJoinSubstitution(
        [launch_ros.substitutions.FindPackageShare("robot_navigation"), "launch", "slam.launch.py"]
    )
    slam = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return (
        launch.LaunchDescription(
            [
                static_base_tf,
                static_lidar_tf,
                chassis,
                fake_scan,
                slam,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "static_base_tf": static_base_tf,
            "static_lidar_tf": static_lidar_tf,
            "chassis": chassis,
            "fake_scan": fake_scan,
            "slam": slam,
        },
    )


class TestSlamMockRuntime(unittest.TestCase):
    def test_mock_sensor_topics_publish(self):
        wait_for_topics = WaitForTopics(
            [
                ("/scan", LaserScan),
                ("/odom", Odometry),
            ],
            timeout=10.0,
        )
        try:
            self.assertTrue(wait_for_topics.wait(), wait_for_topics.topics_not_received())
        finally:
            wait_for_topics.shutdown()

    def test_slam_toolbox_reaches_active(self):
        _wait_for_lifecycle_state("/slam_toolbox/get_state", "active", timeout_sec=30.0)
