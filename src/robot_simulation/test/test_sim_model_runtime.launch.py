import math
import os
import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.substitutions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

import launch_testing.actions


def generate_test_description():
    simulation_launch = launch.substitutions.PathJoinSubstitution(
        [
            launch_ros.substitutions.FindPackageShare("robot_simulation"),
            "launch",
            "sim.launch.py",
        ]
    )
    simulation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            simulation_launch
        ),
        launch_arguments={
            "gui": "false",
            "labels_enabled": "false",
            "robot2_pose_bridge_enabled": "false",
            "enable_robot2_fleet_demo_driver": "false",
        }.items(),
    )
    return (
        launch.LaunchDescription(
            [
                launch.actions.SetEnvironmentVariable(
                    "GZ_PARTITION", f"robot_model_consistency_test_{os.getpid()}"
                ),
                simulation,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"simulation": simulation},
    )


def _yaw(odometry):
    orientation = odometry.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _angle_delta(left, right):
    return math.atan2(math.sin(left - right), math.cos(left - right))


class TestSimulationModelRuntime(unittest.TestCase):
    def test_geometry_sensors_tf_and_robot_isolation(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node(
            "simulation_model_runtime_test",
            context=context,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        messages = {}
        subscriptions = []

        def subscribe(topic_name, message_type, qos):
            subscriptions.append(
                node.create_subscription(
                    message_type,
                    topic_name,
                    lambda message, topic=topic_name: messages.__setitem__(
                        topic, message
                    ),
                    qos,
                )
            )

        subscribe("/odom", Odometry, QoSProfile(depth=10))
        subscribe("/robot_2/odom", Odometry, QoSProfile(depth=10))
        subscribe("/scan", LaserScan, qos_profile_sensor_data)
        subscribe("/robot_2/scan", LaserScan, qos_profile_sensor_data)
        subscribe("/imu/data", Imu, qos_profile_sensor_data)
        subscribe("/robot_2/imu/data", Imu, qos_profile_sensor_data)

        tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        main_command = node.create_publisher(Twist, "/virtual_rc/cmd_vel", 10)
        robot2_command = node.create_publisher(Twist, "/robot_2/cmd_vel", 10)

        def spin_until(predicate, timeout_sec, failure_message):
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if predicate():
                    return
            self.fail(failure_message)

        def wait_for_transform(parent, child):
            spin_until(
                lambda: tf_buffer.can_transform(parent, child, Time()),
                30.0,
                f"transform {parent} -> {child} was not available",
            )
            return tf_buffer.lookup_transform(parent, child, Time())

        def publish_for(publisher, command, duration_sec):
            spin_until(
                lambda: publisher.get_subscription_count() > 0,
                30.0,
                f"no subscriber discovered for {publisher.topic_name}",
            )
            deadline = time.monotonic() + duration_sec
            while time.monotonic() < deadline:
                publisher.publish(command)
                executor.spin_once(timeout_sec=0.05)
            stop = Twist()
            for _ in range(6):
                publisher.publish(stop)
                executor.spin_once(timeout_sec=0.05)

        required_topics = {
            "/odom",
            "/robot_2/odom",
            "/scan",
            "/robot_2/scan",
            "/imu/data",
            "/robot_2/imu/data",
        }
        try:
            spin_until(
                lambda: required_topics.issubset(messages),
                60.0,
                "simulation did not publish all odometry, laser and IMU topics",
            )

            main_odom = messages["/odom"]
            robot2_odom = messages["/robot_2/odom"]
            self.assertEqual(main_odom.header.frame_id, "odom")
            self.assertEqual(main_odom.child_frame_id, "base_footprint")
            self.assertEqual(robot2_odom.header.frame_id, "robot_2/odom")
            self.assertEqual(robot2_odom.child_frame_id, "robot_2/base_footprint")
            self.assertAlmostEqual(main_odom.pose.pose.position.z, 0.0, places=9)
            self.assertAlmostEqual(robot2_odom.pose.pose.position.z, 0.0, places=9)
            self.assertEqual(messages["/scan"].header.frame_id, "lidar_link")
            self.assertEqual(
                messages["/robot_2/scan"].header.frame_id, "robot_2/lidar_link"
            )
            self.assertEqual(messages["/imu/data"].header.frame_id, "imu_link")
            self.assertEqual(
                messages["/robot_2/imu/data"].header.frame_id,
                "robot_2/imu_link",
            )

            main_base = wait_for_transform("base_footprint", "base_link")
            main_lidar = wait_for_transform("base_link", "lidar_link")
            robot2_base = wait_for_transform(
                "robot_2/base_footprint", "robot_2/base_link"
            )
            robot2_lidar = wait_for_transform(
                "robot_2/base_link", "robot_2/lidar_link"
            )
            for transform in [main_base, robot2_base]:
                self.assertAlmostEqual(
                    transform.transform.translation.z, 0.075, places=6
                )
            for transform in [main_lidar, robot2_lidar]:
                self.assertAlmostEqual(
                    transform.transform.translation.x, 0.18, places=6
                )
                self.assertAlmostEqual(
                    transform.transform.translation.y, 0.0, places=6
                )
                self.assertAlmostEqual(
                    transform.transform.translation.z, 0.27, places=6
                )

            def forward_scan_range():
                scan = messages.get("/scan")
                if scan is None or scan.angle_increment == 0.0:
                    return None
                zero_index = round((0.0 - scan.angle_min) / scan.angle_increment)
                if zero_index < 0 or zero_index >= len(scan.ranges):
                    return None
                return scan.ranges[zero_index]

            spin_until(
                lambda: forward_scan_range() is not None
                and 4.60 < forward_scan_range() < 4.90,
                15.0,
                "front laser range did not converge to the modeled east wall",
            )
            scan = messages["/scan"]
            zero_index = round((0.0 - scan.angle_min) / scan.angle_increment)
            self.assertGreaterEqual(zero_index, 0)
            self.assertLess(zero_index, len(scan.ranges))
            self.assertGreater(scan.ranges[zero_index], 4.60)
            self.assertLess(scan.ranges[zero_index], 4.90)

            main_start = messages["/odom"]
            robot2_start = messages["/robot_2/odom"]
            forward = Twist()
            forward.linear.x = 0.25
            publish_for(main_command, forward, 1.0)
            spin_until(
                lambda: math.hypot(
                    messages["/odom"].pose.pose.position.x
                    - main_start.pose.pose.position.x,
                    messages["/odom"].pose.pose.position.y
                    - main_start.pose.pose.position.y,
                )
                > 0.12,
                5.0,
                "main robot did not move through the virtual RC command chain",
            )
            main_after = messages["/odom"]
            robot2_after = messages["/robot_2/odom"]
            main_distance = math.hypot(
                main_after.pose.pose.position.x - main_start.pose.pose.position.x,
                main_after.pose.pose.position.y - main_start.pose.pose.position.y,
            )
            robot2_distance = math.hypot(
                robot2_after.pose.pose.position.x - robot2_start.pose.pose.position.x,
                robot2_after.pose.pose.position.y - robot2_start.pose.pose.position.y,
            )
            self.assertLess(main_distance, 0.40)
            self.assertLess(robot2_distance, 0.02)
            self.assertLess(
                abs(_angle_delta(_yaw(robot2_after), _yaw(robot2_start))), 0.02
            )

            main_before_robot2 = messages["/odom"]
            robot2_before_turn = messages["/robot_2/odom"]
            turn = Twist()
            turn.angular.z = 0.4
            publish_for(robot2_command, turn, 0.8)
            spin_until(
                lambda: abs(
                    _angle_delta(
                        _yaw(messages["/robot_2/odom"]), _yaw(robot2_before_turn)
                    )
                )
                > 0.12,
                5.0,
                "second robot did not rotate through its isolated command topic",
            )
            main_after_robot2 = messages["/odom"]
            self.assertLess(
                math.hypot(
                    main_after_robot2.pose.pose.position.x
                    - main_before_robot2.pose.pose.position.x,
                    main_after_robot2.pose.pose.position.y
                    - main_before_robot2.pose.pose.position.y,
                ),
                0.02,
            )
            self.assertLess(
                abs(
                    _angle_delta(
                        _yaw(main_after_robot2), _yaw(main_before_robot2)
                    )
                ),
                0.02,
            )
        finally:
            main_command.publish(Twist())
            robot2_command.publish(Twist())
            del tf_listener
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)
