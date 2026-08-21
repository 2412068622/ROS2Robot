import time
import unittest

import launch
import launch_testing.actions
import launch_ros.actions
import rclpy
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from robot_interfaces.action import NavigateSequence
from robot_interfaces.msg import ChassisState, RobotState, TaskStatus
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from launch_testing_ros import WaitForTopics
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_test_description():
    twist_mux_config = PathJoinSubstitution(
        [FindPackageShare("robot_teleop"), "config", "twist_mux.yaml"]
    )
    twist_mux = launch_ros.actions.Node(
        package="twist_mux",
        executable="twist_mux",
        remappings=[("/cmd_vel_out", "/muxed_cmd_vel")],
        parameters=[twist_mux_config],
        output="screen",
    )
    cmd_vel_safety_gate = launch_ros.actions.Node(
        package="robot_teleop",
        executable="cmd_vel_safety_gate_node",
        parameters=[
            {
                "input_topic": "/muxed_cmd_vel",
                "default_source": "teleop",
                "watchdog_timeout_ms": 500,
            }
        ],
        output="screen",
    )
    chassis = launch_ros.actions.Node(
        package="robot_hardware",
        executable="chassis_driver_node",
        parameters=[{"backend": "mock"}],
        output="screen",
    )
    fake_scan = launch_ros.actions.Node(
        package="robot_sensors",
        executable="fake_scan_node",
        parameters=[{"topic": "/scan"}],
        output="screen",
    )
    tasks = launch_ros.actions.Node(
        package="robot_tasks",
        executable="navigate_sequence_server_node",
        parameters=[
            {
                "use_nav2_action": True,
                "simulate_without_nav2": True,
                "nav2_server_timeout_ms": 200,
            }
        ],
        output="screen",
    )
    system_monitor = launch_ros.actions.Node(
        package="robot_utils",
        executable="system_monitor_node",
        output="screen",
    )
    fault_supervisor = launch_ros.actions.Node(
        package="robot_utils",
        executable="fault_supervisor_node",
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                twist_mux,
                cmd_vel_safety_gate,
                chassis,
                fake_scan,
                tasks,
                system_monitor,
                fault_supervisor,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "twist_mux": twist_mux,
            "cmd_vel_safety_gate": cmd_vel_safety_gate,
            "chassis": chassis,
            "fake_scan": fake_scan,
            "tasks": tasks,
            "system_monitor": system_monitor,
            "fault_supervisor": fault_supervisor,
        },
    )


class TestMockBringup(unittest.TestCase):
    def test_required_topics_publish(self):
        required_topics = [
            ("/cmd_vel_mux/active_source", String),
            ("/emergency_stop/state", Bool),
            ("/scan", LaserScan),
            ("/imu/data", Imu),
            ("/odom", Odometry),
            ("/wheel/odom", Odometry),
            ("/task_status", TaskStatus),
            ("/robot_state", RobotState),
            ("/system_health", RobotState),
            ("/fault_supervisor/state", RobotState),
            ("/diagnostics", DiagnosticArray),
            ("/chassis/state", ChassisState),
        ]
        wait_for_topics = WaitForTopics(required_topics, timeout=10.0)
        try:
            self.assertTrue(wait_for_topics.wait(), wait_for_topics.topics_not_received())
        finally:
            wait_for_topics.shutdown()

    def test_required_services_available(self):
        required_services = {
            "/v2/set_cmd_source",
            "/enable_emergency_stop",
            "/clear_emergency_stop",
            "/clear_odometry",
            "/v2/get_statistics",
            "/pause_mission",
            "/resume_mission",
            "/cancel_mission",
            "/v2/set_chassis_mode",
        }
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("mock_bringup_service_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                available = {name for name, _ in node.get_service_names_and_types()}
                if required_services.issubset(available):
                    return
                executor.spin_once(timeout_sec=0.1)
            self.fail(f"missing services: {sorted(required_services - available)}")
        finally:
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)

    def test_system_health_reaches_ok(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("mock_bringup_health_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        latest_health = {"state": None, "message": None}
        sub = node.create_subscription(
            RobotState,
            "/system_health",
            lambda msg: latest_health.update({"state": msg.state, "message": msg.message}),
            10,
        )
        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if latest_health["state"] == "OK":
                    return
                executor.spin_once(timeout_sec=0.1)
            self.fail(
                "system health did not reach OK; "
                f"last={latest_health['state']} message={latest_health['message']}"
            )
        finally:
            node.destroy_subscription(sub)
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)

    def test_navigate_sequence_fallback_succeeds(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("mock_bringup_action_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, NavigateSequence, "/navigate_sequence")
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=10.0))
            goal = NavigateSequence.Goal()
            goal.loop = False
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = 1.0
            pose.pose.orientation.w = 1.0
            goal.goals.append(pose)

            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                node, send_future, executor=executor, timeout_sec=10.0)
            goal_handle = send_future.result()
            self.assertIsNotNone(goal_handle)
            self.assertTrue(goal_handle.accepted)

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                node, result_future, executor=executor, timeout_sec=10.0)
            result = result_future.result().result
            self.assertTrue(result.success)
            self.assertEqual(1, result.succeeded_goals)
            self.assertEqual(0, result.failed_goals)
        finally:
            client.destroy()
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)
