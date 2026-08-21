import time
import unittest

import launch
import launch_testing.actions
import launch_ros.actions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from robot_interfaces.msg import RobotState
from std_msgs.msg import Bool


def generate_test_description():
    cmd_vel_mux = launch_ros.actions.Node(
        package="robot_teleop",
        executable="cmd_vel_mux_node",
        parameters=[{"default_source": "teleop", "watchdog_timeout_ms": 500}],
        output="screen",
    )
    fault_supervisor = launch_ros.actions.Node(
        package="robot_utils",
        executable="fault_supervisor_node",
        parameters=[{"auto_clear": True, "command_cooldown_ms": 50, "startup_grace_ms": 0}],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                cmd_vel_mux,
                fault_supervisor,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"cmd_vel_mux": cmd_vel_mux, "fault_supervisor": fault_supervisor},
    )


class TestFaultSupervisor(unittest.TestCase):
    def test_error_health_triggers_and_ok_clears_estop(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("fault_supervisor_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        health_pub = node.create_publisher(RobotState, "/system_health", 10)
        estop = {"seen": False, "active": False}
        estop_sub = node.create_subscription(
            Bool,
            "/emergency_stop/state",
            lambda msg: estop.update({"seen": True, "active": msg.data}),
            10,
        )
        try:
            self._publish_health_until_estop(
                node, executor, health_pub, estop, "ERROR", True, timeout_sec=10.0
            )
            self._publish_health_until_estop(
                node, executor, health_pub, estop, "OK", False, timeout_sec=10.0
            )
        finally:
            node.destroy_subscription(estop_sub)
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _publish_health_until_estop(
        self, node, executor, publisher, estop, state, expected_active, timeout_sec
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            msg = RobotState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.state = state
            msg.message = f"test health {state}"
            msg.recoverable = state != "ERROR"
            publisher.publish(msg)
            executor.spin_once(timeout_sec=0.1)
            if estop["seen"] and estop["active"] == expected_active:
                return
        self.fail(
            f"emergency stop did not become {expected_active} after health {state}; "
            f"last={estop}"
        )
