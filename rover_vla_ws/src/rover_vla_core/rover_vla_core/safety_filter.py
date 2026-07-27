#!/usr/bin/env python3
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyFilter(Node):
    """Clips learned/teleop velocity commands and enforces a watchdog stop."""

    def __init__(self) -> None:
        super().__init__("safety_filter")
        self.declare_parameter("input_topic", "/vla/raw_cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("emergency_stop_topic", "/vla/emergency_stop")
        self.declare_parameter("max_linear_mps", 0.10)
        self.declare_parameter("max_angular_rps", 0.40)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("timeout_sec", 0.50)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        e_stop_topic = str(self.get_parameter("emergency_stop_topic").value)
        self.max_linear = abs(float(self.get_parameter("max_linear_mps").value))
        self.max_angular = abs(float(self.get_parameter("max_angular_rps").value))
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.timeout_sec = max(float(self.get_parameter("timeout_sec").value), 0.05)
        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.command_callback, 10)
        self.create_subscription(Bool, e_stop_topic, self.e_stop_callback, 10)

        self.lock = threading.Lock()
        self.last_command = Twist()
        self.last_command_time = None
        self.emergency_stop = False

        self.timer = self.create_timer(1.0 / rate, self.publish_safe_command)
        self.get_logger().info(
            f"Safety filter: {self.input_topic} -> {self.output_topic}; "
            f"limits linear={self.max_linear:.3f} m/s, angular={self.max_angular:.3f} rad/s"
        )

    @staticmethod
    def _clip(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    def command_callback(self, msg: Twist) -> None:
        values = [msg.linear.x, msg.angular.z]
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error("Rejected non-finite command.")
            return

        safe = Twist()
        linear_min = -self.max_linear if self.allow_reverse else 0.0
        safe.linear.x = self._clip(float(msg.linear.x), linear_min, self.max_linear)
        safe.angular.z = self._clip(float(msg.angular.z), -self.max_angular, self.max_angular)

        with self.lock:
            self.last_command = safe
            self.last_command_time = self.get_clock().now()

    def e_stop_callback(self, msg: Bool) -> None:
        with self.lock:
            self.emergency_stop = bool(msg.data)
        if msg.data:
            self.get_logger().warning("Emergency stop asserted.")

    def publish_safe_command(self) -> None:
        now = self.get_clock().now()
        output = Twist()

        with self.lock:
            stale = (
                self.last_command_time is None
                or (now - self.last_command_time).nanoseconds / 1e9 > self.timeout_sec
            )
            if not self.emergency_stop and not stale:
                output = self.last_command

        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
