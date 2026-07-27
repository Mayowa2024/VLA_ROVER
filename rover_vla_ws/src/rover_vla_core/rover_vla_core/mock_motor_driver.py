#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MockMotorDriver(Node):
    """Publishes illustrative wheel commands without touching rover hardware."""

    def __init__(self) -> None:
        super().__init__("mock_motor_driver")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("wheel_command_topic", "/rover/mock_wheel_command")
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("max_abs_command", 1.0)

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        wheel_topic = str(self.get_parameter("wheel_command_topic").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.limit = abs(float(self.get_parameter("max_abs_command").value))

        self.publisher = self.create_publisher(Float32MultiArray, wheel_topic, 10)
        self.create_subscription(Twist, cmd_topic, self.command_callback, 10)
        self.get_logger().info(
            f"Mock motor driver: {cmd_topic} -> {wheel_topic}; no hardware output"
        )

    def clip(self, value: float) -> float:
        return max(-self.limit, min(value, self.limit))

    def command_callback(self, msg: Twist) -> None:
        values = (float(msg.linear.x), float(msg.angular.z))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Rejected non-finite mock motor command.")
            return

        forward = values[0] * self.linear_scale
        turn = values[1] * self.angular_scale
        left = self.clip(forward - turn)
        right = self.clip(forward + turn)

        output = Float32MultiArray()
        output.data = [left, right]
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
