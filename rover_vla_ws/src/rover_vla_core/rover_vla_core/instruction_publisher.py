#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class InstructionPublisher(Node):
    """Publishes one task instruction with transient-local durability.

    The instruction remains available to late subscribers and is also
    republished periodically for simple recorders.
    """

    def __init__(self) -> None:
        super().__init__("instruction_publisher")
        self.declare_parameter("instruction", "Drive to the blue cone")
        self.declare_parameter("topic", "/vla/instruction")
        self.declare_parameter("republish_period_sec", 1.0)

        self.instruction = str(self.get_parameter("instruction").value)
        topic = str(self.get_parameter("topic").value)
        period = float(self.get_parameter("republish_period_sec").value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(String, topic, qos)
        self.timer = self.create_timer(max(period, 0.1), self.publish_instruction)
        self.publish_instruction()
        self.get_logger().info(f'Publishing instruction "{self.instruction}" on {topic}')

    def publish_instruction(self) -> None:
        msg = String()
        msg.data = self.instruction
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InstructionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
