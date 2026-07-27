#!/usr/bin/env python3
"""Convert the rover's UDP teleoperation packets into ROS Twist messages."""

import json
import math
import socket
import time
from typing import Any, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def parse_twist_datagram(data: bytes) -> Tuple[float, float]:
    """Return finite linear.x and angular.z values from one UDP JSON packet."""
    try:
        payload: Any = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"packet is not valid UTF-8 JSON: {exc}") from exc

    if not isinstance(payload, dict):
        raise ValueError("packet root must be a JSON object")

    linear = payload.get("linear")
    angular = payload.get("angular")
    if not isinstance(linear, dict) or not isinstance(angular, dict):
        raise ValueError("packet must contain linear and angular objects")
    if "x" not in linear or "z" not in angular:
        raise ValueError("packet must contain linear.x and angular.z")

    linear_x = linear["x"]
    angular_z = angular["z"]
    if (
        isinstance(linear_x, bool)
        or isinstance(angular_z, bool)
        or not isinstance(linear_x, (int, float))
        or not isinstance(angular_z, (int, float))
    ):
        raise ValueError("linear.x and angular.z must be numbers")

    values = (float(linear_x), float(angular_z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("linear.x and angular.z must be finite")
    return values


def timeout_expired(
    last_valid_time: float | None, now: float, timeout_sec: float
) -> bool:
    """Return whether a previously received valid command is now stale."""
    return last_valid_time is not None and now - last_valid_time >= timeout_sec


class TeleopAdapter(Node):
    """Receive UDP teleop JSON and publish raw expert actions on a ROS topic."""

    def __init__(self) -> None:
        super().__init__("teleop_adapter")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 6001)
        self.declare_parameter("buffer_size", 1024)
        self.declare_parameter("poll_rate_hz", 100.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("output_topic", "/teleop/cmd_vel")
        self.declare_parameter("allowed_source_ip", "")
        self.declare_parameter("max_packets_per_poll", 32)

        bind_address = str(self.get_parameter("bind_address").value)
        port = int(self.get_parameter("port").value)
        self.buffer_size = max(int(self.get_parameter("buffer_size").value), 128)
        poll_rate = float(self.get_parameter("poll_rate_hz").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self.allowed_source_ip = str(
            self.get_parameter("allowed_source_ip").value
        ).strip()
        self.max_packets_per_poll = max(
            int(self.get_parameter("max_packets_per_poll").value), 1
        )

        if not 0 <= port <= 65535:
            raise ValueError("port must be between 0 and 65535")
        if not math.isfinite(poll_rate) or poll_rate <= 0.0:
            raise ValueError("poll_rate_hz must be finite and positive")
        if (
            not math.isfinite(self.command_timeout)
            or self.command_timeout <= 0.0
        ):
            raise ValueError(
                "command_timeout_sec must be finite and positive"
            )

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((bind_address, port))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise

        self.last_valid_packet_time: float | None = None
        self.timeout_stop_published = False
        self.timer = self.create_timer(1.0 / poll_rate, self.poll_socket)
        source_description = self.allowed_source_ip or "any source"
        self.get_logger().info(
            f"UDP teleop {bind_address}:{port} -> {output_topic}; "
            f"accepting {source_description}"
        )
        if bind_address == "0.0.0.0" and not self.allowed_source_ip:
            self.get_logger().warning(
                "UDP teleop accepts packets from any reachable host. "
                "Use allowed_source_ip on an untrusted network."
            )

    @staticmethod
    def make_twist(linear_x: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def poll_socket(self) -> None:
        for _ in range(self.max_packets_per_poll):
            try:
                data, address = self.socket.recvfrom(self.buffer_size)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f"UDP receive failed: {exc}")
                break

            source_ip = address[0]
            if self.allowed_source_ip and source_ip != self.allowed_source_ip:
                self.get_logger().warning(
                    f"Ignored UDP teleop packet from disallowed source {source_ip}"
                )
                continue

            try:
                linear_x, angular_z = parse_twist_datagram(data)
            except ValueError as exc:
                self.get_logger().warning(
                    f"Ignored invalid UDP teleop packet from {source_ip}: {exc}"
                )
                continue

            self.publisher.publish(self.make_twist(linear_x, angular_z))
            self.last_valid_packet_time = time.monotonic()
            self.timeout_stop_published = False

        now = time.monotonic()
        if (
            timeout_expired(
                self.last_valid_packet_time, now, self.command_timeout
            )
            and not self.timeout_stop_published
        ):
            self.publisher.publish(Twist())
            self.timeout_stop_published = True
            self.get_logger().warning(
                "UDP teleop command timed out; published a zero expert action."
            )

    def destroy_node(self):
        if self.last_valid_packet_time is not None and rclpy.ok():
            self.publisher.publish(Twist())
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
