#!/usr/bin/env python3
"""Publish differential-drive odometry from two serial wheel encoders."""

from dataclasses import dataclass
import math
import re
import time
from typing import Optional, Tuple

import rclpy
import serial
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class CountDiscontinuity(ValueError):
    """Raised after rebasing an invalid or implausible encoder sample."""


ENCODER_RESET_HEADERS = {
    ("left_ticks", "right_ticks"),
    ("rear_left_ticks", "front_right_ticks"),
}


@dataclass(frozen=True)
class EncoderMotion:
    left_distance_m: float
    right_distance_m: float
    center_distance_m: float
    delta_yaw: float
    linear_velocity_mps: float
    angular_velocity_rps: float


def parse_encoder_line(line: bytes | str) -> Tuple[int, int]:
    """Parse one ``left_ticks,right_ticks`` serial line."""
    try:
        text = line.decode("ascii") if isinstance(line, bytes) else str(line)
    except UnicodeDecodeError as exc:
        raise ValueError("encoder line is not ASCII") from exc

    text = text.strip()
    if not text:
        raise ValueError("encoder line is blank")

    fields = text.split(",")
    if len(fields) != 2:
        raise ValueError("encoder line must contain exactly two fields")
    fields = [field.strip() for field in fields]
    if not all(re.fullmatch(r"[+-]?\d+", field) for field in fields):
        raise ValueError("encoder fields must be signed decimal integers")
    return int(fields[0]), int(fields[1])


def is_encoder_reset_marker(line: bytes | str) -> bool:
    """Return true for the known Arduino startup/reset column headers."""
    try:
        text = line.decode("ascii") if isinstance(line, bytes) else str(line)
    except UnicodeDecodeError:
        return False
    fields = tuple(field.strip().lower() for field in text.strip().split(","))
    return fields in ENCODER_RESET_HEADERS


def split_serial_lines(
    pending: bytes, chunk: bytes, max_line_length: int
) -> Tuple[list[bytes], bytes, bool]:
    """Split complete newline-delimited records while retaining a partial tail."""
    combined = pending + chunk
    parts = combined.split(b"\n")
    remainder = parts.pop()
    discarded_overlong = False

    lines = []
    for line in parts:
        if len(line) > max_line_length:
            discarded_overlong = True
        else:
            lines.append(line)

    if len(remainder) > max_line_length:
        # Retain a bounded sentinel so all later fragments of this physical
        # record remain overlong and are discarded through the next newline.
        remainder = remainder[:max_line_length + 1]
        discarded_overlong = True
    return lines, remainder, discarded_overlong


def calculate_motion(
    delta_left_ticks: int,
    delta_right_ticks: int,
    dt: float,
    counts_per_revolution: float,
    wheel_diameter_m: float,
    track_width_m: float,
    left_direction: float = 1.0,
    right_direction: float = 1.0,
) -> EncoderMotion:
    """Convert corrected wheel-count deltas into distance and body velocity."""
    numeric_values = (
        dt,
        counts_per_revolution,
        wheel_diameter_m,
        track_width_m,
        left_direction,
        right_direction,
    )
    if not all(math.isfinite(float(value)) for value in numeric_values):
        raise ValueError("encoder calibration and timing values must be finite")
    if dt <= 0.0:
        raise ValueError("encoder sample interval must be positive")
    if (
        counts_per_revolution <= 0.0
        or wheel_diameter_m <= 0.0
        or track_width_m <= 0.0
    ):
        raise ValueError("encoder geometry values must be positive")
    if left_direction not in (-1.0, 1.0) or right_direction not in (-1.0, 1.0):
        raise ValueError("encoder direction values must be +1.0 or -1.0")

    metres_per_count = (
        math.pi * wheel_diameter_m / counts_per_revolution
    )
    left_distance = delta_left_ticks * left_direction * metres_per_count
    right_distance = delta_right_ticks * right_direction * metres_per_count
    left_velocity = left_distance / dt
    right_velocity = right_distance / dt
    center_distance = (left_distance + right_distance) * 0.5
    delta_yaw = (right_distance - left_distance) / track_width_m
    linear_velocity = (left_velocity + right_velocity) * 0.5
    angular_velocity = (right_velocity - left_velocity) / track_width_m

    outputs = (
        left_distance,
        right_distance,
        center_distance,
        delta_yaw,
        linear_velocity,
        angular_velocity,
    )
    if not all(math.isfinite(value) for value in outputs):
        raise ValueError("encoder calculation produced a non-finite result")
    return EncoderMotion(*outputs)


def integrate_midpoint_pose(
    x: float,
    y: float,
    yaw: float,
    center_distance_m: float,
    delta_yaw: float,
) -> Tuple[float, float, float]:
    """Integrate one differential-drive increment using midpoint heading."""
    midpoint_yaw = yaw + delta_yaw * 0.5
    x += center_distance_m * math.cos(midpoint_yaw)
    y += center_distance_m * math.sin(midpoint_yaw)
    yaw = math.atan2(math.sin(yaw + delta_yaw), math.cos(yaw + delta_yaw))
    return x, y, yaw


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def measurement_timed_out(
    last_measurement_time: Optional[float],
    now: float,
    timeout_sec: float,
) -> bool:
    return (
        last_measurement_time is not None
        and now - last_measurement_time >= timeout_sec
    )


class EncoderTracker:
    """ROS-independent encoder baseline, calibration and discontinuity state."""

    def __init__(
        self,
        counts_per_revolution: float,
        wheel_diameter_m: float,
        track_width_m: float,
        left_direction: float,
        right_direction: float,
        max_tick_jump: int,
        max_wheel_speed_mps: float,
    ) -> None:
        # Validate calibration once before any serial device is opened.
        calculate_motion(
            0,
            0,
            1.0,
            counts_per_revolution,
            wheel_diameter_m,
            track_width_m,
            left_direction,
            right_direction,
        )
        if max_tick_jump <= 0:
            raise ValueError("max_tick_jump must be positive")
        if (
            not math.isfinite(max_wheel_speed_mps)
            or max_wheel_speed_mps <= 0.0
        ):
            raise ValueError("max_wheel_speed_mps must be finite and positive")

        self.counts_per_revolution = counts_per_revolution
        self.wheel_diameter_m = wheel_diameter_m
        self.track_width_m = track_width_m
        self.left_direction = left_direction
        self.right_direction = right_direction
        self.max_tick_jump = max_tick_jump
        self.max_wheel_speed_mps = max_wheel_speed_mps
        self.previous_left: Optional[int] = None
        self.previous_right: Optional[int] = None
        self.previous_time: Optional[float] = None

    @property
    def metres_per_count(self) -> float:
        return (
            math.pi * self.wheel_diameter_m / self.counts_per_revolution
        )

    def reset(self) -> None:
        self.previous_left = None
        self.previous_right = None
        self.previous_time = None

    def update(
        self, left_ticks: int, right_ticks: int, timestamp: float
    ) -> Optional[EncoderMotion]:
        if not math.isfinite(timestamp):
            raise ValueError("encoder timestamp must be finite")

        if self.previous_left is None:
            self.previous_left = left_ticks
            self.previous_right = right_ticks
            self.previous_time = timestamp
            return None

        delta_left = left_ticks - self.previous_left
        delta_right = right_ticks - self.previous_right
        dt = timestamp - self.previous_time

        # Always rebase before validating so the next sample starts safely.
        self.previous_left = left_ticks
        self.previous_right = right_ticks
        self.previous_time = timestamp

        if dt <= 0.0:
            raise CountDiscontinuity("non-positive encoder sample interval")
        if (
            abs(delta_left) > self.max_tick_jump
            or abs(delta_right) > self.max_tick_jump
        ):
            raise CountDiscontinuity(
                f"tick jump left={delta_left}, right={delta_right}"
            )

        max_distance = (
            max(abs(delta_left), abs(delta_right)) * self.metres_per_count
        )
        if max_distance / dt > self.max_wheel_speed_mps:
            raise CountDiscontinuity(
                f"implied wheel speed {max_distance / dt:.3f} m/s"
            )

        return calculate_motion(
            delta_left,
            delta_right,
            dt,
            self.counts_per_revolution,
            self.wheel_diameter_m,
            self.track_width_m,
            self.left_direction,
            self.right_direction,
        )


class EncoderOdom(Node):
    """Read Arduino wheel counts and publish measured rover odometry."""

    def __init__(self) -> None:
        super().__init__("encoder_odom")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("counts_per_revolution", 1224.0)
        self.declare_parameter("wheel_diameter_m", 0.15875)
        self.declare_parameter("track_width_m", 0.352425)
        self.declare_parameter("left_direction", 1.0)
        self.declare_parameter("right_direction", 1.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("state_topic", "/vla/state")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("serial_timeout_sec", 0.05)
        self.declare_parameter("poll_rate_hz", 50.0)
        self.declare_parameter("measurement_timeout_sec", 0.5)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("max_tick_jump", 4096)
        self.declare_parameter("max_wheel_speed_mps", 2.0)
        self.declare_parameter("warning_throttle_sec", 5.0)
        self.declare_parameter("max_line_length", 128)
        self.declare_parameter("max_lines_per_poll", 32)
        self.declare_parameter("max_read_bytes", 4096)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        counts_per_revolution = float(
            self.get_parameter("counts_per_revolution").value
        )
        wheel_diameter_m = float(
            self.get_parameter("wheel_diameter_m").value
        )
        track_width_m = float(self.get_parameter("track_width_m").value)
        left_direction = float(self.get_parameter("left_direction").value)
        right_direction = float(self.get_parameter("right_direction").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.serial_timeout = float(
            self.get_parameter("serial_timeout_sec").value
        )
        poll_rate = float(self.get_parameter("poll_rate_hz").value)
        self.measurement_timeout = float(
            self.get_parameter("measurement_timeout_sec").value
        )
        self.reconnect_interval = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        max_tick_jump = int(self.get_parameter("max_tick_jump").value)
        max_wheel_speed_mps = float(
            self.get_parameter("max_wheel_speed_mps").value
        )
        self.warning_throttle = float(
            self.get_parameter("warning_throttle_sec").value
        )
        self.max_line_length = int(
            self.get_parameter("max_line_length").value
        )
        self.max_lines_per_poll = int(
            self.get_parameter("max_lines_per_poll").value
        )
        self.max_read_bytes = int(
            self.get_parameter("max_read_bytes").value
        )

        finite_positive = (
            self.serial_timeout,
            poll_rate,
            self.measurement_timeout,
            self.reconnect_interval,
            self.warning_throttle,
        )
        if not self.serial_port:
            raise ValueError("serial_port cannot be empty")
        if self.baud_rate <= 0:
            raise ValueError("baud_rate must be positive")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in finite_positive
        ):
            raise ValueError("encoder timing parameters must be finite and positive")
        if (
            self.max_line_length < 8
            or self.max_lines_per_poll <= 0
            or self.max_read_bytes <= 0
        ):
            raise ValueError("encoder serial buffer parameters are invalid")

        self.tracker = EncoderTracker(
            counts_per_revolution,
            wheel_diameter_m,
            track_width_m,
            left_direction,
            right_direction,
            max_tick_jump,
            max_wheel_speed_mps,
        )

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, qos)
        self.state_publisher = self.create_publisher(
            Float32MultiArray, state_topic, qos
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.serial = None
        self.pending_bytes = b""
        self.next_reconnect_time = 0.0
        self.last_measurement_time: Optional[float] = None
        self.timeout_zero_published = False
        self.closed = False
        self.last_warning_times = {}

        self.timer = self.create_timer(1.0 / poll_rate, self.poll_serial)
        self.attempt_connection(time.monotonic())

    def warn_throttled(self, key: str, message: str, now: float) -> None:
        last_time = self.last_warning_times.get(key)
        if (
            last_time is None
            or now - last_time >= self.warning_throttle
        ):
            self.get_logger().warning(message)
            self.last_warning_times[key] = now

    def attempt_connection(self, now: float) -> None:
        if (
            self.closed
            or self.serial is not None
            or now < self.next_reconnect_time
        ):
            return

        connection = None
        try:
            connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
            )
            connection.reset_input_buffer()
        except Exception as exc:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
            self.serial = None
            self.next_reconnect_time = now + self.reconnect_interval
            self.warn_throttled(
                "connect",
                f"Encoder serial open failed on {self.serial_port}: {exc}; "
                f"retrying in {self.reconnect_interval:.1f}s",
                now,
            )
            return

        self.serial = connection
        self.pending_bytes = b""
        self.tracker.reset()
        self.next_reconnect_time = 0.0
        self.get_logger().info(
            f"Connected to encoder Arduino on {self.serial_port} at "
            f"{self.baud_rate} baud"
        )

    def disconnect_serial(self, now: float, reason: str) -> None:
        connection = self.serial
        self.serial = None
        self.pending_bytes = b""
        self.tracker.reset()
        self.next_reconnect_time = now + self.reconnect_interval
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        self.warn_throttled(
            "disconnect",
            f"Encoder serial disconnected: {reason}; "
            f"retrying in {self.reconnect_interval:.1f}s",
            now,
        )

    def read_latest_counts(
        self, now: float
    ) -> Tuple[Optional[Tuple[int, int]], bool]:
        if self.serial is None:
            return None, False

        latest_counts = None
        reset_marker_seen = False
        lines_processed = 0
        try:
            while (
                lines_processed < self.max_lines_per_poll
                and self.serial.in_waiting > 0
            ):
                bytes_to_read = min(
                    int(self.serial.in_waiting), self.max_read_bytes
                )
                chunk = self.serial.read(bytes_to_read)
                if not chunk:
                    break
                lines, self.pending_bytes, overlong = split_serial_lines(
                    self.pending_bytes, chunk, self.max_line_length
                )
                remaining_capacity = (
                    self.max_lines_per_poll - lines_processed
                )
                if len(lines) > remaining_capacity:
                    # Keep the freshest complete samples if a backlog exists.
                    lines = lines[-remaining_capacity:]
                if overlong:
                    self.warn_throttled(
                        "malformed",
                        "Ignored overlong encoder serial line.",
                        now,
                    )
                for line in lines:
                    lines_processed += 1
                    if is_encoder_reset_marker(line):
                        # Discard any earlier numeric line in this batch. A
                        # numeric line after the marker becomes the baseline.
                        latest_counts = None
                        reset_marker_seen = True
                        self.warn_throttled(
                            "reset_marker",
                            "Encoder startup/reset marker received; resetting "
                            "the count baseline.",
                            now,
                        )
                        continue
                    try:
                        latest_counts = parse_encoder_line(line)
                    except ValueError as exc:
                        preview = repr(line[:64])
                        self.warn_throttled(
                            "malformed",
                            f"Ignored malformed encoder line {preview}: {exc}",
                            now,
                        )
                    if lines_processed >= self.max_lines_per_poll:
                        break
        except Exception as exc:
            self.disconnect_serial(now, str(exc))
            return None, False
        return latest_counts, reset_marker_seen

    def process_counts(
        self, left_ticks: int, right_ticks: int, now: float
    ) -> None:
        # If this sample arrived after the configured stale interval, do not
        # integrate an aggregate delta across the gap. Treat it as a baseline.
        if measurement_timed_out(
            self.last_measurement_time,
            now,
            self.measurement_timeout,
        ):
            self.tracker.reset()

        try:
            motion = self.tracker.update(left_ticks, right_ticks, now)
        except CountDiscontinuity as exc:
            self.last_measurement_time = now
            self.timeout_zero_published = False
            self.publish_measurement(0.0, 0.0)
            self.warn_throttled(
                "discontinuity",
                f"Encoder baseline reset: {exc}",
                now,
            )
            return

        self.last_measurement_time = now
        self.timeout_zero_published = False
        if motion is None:
            self.publish_measurement(0.0, 0.0)
            return

        self.x, self.y, self.yaw = integrate_midpoint_pose(
            self.x,
            self.y,
            self.yaw,
            motion.center_distance_m,
            motion.delta_yaw,
        )
        self.publish_measurement(
            motion.linear_velocity_mps,
            motion.angular_velocity_rps,
        )

    def publish_measurement(
        self, linear_velocity: float, angular_velocity: float
    ) -> None:
        values = (
            self.x,
            self.y,
            self.yaw,
            linear_velocity,
            angular_velocity,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error(
                "Refused to publish non-finite encoder odometry."
            )
            return

        now = self.get_clock().now()
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(odom)

        state = Float32MultiArray()
        state.data = [float(linear_velocity), float(angular_velocity)]
        self.state_publisher.publish(state)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def publish_timeout_zero_if_needed(self, now: float) -> None:
        if (
            measurement_timed_out(
                self.last_measurement_time,
                now,
                self.measurement_timeout,
            )
            and not self.timeout_zero_published
        ):
            self.publish_measurement(0.0, 0.0)
            self.timeout_zero_published = True
            self.tracker.reset()
            self.get_logger().warning(
                "Encoder measurement timed out; velocity set to zero and "
                "the next valid sample will establish a new baseline."
            )

    def poll_serial(self) -> None:
        if self.closed:
            return
        now = time.monotonic()
        if self.serial is None:
            self.attempt_connection(now)

        counts, reset_marker_seen = self.read_latest_counts(now)
        if reset_marker_seen:
            self.tracker.reset()
            self.last_measurement_time = now
            self.timeout_zero_published = False
            if counts is None:
                self.publish_measurement(0.0, 0.0)
        if counts is not None:
            self.process_counts(counts[0], counts[1], now)

        self.publish_timeout_zero_if_needed(time.monotonic())

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.last_measurement_time is not None and rclpy.ok():
            self.publish_measurement(0.0, 0.0)
        connection = self.serial
        self.serial = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        self.pending_bytes = b""
        self.tracker.reset()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderOdom()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"encoder_odom failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
