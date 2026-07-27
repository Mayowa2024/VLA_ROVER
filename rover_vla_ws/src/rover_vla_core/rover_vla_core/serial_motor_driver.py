#!/usr/bin/env python3
"""Send safety-filtered ROS Twist commands to the rover's serial controller."""

import math
import time
from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LEGACY_COMMAND_PREFIX = "000000000000000"


def mix_legacy_twist(
    linear_x: float,
    angular_z: float,
    command_scale: float = 20.0,
    max_abs_command: int = 255,
    invert_output: bool = True,
) -> Tuple[int, int]:
    """Preserve the discovered rover mixer while making calibration explicit."""
    values = (linear_x, angular_z, command_scale)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("twist and scale values must be finite")

    left = int(round((linear_x - angular_z * linear_x) * command_scale))
    right = int(round((linear_x + angular_z * linear_x) * command_scale))
    if invert_output:
        left *= -1
        right *= -1

    limit = abs(int(max_abs_command))
    left = max(-limit, min(limit, left))
    right = max(-limit, min(limit, right))
    return left, right


def encode_motor_command(
    left: int,
    right: int,
    prefix: str = LEGACY_COMMAND_PREFIX,
) -> bytes:
    """Encode one command using the format used by the copied movement path."""
    if "\n" in prefix or "\r" in prefix:
        raise ValueError("command prefix cannot contain a line break")
    return f"{prefix}r{int(left)}l{int(right)}\n".encode("ascii")


def command_is_stale(
    last_command_time: float | None, now: float, timeout_sec: float
) -> bool:
    return last_command_time is None or now - last_command_time >= timeout_sec


class SerialMotorDriver(Node):
    """Translate safe body commands and write them to the motor controller."""

    def __init__(self) -> None:
        super().__init__("serial_motor_driver")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("serial_timeout_sec", 1.0)
        self.declare_parameter("write_timeout_sec", 0.1)
        self.declare_parameter("startup_delay_sec", 2.0)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("command_scale", 20.0)
        self.declare_parameter("max_abs_command", 255)
        self.declare_parameter("invert_output", True)
        self.declare_parameter("command_prefix", LEGACY_COMMAND_PREFIX)

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        serial_timeout = float(
            self.get_parameter("serial_timeout_sec").value
        )
        write_timeout = float(self.get_parameter("write_timeout_sec").value)
        startup_delay = float(self.get_parameter("startup_delay_sec").value)
        send_rate = float(self.get_parameter("send_rate_hz").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.command_scale = float(self.get_parameter("command_scale").value)
        self.max_abs_command = abs(
            int(self.get_parameter("max_abs_command").value)
        )
        self.invert_output = bool(self.get_parameter("invert_output").value)
        self.command_prefix = str(self.get_parameter("command_prefix").value)

        numeric_parameters = (
            serial_timeout,
            write_timeout,
            startup_delay,
            send_rate,
            self.command_timeout,
            self.command_scale,
        )
        if not all(math.isfinite(value) for value in numeric_parameters):
            raise ValueError("serial motor numeric parameters must be finite")
        if not serial_port:
            raise ValueError("serial_port cannot be empty")
        if baud_rate <= 0:
            raise ValueError("baud_rate must be positive")
        if serial_timeout < 0.0 or startup_delay < 0.0:
            raise ValueError("serial and startup timeouts cannot be negative")
        if write_timeout <= 0.0:
            raise ValueError("write_timeout_sec must be positive")
        if send_rate <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError("send rate and command timeout must be positive")

        stop_payload = encode_motor_command(0, 0, self.command_prefix)

        # Import locally so pure protocol helpers remain testable without hardware.
        import serial

        self.serial = None
        try:
            self.serial = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=serial_timeout,
                write_timeout=write_timeout,
            )
            # Stop immediately on open, before any controller reset delay.
            self.serial.write(stop_payload)
            if startup_delay:
                time.sleep(startup_delay)
                self.serial.write(stop_payload)
        except Exception as exc:
            if self.serial is not None:
                self.serial.close()
            raise RuntimeError(
                f"Could not open motor serial port {serial_port}: {exc}"
            ) from exc

        self.latest_command = (0.0, 0.0)
        self.last_command_time: float | None = None
        self.closed = False
        self.create_subscription(Twist, cmd_topic, self.command_callback, 10)
        self.timer = self.create_timer(1.0 / send_rate, self.send_motor_command)

        self.get_logger().warning(
            f"REAL motor driver active: {cmd_topic} -> {serial_port} at "
            f"{baud_rate} baud"
        )

    def command_callback(self, msg: Twist) -> None:
        command = (float(msg.linear.x), float(msg.angular.z))
        if not all(math.isfinite(value) for value in command):
            self.get_logger().error("Rejected non-finite motor command.")
            return
        self.latest_command = command
        self.last_command_time = time.monotonic()

    def write_wheels(self, left: int, right: int) -> bool:
        payload = encode_motor_command(left, right, self.command_prefix)
        try:
            self.serial.write(payload)
            return True
        except Exception as exc:
            self.get_logger().error(f"Motor serial write failed: {exc}")
            return False

    def send_motor_command(self) -> None:
        now = time.monotonic()
        if command_is_stale(
            self.last_command_time, now, self.command_timeout
        ):
            self.write_wheels(0, 0)
            return

        linear_x, angular_z = self.latest_command
        try:
            left, right = mix_legacy_twist(
                linear_x,
                angular_z,
                command_scale=self.command_scale,
                max_abs_command=self.max_abs_command,
                invert_output=self.invert_output,
            )
        except ValueError as exc:
            self.get_logger().error(f"Motor command conversion failed: {exc}")
            self.write_wheels(0, 0)
            return
        self.write_wheels(left, right)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.write_wheels(0, 0)
        finally:
            self.serial.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SerialMotorDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"serial_motor_driver failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
