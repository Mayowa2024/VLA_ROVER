import os

import pytest
import rclpy
import serial

import rover_vla_core.encoder_odom as encoder_module
from rover_vla_core.encoder_odom import EncoderOdom


pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_NODE_TESTS") != "1",
    reason="set RUN_ROS_NODE_TESTS=1 in an environment that permits DDS sockets",
)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeEncoderSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.buffer = bytearray()
        self.closed = False
        self.read_error = None

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def feed(self, data):
        self.buffer.extend(data)

    def read(self, size):
        if self.read_error is not None:
            error = self.read_error
            self.read_error = None
            raise error
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        self.closed = True


def initialize_node(monkeypatch, serial_factory, clock):
    monkeypatch.setattr(serial, "Serial", serial_factory)
    monkeypatch.setattr(
        encoder_module.time, "monotonic", lambda: clock[0]
    )
    rclpy.init(args=[
        "--ros-args",
        "-p", "serial_port:=/dev/fake-encoder",
        "-p", "poll_rate_hz:=100.0",
        "-p", "measurement_timeout_sec:=0.5",
        "-p", "reconnect_interval_sec:=0.1",
        "-p", "max_tick_jump:=1000",
        "-p", "max_wheel_speed_mps:=10.0",
    ])
    return EncoderOdom()


def test_encoder_node_baseline_motion_timeout_and_close(monkeypatch):
    clock = [10.0]
    fake_serial = FakeEncoderSerial()
    node = None
    try:
        node = initialize_node(
            monkeypatch, lambda **kwargs: fake_serial, clock
        )
        odom_capture = CapturePublisher()
        state_capture = CapturePublisher()
        node.odom_publisher = odom_capture
        node.state_publisher = state_capture

        fake_serial.feed(b"left_ticks,right_ticks\n1000,2000\n")
        node.poll_serial()
        assert len(odom_capture.messages) == 1
        assert odom_capture.messages[-1].twist.twist.linear.x == 0.0
        assert list(state_capture.messages[-1].data) == [0.0, 0.0]

        clock[0] = 10.1
        fake_serial.feed(b"1100,2100\n")
        node.poll_serial()
        odom = odom_capture.messages[-1]
        assert odom.header.frame_id == "odom"
        assert odom.child_frame_id == "base_link"
        assert odom.twist.twist.linear.x == pytest.approx(
            0.407457380520735
        )
        assert odom.twist.twist.angular.z == pytest.approx(0.0, abs=1e-9)
        assert state_capture.messages[-1].data[0] == pytest.approx(
            0.407457380520735, rel=1e-6
        )

        message_count = len(odom_capture.messages)
        clock[0] = 10.6
        node.poll_serial()
        assert len(odom_capture.messages) == message_count + 1
        assert odom_capture.messages[-1].twist.twist.linear.x == 0.0

        clock[0] = 10.7
        node.poll_serial()
        assert len(odom_capture.messages) == message_count + 1

        # Timeout cleared the baseline, so recovery starts at zero velocity.
        fake_serial.feed(b"5000,6000\n")
        node.poll_serial()
        assert odom_capture.messages[-1].twist.twist.linear.x == 0.0

        node.destroy_node()
        node = None
        assert fake_serial.closed
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_encoder_node_reconnects_and_rebaselines(monkeypatch):
    clock = [20.0]
    first = FakeEncoderSerial()
    second = FakeEncoderSerial()
    connections = [first, second]

    def serial_factory(**kwargs):
        connection = connections.pop(0)
        connection.kwargs = kwargs
        return connection

    node = None
    try:
        node = initialize_node(monkeypatch, serial_factory, clock)
        node.odom_publisher = CapturePublisher()
        node.state_publisher = CapturePublisher()

        first.feed(b"100,100\n")
        node.poll_serial()
        first.read_error = serial.SerialException("disconnected")
        first.feed(b"x")
        clock[0] = 20.1
        node.poll_serial()
        assert first.closed
        assert node.serial is None

        clock[0] = 20.19
        node.poll_serial()
        assert node.serial is None

        clock[0] = 20.21
        node.poll_serial()
        assert node.serial is second
        assert second.kwargs["port"] == "/dev/fake-encoder"
        assert second.kwargs["baudrate"] == 115200
        assert second.kwargs["timeout"] == pytest.approx(0.05)

        second.feed(b"5000,5000\n")
        node.poll_serial()
        assert node.odom_publisher.messages[-1].twist.twist.linear.x == 0.0

        clock[0] = 20.31
        second.feed(b"5010,5010\n")
        node.poll_serial()
        assert (
            node.odom_publisher.messages[-1].twist.twist.linear.x
            == pytest.approx(0.0407457380520735)
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_arduino_reset_header_rebaselines_without_velocity_spike(monkeypatch):
    clock = [40.0]
    fake_serial = FakeEncoderSerial()
    node = None
    try:
        node = initialize_node(
            monkeypatch, lambda **kwargs: fake_serial, clock
        )
        node.odom_publisher = CapturePublisher()
        node.state_publisher = CapturePublisher()

        fake_serial.feed(b"100,100\n")
        node.poll_serial()
        clock[0] = 40.1
        fake_serial.feed(b"110,110\n")
        node.poll_serial()
        pose_before_reset = (
            node.odom_publisher.messages[-1].pose.pose.position.x
        )

        clock[0] = 40.2
        fake_serial.feed(b"left_ticks,right_ticks\n0,0\n")
        node.poll_serial()
        reset_odom = node.odom_publisher.messages[-1]
        assert reset_odom.twist.twist.linear.x == 0.0
        assert reset_odom.pose.pose.position.x == pytest.approx(
            pose_before_reset
        )

        clock[0] = 40.3
        fake_serial.feed(b"10,10\n")
        node.poll_serial()
        recovered_odom = node.odom_publisher.messages[-1]
        assert recovered_odom.twist.twist.linear.x == pytest.approx(
            0.0407457380520735
        )
        assert recovered_odom.pose.pose.position.x > pose_before_reset
        assert all(
            message.twist.twist.linear.x >= 0.0
            for message in node.odom_publisher.messages
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_initial_open_failure_retries_without_crashing(monkeypatch):
    clock = [50.0]
    recovered_serial = FakeEncoderSerial()
    attempts = []

    def serial_factory(**kwargs):
        attempts.append(kwargs)
        if len(attempts) == 1:
            raise serial.SerialException("device is not ready")
        recovered_serial.kwargs = kwargs
        return recovered_serial

    node = None
    try:
        node = initialize_node(monkeypatch, serial_factory, clock)
        node.odom_publisher = CapturePublisher()
        node.state_publisher = CapturePublisher()
        assert node.serial is None

        clock[0] = 50.09
        node.poll_serial()
        assert len(attempts) == 1

        clock[0] = 50.11
        node.poll_serial()
        assert node.serial is recovered_serial
        assert len(attempts) == 2

        recovered_serial.feed(b"100,100\n")
        node.poll_serial()
        assert node.odom_publisher.messages[-1].twist.twist.linear.x == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
