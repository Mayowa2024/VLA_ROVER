import os
import time

import pytest
import rclpy
import serial
from geometry_msgs.msg import Twist

from rover_vla_core.serial_motor_driver import SerialMotorDriver


pytestmark = pytest.mark.skipif(
    os.environ.get("RUN_ROS_NODE_TESTS") != "1",
    reason="set RUN_ROS_NODE_TESTS=1 in an environment that permits DDS sockets",
)


class FakeSerial:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.writes = []
        self.closed = False
        self.instances.append(self)

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)

    def close(self):
        self.closed = True


def test_serial_node_lifecycle_without_hardware(monkeypatch):
    FakeSerial.instances.clear()
    monkeypatch.setattr(serial, "Serial", FakeSerial)
    rclpy.init(args=[
        "--ros-args",
        "-p", "serial_port:=/dev/fake-motor",
        "-p", "startup_delay_sec:=0.0",
    ])
    node = None
    try:
        node = SerialMotorDriver()
        fake_serial = FakeSerial.instances[0]
        stop = b"000000000000000r0l0\n"

        assert fake_serial.kwargs["port"] == "/dev/fake-motor"
        assert fake_serial.kwargs["baudrate"] == 9600
        assert fake_serial.writes == [stop]

        command = Twist()
        command.linear.x = 0.1
        command.angular.z = 0.4
        node.command_callback(command)
        node.send_motor_command()
        assert fake_serial.writes[-1] == b"000000000000000r-1l-3\n"

        node.last_command_time = time.monotonic() - 1.0
        node.send_motor_command()
        assert fake_serial.writes[-1] == stop

        node.destroy_node()
        node = None
        assert fake_serial.writes[-1] == stop
        assert fake_serial.closed
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
