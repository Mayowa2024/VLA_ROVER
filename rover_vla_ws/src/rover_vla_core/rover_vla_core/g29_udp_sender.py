#!/usr/bin/env python3
"""Read a Linux joystick and send safe UDP teleoperation commands."""

import argparse
from dataclasses import dataclass
import errno
import fcntl
import json
import math
import os
import signal
import socket
import struct
import sys
import time
from typing import Dict, Iterable, List, Tuple


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FORMAT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JSIOCGNAME_BASE = 0x80006A13


@dataclass(frozen=True)
class JoystickEvent:
    """One event from the Linux joystick API."""

    timestamp_ms: int
    value: int
    event_type: int
    number: int
    initial: bool


class DeadmanGate:
    """Require a release after startup before a press can arm control."""

    def __init__(self) -> None:
        self.release_seen = False

    def update(self, pressed: bool) -> bool:
        """Return whether the deadman is safely armed."""
        if not pressed:
            self.release_seen = True
            return False
        return self.release_seen


def normalize_axis(raw_value: int) -> float:
    """Map a signed 16-bit joystick value to the closed interval [-1, 1]."""
    return max(-1.0, min(1.0, int(raw_value) / 32767.0))


def apply_deadzone(value: float, deadzone: float) -> float:
    """Remove a centred deadzone while retaining the full output range."""
    value = max(-1.0, min(1.0, float(value)))
    deadzone = float(deadzone)
    if not math.isfinite(deadzone) or not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be finite and in [0, 1)")
    if abs(value) <= deadzone:
        return 0.0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(magnitude, value)


def pedal_position(raw_value: int, invert: bool = False) -> float:
    """
    Return pedal depression in [0, 1].

    A G29 normally reports +32767 when released and -32767 when pressed.
    ``invert`` supports drivers that expose the opposite direction.
    """
    normalized = normalize_axis(raw_value)
    if invert:
        normalized *= -1.0
    return max(0.0, min(1.0, (1.0 - normalized) / 2.0))


def calculate_command(
    steering_raw: int,
    accelerator_raw: int,
    brake_raw: int,
    *,
    enabled: bool,
    steering_deadzone: float,
    steering_sign: float,
    accelerator_invert: bool,
    brake_invert: bool,
    max_linear_mps: float,
    max_angular_rps: float,
) -> Tuple[float, float, float, float, float]:
    """
    Convert G29 inputs into linear and angular commands.

    The brake reduces the forward command; it never requests reverse motion.
    """
    steering = apply_deadzone(
        normalize_axis(steering_raw), steering_deadzone
    )
    accelerator = pedal_position(accelerator_raw, accelerator_invert)
    brake = pedal_position(brake_raw, brake_invert)

    if not enabled:
        return 0.0, 0.0, steering, accelerator, brake

    linear_x = max_linear_mps * accelerator * (1.0 - brake)
    angular_z = max_angular_rps * steering_sign * steering
    return linear_x, angular_z, steering, accelerator, brake


def encode_twist_packet(linear_x: float, angular_z: float) -> bytes:
    """Encode the JSON format accepted by ``teleop_adapter``."""
    values = (float(linear_x), float(angular_z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("command values must be finite")
    return json.dumps(
        {
            "linear": {"x": values[0]},
            "angular": {"z": values[1]},
        },
        separators=(",", ":"),
    ).encode("utf-8")


def decode_events(data: bytes) -> Tuple[List[JoystickEvent], bytes]:
    """Decode complete Linux joystick events and return any partial tail."""
    events = []
    complete_length = len(data) - len(data) % JS_EVENT_SIZE
    for offset in range(0, complete_length, JS_EVENT_SIZE):
        timestamp_ms, value, raw_type, number = struct.unpack_from(
            JS_EVENT_FORMAT, data, offset
        )
        events.append(
            JoystickEvent(
                timestamp_ms=timestamp_ms,
                value=value,
                event_type=raw_type & ~JS_EVENT_INIT,
                number=number,
                initial=bool(raw_type & JS_EVENT_INIT),
            )
        )
    return events, data[complete_length:]


class LinuxJoystick:
    """Small non-blocking wrapper around ``/dev/input/js*``."""

    def __init__(self, device: str) -> None:
        self.device = device
        try:
            self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError as exc:
            raise RuntimeError(
                f"Permission denied opening {device}; check its permissions "
                "or run the test from a logged-in desktop session."
            ) from exc
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"{device} does not exist; connect the wheel and check "
                "'ls -l /dev/input/js*'."
            ) from exc
        self.pending = bytearray()

    @property
    def name(self) -> str:
        buffer = bytearray(128)
        request = JSIOCGNAME_BASE + (len(buffer) << 16)
        try:
            fcntl.ioctl(self.fd, request, buffer)
        except OSError:
            return os.path.basename(self.device)
        return bytes(buffer).split(b"\0", 1)[0].decode(
            "utf-8", errors="replace"
        )

    def read_available(self) -> List[JoystickEvent]:
        while True:
            try:
                chunk = os.read(self.fd, JS_EVENT_SIZE * 64)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.ENODEV, errno.EIO):
                    raise RuntimeError(
                        f"Joystick {self.device} disconnected."
                    ) from exc
                raise
            if not chunk:
                raise RuntimeError(f"Joystick {self.device} disconnected.")
            self.pending.extend(chunk)

        events, tail = decode_events(bytes(self.pending))
        self.pending = bytearray(tail)
        return events

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


def update_state(
    events: Iterable[JoystickEvent],
    axes: Dict[int, int],
    buttons: Dict[int, bool],
) -> None:
    """Apply joystick events to the current input state."""
    for event in events:
        if event.event_type == JS_EVENT_AXIS:
            axes[event.number] = event.value
        elif event.event_type == JS_EVENT_BUTTON:
            buttons[event.number] = bool(event.value)


def monitor(device: str) -> int:
    """Print changed axis and button numbers for input mapping."""
    with LinuxJoystick(device) as joystick:
        print(f"Monitoring {joystick.name} at {device}")
        print("Move one control at a time; press Ctrl+C when finished.")
        last_axis_print: Dict[int, int] = {}
        while True:
            events = joystick.read_available()
            for event in events:
                if event.event_type == JS_EVENT_BUTTON:
                    state = "pressed" if event.value else "released"
                    prefix = "initial " if event.initial else ""
                    print(f"{prefix}button {event.number}: {state}")
                elif event.event_type == JS_EVENT_AXIS:
                    previous = last_axis_print.get(event.number)
                    changed_enough = (
                        previous is None or abs(event.value - previous) >= 1024
                    )
                    if event.initial or changed_enough:
                        prefix = "initial " if event.initial else ""
                        value = normalize_axis(event.value)
                        print(
                            f"{prefix}axis {event.number}: "
                            f"{value:+.3f} (raw {event.value})"
                        )
                        last_axis_print[event.number] = event.value
            time.sleep(0.01)


def send_commands(args: argparse.Namespace) -> int:
    """Read the wheel at a fixed rate and send UDP commands."""
    required_axes = {
        args.steering_axis,
        args.accelerator_axis,
        args.brake_axis,
    }
    if len(required_axes) != 3:
        raise ValueError(
            "steering, accelerator and brake must use different axes"
        )
    axes: Dict[int, int] = {}
    buttons: Dict[int, bool] = {}
    deadman_gate = DeadmanGate()
    period = 1.0 / args.rate_hz
    stop_packet = encode_twist_packet(0.0, 0.0)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.connect((args.host, args.port))
    source_ip, source_port = udp_socket.getsockname()
    destination_ip, destination_port = udp_socket.getpeername()

    with LinuxJoystick(args.device) as joystick:
        print(f"Controller: {joystick.name} at {args.device}")
        print(
            f"UDP: {source_ip}:{source_port} -> "
            f"{destination_ip}:{destination_port}"
        )
        if args.no_deadman:
            print(
                "WARNING: deadman is disabled; use only while rover motors "
                "are disabled."
            )
        else:
            print(
                f"Hold button {args.deadman_button} to enable commands."
            )

        next_send = time.monotonic()
        next_status = next_send
        waiting_reported = False
        try:
            while True:
                update_state(joystick.read_available(), axes, buttons)
                now = time.monotonic()
                if now < next_send:
                    time.sleep(min(next_send - now, 0.005))
                    continue

                axes_ready = required_axes.issubset(axes)
                deadman_ready = (
                    args.no_deadman
                    or args.deadman_button in buttons
                )
                ready = axes_ready and deadman_ready
                if ready:
                    if args.no_deadman:
                        enabled = True
                    else:
                        enabled = deadman_gate.update(
                            buttons.get(args.deadman_button, False)
                        )
                    (
                        linear_x,
                        angular_z,
                        steering,
                        accelerator,
                        brake,
                    ) = calculate_command(
                        axes[args.steering_axis],
                        axes[args.accelerator_axis],
                        axes[args.brake_axis],
                        enabled=enabled,
                        steering_deadzone=args.steering_deadzone,
                        steering_sign=args.steering_sign,
                        accelerator_invert=args.accelerator_invert,
                        brake_invert=args.brake_invert,
                        max_linear_mps=args.max_linear_mps,
                        max_angular_rps=args.max_angular_rps,
                    )
                    packet = encode_twist_packet(linear_x, angular_z)
                    udp_socket.send(packet)
                    waiting_reported = False
                    if now >= next_status:
                        deadman = "ON" if enabled else "off"
                        print(
                            "\r"
                            f"deadman={deadman:>3} "
                            f"steer={steering:+.2f} "
                            f"accelerator={accelerator:.2f} "
                            f"brake={brake:.2f} "
                            f"cmd=({linear_x:+.3f}, {angular_z:+.3f})",
                            end="",
                            flush=True,
                        )
                        next_status = now + 0.25
                else:
                    udp_socket.send(stop_packet)
                    if not waiting_reported:
                        missing = sorted(required_axes.difference(axes))
                        needs = []
                        if missing:
                            needs.append(f"axes {missing}")
                        if not deadman_ready:
                            needs.append(
                                f"button {args.deadman_button}"
                            )
                        description = " and ".join(needs)
                        print(
                            f"Waiting for initial values from {description}."
                        )
                        waiting_reported = True

                next_send += period
                if next_send <= now:
                    next_send = now + period
        finally:
            for _ in range(3):
                try:
                    udp_socket.send(stop_packet)
                except OSError:
                    break
                time.sleep(0.02)
            print("\nStopped; zero command sent.")
            udp_socket.close()


def raise_keyboard_interrupt(_signum, _frame) -> None:
    """Turn termination into the same safe shutdown path as Ctrl+C."""
    raise KeyboardInterrupt


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return parsed


def fraction(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or not 0.0 <= parsed < 1.0:
        raise argparse.ArgumentTypeError("must be in [0, 1)")
    return parsed


def axis_number(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def udp_port(value: str) -> int:
    parsed = int(value)
    if not 1 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("must be between 1 and 65535")
    return parsed


def make_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Monitor a Linux joystick or send Logitech G29 controls to the "
            "rover's UDP teleop adapter."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    monitor_parser = subparsers.add_parser(
        "monitor", help="show axis and button numbers"
    )
    monitor_parser.add_argument(
        "--device", default="/dev/input/js0", help="Linux joystick device"
    )

    send_parser = subparsers.add_parser(
        "send", help="send steering and pedal commands over UDP"
    )
    send_parser.add_argument("host", help="Jetson IPv4 or hostname")
    send_parser.add_argument("--port", type=udp_port, default=6001)
    send_parser.add_argument(
        "--device", default="/dev/input/js0", help="Linux joystick device"
    )
    send_parser.add_argument("--steering-axis", type=axis_number, default=0)
    send_parser.add_argument(
        "--accelerator-axis", type=axis_number, default=2
    )
    send_parser.add_argument("--brake-axis", type=axis_number, default=3)
    deadman_group = send_parser.add_mutually_exclusive_group(required=True)
    deadman_group.add_argument(
        "--deadman-button",
        type=axis_number,
        help="button that must be held to enable non-zero commands",
    )
    deadman_group.add_argument(
        "--no-deadman",
        action="store_true",
        help="disable deadman only for motor-disabled testing",
    )
    send_parser.add_argument(
        "--rate-hz", type=positive_float, default=30.0
    )
    send_parser.add_argument(
        "--max-linear-mps", type=positive_float, default=0.10
    )
    send_parser.add_argument(
        "--max-angular-rps", type=positive_float, default=0.40
    )
    send_parser.add_argument(
        "--steering-deadzone", type=fraction, default=0.03
    )
    send_parser.add_argument(
        "--steering-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=-1.0,
        help="-1 maps wheel-left to positive ROS angular.z",
    )
    send_parser.add_argument(
        "--accelerator-invert",
        action="store_true",
        help="use when the accelerator reports -1 released and +1 pressed",
    )
    send_parser.add_argument(
        "--brake-invert",
        action="store_true",
        help="use when the brake reports -1 released and +1 pressed",
    )
    return parser


def main(argv=None) -> int:
    """Run the command-line sender."""
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)
    parser = make_argument_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "monitor":
            return monitor(args.device)
        return send_commands(args)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"g29_udp_sender: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
