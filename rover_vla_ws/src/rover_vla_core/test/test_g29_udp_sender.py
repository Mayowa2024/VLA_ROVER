import json
import struct

import pytest

from rover_vla_core.g29_udp_sender import (
    apply_deadzone,
    calculate_command,
    DeadmanGate,
    decode_events,
    encode_twist_packet,
    JS_EVENT_AXIS,
    JS_EVENT_BUTTON,
    JS_EVENT_FORMAT,
    JS_EVENT_INIT,
    normalize_axis,
    pedal_position,
)
from rover_vla_core.teleop_adapter import parse_twist_datagram


def test_deadman_requires_release_then_press():
    gate = DeadmanGate()
    assert not gate.update(True)
    assert not gate.update(False)
    assert gate.update(True)
    assert not gate.update(False)


def test_normalize_axis_clamps_signed_16_bit_range():
    assert normalize_axis(-32768) == -1.0
    assert normalize_axis(0) == 0.0
    assert normalize_axis(32767) == 1.0


def test_deadzone_is_zero_at_centre_and_preserves_full_scale():
    assert apply_deadzone(0.02, 0.03) == 0.0
    assert apply_deadzone(-0.03, 0.03) == 0.0
    assert apply_deadzone(1.0, 0.03) == 1.0
    assert apply_deadzone(-1.0, 0.03) == -1.0
    with pytest.raises(ValueError):
        apply_deadzone(0.0, 1.0)


def test_pedal_mapping_and_inversion():
    assert pedal_position(32767) == 0.0
    assert pedal_position(-32768) == 1.0
    assert pedal_position(-32768, invert=True) == 0.0
    assert pedal_position(32767, invert=True) == 1.0


def test_disabled_command_is_zero_but_retains_status_values():
    linear, angular, steering, accelerator, brake = calculate_command(
        steering_raw=-32768,
        accelerator_raw=-32768,
        brake_raw=32767,
        enabled=False,
        steering_deadzone=0.03,
        steering_sign=-1.0,
        accelerator_invert=False,
        brake_invert=False,
        max_linear_mps=0.1,
        max_angular_rps=0.4,
    )
    assert (linear, angular) == (0.0, 0.0)
    assert steering == -1.0
    assert accelerator == 1.0
    assert brake == 0.0


def test_brake_reduces_forward_command_and_never_reverses():
    full_speed = calculate_command(
        0,
        -32768,
        32767,
        enabled=True,
        steering_deadzone=0.03,
        steering_sign=-1.0,
        accelerator_invert=False,
        brake_invert=False,
        max_linear_mps=0.1,
        max_angular_rps=0.4,
    )
    stopped = calculate_command(
        0,
        -32768,
        -32768,
        enabled=True,
        steering_deadzone=0.03,
        steering_sign=-1.0,
        accelerator_invert=False,
        brake_invert=False,
        max_linear_mps=0.1,
        max_angular_rps=0.4,
    )
    assert full_speed[:2] == (0.1, -0.0)
    assert stopped[0] == 0.0


def test_encode_twist_packet_matches_receiver_shape():
    packet = encode_twist_packet(0.1, -0.2)
    assert json.loads(packet) == {
        'linear': {'x': 0.1},
        'angular': {'z': -0.2},
    }
    assert parse_twist_datagram(packet) == (0.1, -0.2)


def test_decode_events_preserves_partial_tail_and_init_flag():
    axis = struct.pack(
        JS_EVENT_FORMAT, 10, -123, JS_EVENT_AXIS | JS_EVENT_INIT, 2
    )
    button = struct.pack(JS_EVENT_FORMAT, 20, 1, JS_EVENT_BUTTON, 4)
    events, tail = decode_events(axis + button + b'\x01\x02')

    assert len(events) == 2
    assert events[0].event_type == JS_EVENT_AXIS
    assert events[0].number == 2
    assert events[0].value == -123
    assert events[0].initial
    assert events[1].event_type == JS_EVENT_BUTTON
    assert events[1].number == 4
    assert not events[1].initial
    assert tail == b'\x01\x02'
