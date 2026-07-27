import math

import pytest

from rover_vla_core.serial_motor_driver import (
    command_is_stale,
    encode_motor_command,
    mix_legacy_twist,
)


@pytest.mark.parametrize(
    ("linear_x", "angular_z", "expected"),
    [
        (0.1, 0.0, (-2, -2)),
        (0.1, 0.4, (-1, -3)),
        (0.1, -0.4, (-3, -1)),
        (-0.1, 0.0, (2, 2)),
        (0.0, 1.0, (0, 0)),
        (100.0, 0.0, (-255, -255)),
    ],
)
def test_legacy_mixer_preserves_discovered_behavior(
    linear_x, angular_z, expected
):
    assert mix_legacy_twist(linear_x, angular_z) == expected


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_legacy_mixer_rejects_non_finite_values(value):
    with pytest.raises(ValueError):
        mix_legacy_twist(value, 0.0)


def test_encoder_uses_one_canonical_wire_format():
    assert (
        encode_motor_command(-2, -3)
        == b"000000000000000r-2l-3\n"
    )
    assert encode_motor_command(0, 0) == b"000000000000000r0l0\n"


def test_encoder_rejects_multiline_prefix():
    with pytest.raises(ValueError):
        encode_motor_command(0, 0, "bad\nprefix")


def test_command_watchdog():
    assert command_is_stale(None, now=10.0, timeout_sec=0.5)
    assert not command_is_stale(10.0, now=10.49, timeout_sec=0.5)
    assert command_is_stale(10.0, now=10.5, timeout_sec=0.5)
