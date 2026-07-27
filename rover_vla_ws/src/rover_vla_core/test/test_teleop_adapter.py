import json

import pytest

from rover_vla_core.teleop_adapter import parse_twist_datagram, timeout_expired


def make_packet(linear_x, angular_z):
    return json.dumps({
        "linear": {"x": linear_x},
        "angular": {"z": angular_z},
    }).encode("utf-8")


def test_parse_twist_datagram():
    assert parse_twist_datagram(make_packet(0.1, -0.4)) == (0.1, -0.4)


@pytest.mark.parametrize(
    "packet",
    [
        b"",
        b"\xff",
        b"[]",
        b"{}",
        b'{"linear": null, "angular": {"z": 0.0}}',
        b'{"linear": {"x": 0.1}, "angular": {}}',
        make_packet("0.1", 0.0),
        make_packet(True, 0.0),
        make_packet(float("nan"), 0.0),
        make_packet(0.0, float("inf")),
    ],
)
def test_parse_twist_datagram_rejects_invalid_packets(packet):
    with pytest.raises(ValueError):
        parse_twist_datagram(packet)


def test_timeout_requires_a_valid_command_and_includes_boundary():
    assert not timeout_expired(None, now=10.0, timeout_sec=0.5)
    assert not timeout_expired(10.0, now=10.49, timeout_sec=0.5)
    assert timeout_expired(10.0, now=10.5, timeout_sec=0.5)
