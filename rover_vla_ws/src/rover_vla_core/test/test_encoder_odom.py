import math

import pytest

from rover_vla_core.encoder_odom import (
    CountDiscontinuity,
    EncoderTracker,
    calculate_motion,
    integrate_midpoint_pose,
    is_encoder_reset_marker,
    measurement_timed_out,
    parse_encoder_line,
    split_serial_lines,
    yaw_to_quaternion,
)


COUNTS_PER_REVOLUTION = 1224.0
WHEEL_DIAMETER_M = 0.15875
TRACK_WIDTH_M = 0.352425
METRES_PER_COUNT = math.pi * WHEEL_DIAMETER_M / COUNTS_PER_REVOLUTION


def make_tracker(max_tick_jump=500, max_wheel_speed_mps=10.0):
    return EncoderTracker(
        counts_per_revolution=COUNTS_PER_REVOLUTION,
        wheel_diameter_m=WHEEL_DIAMETER_M,
        track_width_m=TRACK_WIDTH_M,
        left_direction=1.0,
        right_direction=1.0,
        max_tick_jump=max_tick_jump,
        max_wheel_speed_mps=max_wheel_speed_mps,
    )


@pytest.mark.parametrize(
    ("line", "expected"),
    [
        (b"100,200\r\n", (100, 200)),
        (b" -10, +20 \n", (-10, 20)),
        ("0,0", (0, 0)),
    ],
)
def test_parse_encoder_line(line, expected):
    assert parse_encoder_line(line) == expected


@pytest.mark.parametrize(
    "line",
    [
        b"",
        b"  ",
        b"left_ticks,right_ticks",
        b"100",
        b"100,200,300",
        b",200",
        b"100.0,200",
        b"1_000,200",
        b"nan,200",
        b"\xff,200",
    ],
)
def test_parse_encoder_line_rejects_invalid_input(line):
    with pytest.raises(ValueError):
        parse_encoder_line(line)


def test_split_serial_lines_handles_fragments_and_multiple_records():
    lines, pending, overlong = split_serial_lines(
        b"10,", b"20\n30,40\n50", max_line_length=32
    )
    assert lines == [b"10,20", b"30,40"]
    assert pending == b"50"
    assert not overlong

    lines, pending, overlong = split_serial_lines(
        pending, b",60\n", max_line_length=32
    )
    assert lines == [b"50,60"]
    assert pending == b""
    assert not overlong


def test_split_serial_lines_discards_overlong_records():
    lines, pending, overlong = split_serial_lines(
        b"", b"123456789\n", max_line_length=8
    )
    assert lines == []
    assert pending == b""
    assert overlong


def test_split_serial_lines_discards_fragmented_overlong_record_to_newline():
    lines, pending, overlong = split_serial_lines(
        b"", b"X" * 9, max_line_length=8
    )
    assert lines == []
    assert len(pending) == 9
    assert overlong

    lines, pending, overlong = split_serial_lines(
        pending, b"100,100\n200,200\n", max_line_length=8
    )
    assert lines == [b"200,200"]
    assert pending == b""
    assert overlong


@pytest.mark.parametrize(
    "line",
    [
        b"left_ticks,right_ticks\r\n",
        b" rear_left_ticks, front_right_ticks ",
        "LEFT_TICKS,RIGHT_TICKS",
    ],
)
def test_known_arduino_headers_are_reset_markers(line):
    assert is_encoder_reset_marker(line)


def test_numeric_or_unrelated_text_is_not_a_reset_marker():
    assert not is_encoder_reset_marker(b"100,200")
    assert not is_encoder_reset_marker(b"Arduino Nano ready")


def test_equal_deltas_produce_forward_motion():
    motion = calculate_motion(
        100,
        100,
        0.1,
        COUNTS_PER_REVOLUTION,
        WHEEL_DIAMETER_M,
        TRACK_WIDTH_M,
    )
    assert motion.left_distance_m == pytest.approx(100 * METRES_PER_COUNT)
    assert motion.right_distance_m == pytest.approx(100 * METRES_PER_COUNT)
    assert motion.linear_velocity_mps == pytest.approx(
        100 * METRES_PER_COUNT / 0.1
    )
    assert motion.angular_velocity_rps == pytest.approx(0.0, abs=1e-12)


def test_larger_right_delta_produces_positive_angular_velocity():
    motion = calculate_motion(
        100,
        200,
        0.1,
        COUNTS_PER_REVOLUTION,
        WHEEL_DIAMETER_M,
        TRACK_WIDTH_M,
    )
    assert motion.linear_velocity_mps > 0.0
    assert motion.angular_velocity_rps > 0.0
    assert motion.angular_velocity_rps == pytest.approx(
        (100 * METRES_PER_COUNT / 0.1) / TRACK_WIDTH_M
    )


def test_direction_parameters_correct_opposite_raw_count_signs():
    motion = calculate_motion(
        100,
        -100,
        0.1,
        COUNTS_PER_REVOLUTION,
        WHEEL_DIAMETER_M,
        TRACK_WIDTH_M,
        left_direction=1.0,
        right_direction=-1.0,
    )
    assert motion.linear_velocity_mps > 0.0
    assert motion.angular_velocity_rps == pytest.approx(0.0, abs=1e-12)


def test_midpoint_pose_integration():
    motion = calculate_motion(
        100,
        200,
        0.1,
        COUNTS_PER_REVOLUTION,
        WHEEL_DIAMETER_M,
        TRACK_WIDTH_M,
    )
    x, y, yaw = integrate_midpoint_pose(
        0.0,
        0.0,
        0.0,
        motion.center_distance_m,
        motion.delta_yaw,
    )
    assert x == pytest.approx(
        motion.center_distance_m * math.cos(motion.delta_yaw * 0.5)
    )
    assert y == pytest.approx(
        motion.center_distance_m * math.sin(motion.delta_yaw * 0.5)
    )
    assert yaw == pytest.approx(motion.delta_yaw)


def test_pose_yaw_is_normalized_and_quaternion_is_unit_length():
    _, _, yaw = integrate_midpoint_pose(0.0, 0.0, math.pi - 0.1, 0.0, 0.2)
    assert -math.pi <= yaw <= math.pi
    quaternion = yaw_to_quaternion(yaw)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)


def test_first_tracker_sample_is_only_a_baseline():
    tracker = make_tracker()
    assert tracker.update(1000, 2000, 10.0) is None
    motion = tracker.update(1100, 2100, 10.1)
    assert motion.linear_velocity_mps == pytest.approx(
        100 * METRES_PER_COUNT / 0.1
    )
    assert motion.angular_velocity_rps == pytest.approx(0.0, abs=1e-12)


def test_count_jump_rebases_instead_of_publishing_a_spike():
    tracker = make_tracker(max_tick_jump=100)
    assert tracker.update(1000, 1000, 0.0) is None
    tracker.update(1020, 1020, 0.1)

    with pytest.raises(CountDiscontinuity):
        tracker.update(0, 0, 0.2)

    # The discontinuous sample became the new baseline.
    motion = tracker.update(10, 10, 0.3)
    assert motion.linear_velocity_mps == pytest.approx(
        10 * METRES_PER_COUNT / 0.1
    )


def test_implied_wheel_speed_rebases():
    tracker = make_tracker(max_tick_jump=1000, max_wheel_speed_mps=0.5)
    assert tracker.update(0, 0, 0.0) is None
    with pytest.raises(CountDiscontinuity):
        tracker.update(200, 200, 0.1)
    motion = tracker.update(201, 201, 0.2)
    assert motion is not None


def test_delayed_but_plausible_aggregate_is_not_a_count_jump():
    tracker = make_tracker(max_tick_jump=4096, max_wheel_speed_mps=2.0)
    assert tracker.update(0, 0, 0.0) is None
    motion = tracker.update(800, 800, 0.4)
    assert motion is not None
    assert motion.linear_velocity_mps == pytest.approx(
        800 * METRES_PER_COUNT / 0.4
    )


def test_measurement_timeout_boundary():
    assert not measurement_timed_out(None, now=10.0, timeout_sec=0.5)
    assert not measurement_timed_out(10.0, now=10.49, timeout_sec=0.5)
    assert measurement_timed_out(10.0, now=10.5, timeout_sec=0.5)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"counts_per_revolution": 0.0},
        {"wheel_diameter_m": -1.0},
        {"track_width_m": 0.0},
        {"left_direction": 0.0},
        {"right_direction": 2.0},
    ],
)
def test_invalid_calibration_is_rejected(kwargs):
    values = {
        "delta_left_ticks": 1,
        "delta_right_ticks": 1,
        "dt": 0.1,
        "counts_per_revolution": COUNTS_PER_REVOLUTION,
        "wheel_diameter_m": WHEEL_DIAMETER_M,
        "track_width_m": TRACK_WIDTH_M,
        "left_direction": 1.0,
        "right_direction": 1.0,
    }
    values.update(kwargs)
    with pytest.raises(ValueError):
        calculate_motion(**values)
