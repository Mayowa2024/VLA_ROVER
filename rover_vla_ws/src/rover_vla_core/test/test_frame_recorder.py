"""Tests for fixed-rate frame sampling."""

import pytest

from rover_vla_core.frame_recorder import (
    advance_sample_deadline,
    sample_is_due,
)


def select_samples(callback_times_ns, period_ns):
    """Return callback times accepted by the frame recorder's rate gate."""
    deadline_ns = None
    selected = []
    for now_ns in callback_times_ns:
        if not sample_is_due(now_ns, deadline_ns):
            continue
        selected.append(now_ns)
        deadline_ns = advance_sample_deadline(
            deadline_ns, now_ns, period_ns
        )
    return selected


def test_fixed_schedule_achieves_ten_hz_from_33_ms_camera_callbacks():
    """A 33 ms camera callback cadence should average 10 saved frames/second."""
    callback_times_ns = [index * 33_000_000 for index in range(304)]

    selected = select_samples(callback_times_ns, period_ns=100_000_000)

    duration_sec = (selected[-1] - selected[0]) / 1_000_000_000
    effective_rate = (len(selected) - 1) / duration_sec
    assert len(selected) == 100
    assert effective_rate == pytest.approx(10.0, rel=0.01)


def test_exact_30_hz_camera_selects_every_third_frame():
    """An exact 30 Hz source should select every third frame."""
    callback_times_ns = [
        round(index * 1_000_000_000 / 30) for index in range(301)
    ]

    selected = select_samples(callback_times_ns, period_ns=100_000_000)

    assert len(selected) == 101
    assert selected == [index * 100_000_000 for index in range(101)]


def test_stall_skips_missed_deadlines_without_a_catch_up_burst():
    """A long callback stall must not trigger rapid catch-up samples."""
    callback_times_ns = [
        0,
        100_000_000,
        5_000_000_000,
        5_010_000_000,
        5_050_000_000,
        5_100_000_000,
    ]
    selected = select_samples(
        callback_times_ns, period_ns=100_000_000
    )

    assert selected == [
        0,
        100_000_000,
        5_000_000_000,
        5_100_000_000,
    ]
