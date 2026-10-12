import argparse

import pytest

from rov_thruster_bridge.communication_stress_test import (
    StressRunner,
    build_parser,
    percentile,
    positive,
    summarize_ms,
)


def test_percentile_and_summary():
    assert percentile([], 0.99) is None
    assert percentile([1.0, 2.0, 3.0], 0.50) == pytest.approx(2.0)
    assert percentile([0.0, 10.0], 0.95) == pytest.approx(9.5)
    summary = summarize_ms([1.0, 2.0, 3.0])
    assert summary['count'] == 3
    assert summary['mean_ms'] == pytest.approx(2.0)
    assert summary['max_ms'] == pytest.approx(3.0)


def test_positive_arguments_and_competition_defaults():
    options = build_parser().parse_args([])
    assert options.duration == pytest.approx(600.0)
    assert options.send_rate == pytest.approx(20.0)
    assert options.telemetry_rate == pytest.approx(10.0)
    assert options.timeout == pytest.approx(0.20)
    assert positive('1.5') == pytest.approx(1.5)
    with pytest.raises(argparse.ArgumentTypeError):
        positive('0')


def test_deadline_advance_skips_missed_slots_without_bursting():
    deadline, skipped = StressRunner.advance_deadline(1.0, 0.05, 1.18)
    assert deadline == pytest.approx(1.20)
    assert skipped == 3


def test_stress_tool_has_no_nonzero_motion_arguments():
    parser = build_parser()
    option_names = {action.dest for action in parser._actions}
    assert not {'sway', 'surge', 'heave', 'yaw', 'allow_motion'} & option_names
