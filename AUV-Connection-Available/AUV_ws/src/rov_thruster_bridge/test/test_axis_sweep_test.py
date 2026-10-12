import argparse

import pytest

from rov_thruster_bridge.axis_sweep_test import (
    expected_signs,
    format_power,
    normalize_power,
    safe_amplitude,
    validate_pattern,
)


def test_canonical_axis_patterns():
    assert expected_signs('surge', 1) == {
        'left_up': 1, 'right_up': 1,
        'left_middle': 0, 'right_middle': 0,
        'left_down': 1, 'right_down': 1,
    }
    assert expected_signs('yaw', -1) == {
        'left_up': -1, 'right_up': 1,
        'left_middle': 0, 'right_middle': 0,
        'left_down': -1, 'right_down': 1,
    }


def test_pattern_validation_accepts_and_rejects():
    valid = {
        'left_up': 20, 'right_up': -20,
        'left_middle': 0, 'right_middle': 0,
        'left_down': -20, 'right_down': 20,
    }
    validate_pattern('sway', 1, valid)
    invalid = dict(valid, right_down=-20)
    with pytest.raises(RuntimeError, match='mixer mismatch'):
        validate_pattern('sway', 1, invalid)


def test_power_validation_and_display():
    power = normalize_power({
        'left_up': 20.0, 'right_up': -20,
        'left_middle': 0, 'right_middle': 0,
        'left_down': -20, 'right_down': 20,
    })
    display = format_power(power)
    assert 'left_up=+20 (1520us/' in display
    assert 'right_up=-20 (1480us/' in display
    with pytest.raises(RuntimeError, match='missing fields'):
        normalize_power({'left_up': 0})


def test_amplitude_is_bounded_for_bench_use():
    assert safe_amplitude('0.30') == pytest.approx(0.30)
    with pytest.raises(argparse.ArgumentTypeError):
        safe_amplitude('0.20')
    with pytest.raises(argparse.ArgumentTypeError):
        safe_amplitude('0.60')
