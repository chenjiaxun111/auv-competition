import math

import pytest

from rov_thruster_bridge.motion_mapper import MotionCommand, MotionMapper


def make_mapper(**overrides):
    values = {
        'max_surge': 100.0,
        'max_sway': 50.0,
        'max_heave': 250.0,
        'max_yaw': 30.0,
    }
    values.update(overrides)
    return MotionMapper(**values)


def test_maps_axes_and_clamps():
    command = make_mapper().map(surge=50.0, sway=-25.0, heave=500.0, yaw=-60.0)
    assert command == MotionCommand(sway=-0.5, surge=0.5, heave=1.0, yaw=-1.0)


def test_axis_signs_are_applied():
    command = make_mapper(surge_sign=-1.0, heave_sign=-1.0).map(
        surge=20.0, sway=0.0, heave=25.0, yaw=0.0)
    assert command.surge == pytest.approx(-0.2)
    assert command.heave == pytest.approx(-0.1)


def test_non_finite_command_fails_safe_to_zero():
    command = make_mapper().map(
        surge=math.nan, sway=math.inf, heave=-math.inf, yaw=math.nan)
    assert command == MotionCommand.zero()


def test_rejects_invalid_configuration():
    with pytest.raises(ValueError):
        make_mapper(max_surge=0.0)
    with pytest.raises(ValueError):
        make_mapper(yaw_sign=0.0)
