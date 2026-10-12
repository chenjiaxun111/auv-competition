import math

import pytest

from auv_competition_control.pose_geometry import gate_image_geometry


def test_gate_geometry_normalizes_center_and_size():
    geometry = gate_image_geometry(
        [(100, 100), (500, 100), (500, 380), (100, 380)],
        image_width=640,
        image_height=480,
        fx=400.0,
        cx=320.0,
    )
    assert geometry.valid
    assert geometry.center_x_norm == pytest.approx(-0.0625)
    assert geometry.center_y_norm == pytest.approx(0.0)
    assert geometry.width_norm == pytest.approx(0.625)
    assert geometry.bearing_x_rad == pytest.approx(math.atan2(-20.0, 400.0))


def test_gate_geometry_rejects_crossed_corner_order():
    geometry = gate_image_geometry(
        [(100, 100), (500, 380), (500, 100), (100, 380)],
        image_width=640,
        image_height=480,
        fx=400.0,
        cx=320.0,
    )
    assert not geometry.valid


def test_gate_geometry_rejects_corner_outside_image():
    geometry = gate_image_geometry(
        [(-1, 100), (500, 100), (500, 380), (100, 380)],
        image_width=640,
        image_height=480,
        fx=400.0,
        cx=320.0,
    )
    assert not geometry.in_bounds
    assert not geometry.valid
