import json

import pytest

from auv_competition_control.perception_adapter import (
    parse_ball_observations,
    parse_gate_pose_observations,
    parse_objects3d,
)


def test_parse_real_zed_schema():
    frame = parse_objects3d(json.dumps({
        'frame_id': 'zed_left_camera_optical_frame',
        'units': 'm',
        'objects': [{
            'class_name': 'person',
            'confidence': 0.91,
            'position_valid': True,
            'position_m': [0.2, 0.1, 2.0],
            'tracking_state': 'OK',
        }],
    }), 12.5)
    assert frame.raw_object_count == 1
    assert frame.rejected_object_count == 0
    assert frame.detections[0].class_name == 'person'
    assert frame.detections[0].received_s == 12.5


def test_rejects_predicted_and_invalid_positions():
    frame = parse_objects3d(json.dumps({'objects': [
        {
            'class_name': 'tv', 'confidence': 0.8,
            'position_valid': True, 'position_m': [0.0, 0.0, 1.0],
            'tracking_state': 'OFF',
        },
        {
            'class_name': 'person', 'confidence': 0.8,
            'position_valid': False, 'position_m': None,
            'tracking_state': 'OK',
        },
    ]}), 1.0)
    assert frame.detections == ()
    assert frame.rejected_object_count == 2


@pytest.mark.parametrize('payload', [
    '[]',
    '{"units":"cm","objects":[]}',
    '{"objects":{}}',
])
def test_rejects_malformed_top_level(payload):
    with pytest.raises(ValueError):
        parse_objects3d(payload, 1.0)


def test_parse_dual_model_ball_contract():
    frame = parse_ball_observations(json.dumps({
        'frame_id': 'camera',
        'units': 'm',
        'observations': [{
            'class_name': 'red_ball',
            'confidence': 0.9,
            'position_m': [0.1, -0.2, 1.5],
        }],
    }), 2.0)
    assert len(frame.observations) == 1
    assert frame.observations[0].z == 1.5


def test_parse_gate_pose_contract_with_optional_pnp():
    frame = parse_gate_pose_observations(json.dumps({
        'frame_id': 'camera',
        'observations': [{
            'confidence': 0.95,
            'keypoints': [
                {'x_norm': 0.2, 'y_norm': 0.2, 'confidence': 0.9},
                {'x_norm': 0.8, 'y_norm': 0.2, 'confidence': 0.9},
                {'x_norm': 0.8, 'y_norm': 0.8, 'confidence': 0.9},
                {'x_norm': 0.2, 'y_norm': 0.8, 'confidence': 0.9},
            ],
            'center_error_norm': [0.0, 0.0],
            'size_norm': [0.6, 0.6],
            'bearing_x_rad': 0.0,
            'geometry_valid': True,
            'translation_m': [0.0, 0.0, 1.8],
            'reprojection_error_px': 2.0,
        }],
    }), 3.0)
    pose = frame.observations[0]
    assert pose.minimum_keypoint_confidence == pytest.approx(0.9)
    assert pose.range_m == pytest.approx(1.8)


def test_gate_pose_parser_rejects_incomplete_keypoints():
    frame = parse_gate_pose_observations(json.dumps({
        'observations': [{
            'confidence': 0.9,
            'keypoints': [],
            'center_error_norm': [0.0, 0.0],
            'size_norm': [0.5, 0.5],
        }],
    }), 1.0)
    assert frame.observations == ()
    assert frame.rejected_count == 1
