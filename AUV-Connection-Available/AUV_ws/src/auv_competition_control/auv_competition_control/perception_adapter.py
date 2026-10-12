"""Validation for the dual-model perception JSON contracts."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Tuple

from auv_competition_control.controller_core import (
    BallObservation,
    Detection,
    GateKeypoint,
    GatePoseObservation,
)


@dataclass(frozen=True)
class Objects3dFrame:
    frame_id: str
    detections: Tuple[Detection, ...]
    raw_object_count: int
    rejected_object_count: int


@dataclass(frozen=True)
class BallFrame:
    frame_id: str
    observations: Tuple[BallObservation, ...]
    raw_count: int
    rejected_count: int


@dataclass(frozen=True)
class GatePoseFrame:
    frame_id: str
    observations: Tuple[GatePoseObservation, ...]
    raw_count: int
    rejected_count: int


def _root(message_data: str) -> dict:
    payload = json.loads(message_data)
    if not isinstance(payload, dict):
        raise ValueError('perception root must be a JSON object')
    return payload


def _frame_id(payload: dict) -> str:
    value = payload.get('frame_id', 'zed_left_camera_optical_frame')
    if not isinstance(value, str) or not value:
        raise ValueError('frame_id must be a non-empty string')
    return value


def _observations(payload: dict) -> list:
    value = payload.get('observations', [])
    if not isinstance(value, list):
        raise ValueError('observations must be a JSON array')
    return value


def parse_ball_observations(message_data: str, received_s: float) -> BallFrame:
    """Parse metric ball observations published by the coordinator."""
    payload = _root(message_data)
    if payload.get('units', 'm') != 'm':
        raise ValueError('ball observation units must be metres')
    raw = _observations(payload)
    accepted = []
    rejected = 0
    for item in raw:
        try:
            if not isinstance(item, dict):
                raise ValueError('ball observation must be an object')
            position = item.get('position_m')
            if not isinstance(position, (list, tuple)) or len(position) != 3:
                raise ValueError('position_m must contain XYZ')
            values = tuple(float(value) for value in position)
            if not all(math.isfinite(value) for value in values):
                raise ValueError('position_m must be finite')
            accepted.append(BallObservation(
                class_name=str(item.get('class_name', '')),
                confidence=float(item.get('confidence', 0.0)),
                x=values[0], y=values[1], z=values[2],
                received_s=received_s,
            ))
        except (TypeError, ValueError, OverflowError):
            rejected += 1
    return BallFrame(
        frame_id=_frame_id(payload),
        observations=tuple(accepted),
        raw_count=len(raw),
        rejected_count=rejected,
    )


def parse_gate_pose_observations(
    message_data: str, received_s: float,
) -> GatePoseFrame:
    """Parse four-corner gate Pose observations from normalized JSON."""
    payload = _root(message_data)
    raw = _observations(payload)
    accepted = []
    rejected = 0
    for item in raw:
        try:
            if not isinstance(item, dict):
                raise ValueError('gate observation must be an object')
            raw_points = item.get('keypoints')
            if not isinstance(raw_points, list) or len(raw_points) != 4:
                raise ValueError('gate keypoints must contain TL,TR,BR,BL')
            points = []
            for point in raw_points:
                if not isinstance(point, dict):
                    raise ValueError('keypoint must be an object')
                keypoint = GateKeypoint(
                    x=float(point['x_norm']),
                    y=float(point['y_norm']),
                    confidence=float(point.get('confidence', 0.0)),
                )
                if not all(math.isfinite(value) for value in (
                        keypoint.x, keypoint.y, keypoint.confidence)):
                    raise ValueError('keypoint values must be finite')
                points.append(keypoint)
            center = item.get('center_error_norm')
            size = item.get('size_norm')
            if not isinstance(center, (list, tuple)) or len(center) != 2:
                raise ValueError('center_error_norm must contain XY')
            if not isinstance(size, (list, tuple)) or len(size) != 2:
                raise ValueError('size_norm must contain width,height')
            translation = item.get('translation_m')
            tx = ty = range_m = None
            if translation is not None:
                if not isinstance(translation, (list, tuple)) or len(translation) != 3:
                    raise ValueError('translation_m must contain XYZ or be null')
                tx, ty, range_m = (float(value) for value in translation)
            reprojection = item.get('reprojection_error_px')
            accepted.append(GatePoseObservation(
                confidence=float(item.get('confidence', 0.0)),
                keypoints=tuple(points),
                center_x_norm=float(center[0]),
                center_y_norm=float(center[1]),
                width_norm=float(size[0]),
                height_norm=float(size[1]),
                bearing_x_rad=float(item.get('bearing_x_rad', 0.0)),
                received_s=received_s,
                geometry_valid=bool(item.get('geometry_valid', False)),
                range_m=range_m,
                translation_x_m=tx,
                translation_y_m=ty,
                reprojection_error_px=(
                    None if reprojection is None else float(reprojection)),
            ))
        except (KeyError, TypeError, ValueError, OverflowError):
            rejected += 1
    return GatePoseFrame(
        frame_id=_frame_id(payload),
        observations=tuple(accepted),
        raw_count=len(raw),
        rejected_count=rejected,
    )


def parse_objects3d(message_data: str, received_s: float) -> Objects3dFrame:
    """Parse one detector message and reject stale SDK tracks or invalid XYZ."""
    payload = _root(message_data)
    if payload.get('units', 'm') != 'm':
        raise ValueError('objects3d units must be metres')

    frame_id = _frame_id(payload)
    objects = payload.get('objects', [])
    if not isinstance(objects, list):
        raise ValueError('objects must be a JSON array')

    detections = []
    rejected = 0
    for item in objects:
        try:
            if not isinstance(item, dict):
                raise ValueError('object must be a JSON object')
            if not item.get('position_valid', True):
                raise ValueError('invalid position')
            # The ZED SDK may keep publishing predicted tracks marked OFF.
            # They are not fresh observations and must not drive control.
            if str(item.get('tracking_state', 'OK')).upper() != 'OK':
                raise ValueError('tracking state is not OK')
            position = item.get('position_m')
            if not isinstance(position, (list, tuple)) or len(position) != 3:
                raise ValueError('position_m must contain XYZ')
            detections.append(Detection(
                class_name=str(item.get('class_name', '')),
                confidence=float(item.get('confidence', 0.0)),
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
                received_s=received_s,
            ))
        except (TypeError, ValueError, OverflowError):
            rejected += 1

    return Objects3dFrame(
        frame_id=frame_id,
        detections=tuple(detections),
        raw_object_count=len(objects),
        rejected_object_count=rejected,
    )
