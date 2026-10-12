#!/usr/bin/env python3
"""Closed-loop synthetic ZED source for testing without a trained model."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class MockObjects3dNode(Node):
    def __init__(self) -> None:
        super().__init__('mock_objects3d')
        self.declare_parameter('target_ball_color', 'red')
        self.declare_parameter('depth_m', 0.60)
        self._state = 'IDLE'
        self._state_entered_s = time.monotonic()
        self._objects_pub = self.create_publisher(String, '/zed/objects3d', 10)
        self._ball_pub = self.create_publisher(
            String, '/auv/perception/ball_observations', 10)
        self._gate_pub = self.create_publisher(
            String, '/auv/perception/gate_pose_observations', 10)
        self._depth_pub = self.create_publisher(Float32, '/auv/depth_m', 10)
        self._connected_pub = self.create_publisher(Bool, '/auv/connected', 10)
        self._state_sub = self.create_subscription(
            String, '/auv/mission_state', self._state_callback, 10)
        self._timer = self.create_timer(1.0 / 15.0, self._tick)

    def _state_callback(self, message: String) -> None:
        if message.data != self._state:
            self._state = message.data
            self._state_entered_s = time.monotonic()

    def _make_object(self, class_name: str, x: float, y: float, z: float) -> dict:
        return {
            'id': 1,
            'class_id': 0,
            'class_name': class_name,
            'confidence': 0.92,
            'position_valid': True,
            'position_m': [x, y, z],
            'distance_m': (x * x + y * y + z * z) ** 0.5,
            'tracking_state': 'OK',
            'bbox_2d': [[250, 170], [390, 170], [390, 310], [250, 310]],
        }

    def _tick(self) -> None:
        connected = Bool()
        connected.data = True
        self._connected_pub.publish(connected)
        depth = Float32()
        depth.data = float(self.get_parameter('depth_m').value)
        self._depth_pub.publish(depth)

        elapsed = time.monotonic() - self._state_entered_s
        objects = []
        ball_observations = []
        gate_observations = []
        color = str(self.get_parameter('target_ball_color').value)
        ball_class = f'{color}_ball'
        if self._state in ('SEARCH_BALL', 'ALIGN_BALL'):
            x = max(0.0, 0.35 - 0.35 * elapsed)
            y = max(0.0, 0.15 - 0.15 * elapsed)
            objects.append(self._make_object(ball_class, x, y, 2.2))
            ball_observations.append({
                'class_name': ball_class,
                'confidence': 0.92,
                'position_m': [x, y, 2.2],
            })
        elif self._state == 'APPROACH_BALL':
            z = max(0.25, 2.2 - 1.3 * elapsed)
            objects.append(self._make_object(ball_class, 0.01, 0.01, z))
            ball_observations.append({
                'class_name': ball_class,
                'confidence': 0.92,
                'position_m': [0.01, 0.01, z],
            })
        elif self._state in ('SEARCH_GATE', 'ALIGN_GATE'):
            cx = max(0.0, 0.35 - 0.35 * elapsed)
            cy = max(0.0, 0.15 - 0.15 * elapsed)
            gate_observations.append(self._make_gate_pose(cx, cy, 0.30, 3.0))
        elif self._state == 'APPROACH_GATE':
            z = max(0.85, 3.0 - 1.3 * elapsed)
            width = min(0.75, 0.30 + 0.22 * elapsed)
            gate_observations.append(self._make_gate_pose(0.01, 0.01, width, z))

        message = String()
        message.data = json.dumps({
            'timestamp_ns': self.get_clock().now().nanoseconds,
            'frame_id': 'zed_left_camera_optical_frame',
            'units': 'm',
            'objects': objects,
        })
        self._objects_pub.publish(message)

        ball_message = String()
        ball_message.data = json.dumps({
            'schema': 'auv.ball_observations.v1',
            'frame_id': 'zed_left_camera_optical_frame',
            'units': 'm',
            'observations': ball_observations,
        })
        self._ball_pub.publish(ball_message)
        gate_message = String()
        gate_message.data = json.dumps({
            'schema': 'auv.gate_pose_observations.v1',
            'frame_id': 'zed_left_camera_optical_frame',
            'observations': gate_observations,
        })
        self._gate_pub.publish(gate_message)

    @staticmethod
    def _make_gate_pose(cx: float, cy: float, width: float, range_m: float) -> dict:
        centre_x = 0.5 + 0.5 * cx
        centre_y = 0.5 + 0.5 * cy
        height = width * 0.75
        points = [
            (centre_x - width / 2.0, centre_y - height / 2.0),
            (centre_x + width / 2.0, centre_y - height / 2.0),
            (centre_x + width / 2.0, centre_y + height / 2.0),
            (centre_x - width / 2.0, centre_y + height / 2.0),
        ]
        return {
            'confidence': 0.94,
            'keypoints': [
                {'x_norm': x, 'y_norm': y, 'confidence': 0.93}
                for x, y in points
            ],
            'center_error_norm': [cx, cy],
            'size_norm': [width, height],
            'bearing_x_rad': cx * 0.55,
            'geometry_valid': True,
            'translation_m': [cx * range_m * 0.4,
                              cy * range_m * 0.3, range_m],
            'reprojection_error_px': 1.0,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockObjects3dNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
