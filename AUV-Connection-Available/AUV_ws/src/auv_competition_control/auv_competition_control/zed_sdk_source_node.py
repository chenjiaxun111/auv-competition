#!/usr/bin/env python3
"""Publish ZED SDK left image, registered depth, and intrinsics to ROS 2.

This node is the sole camera owner when the full ZED ROS Wrapper is not
installed.  It performs no neural-network inference and has no actuator
publishers; ``dual_model_perception`` consumes the three sensor topics.
"""

from __future__ import annotations

import json
import math
import signal
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


RESOLUTIONS = {
    'HD720': 'HD720',
    'HD1080': 'HD1080',
    'VGA': 'VGA',
}

DEPTH_MODES = {
    'NEURAL_LIGHT': 'NEURAL_LIGHT',
    'NEURAL': 'NEURAL',
    'PERFORMANCE': 'PERFORMANCE',
    'QUALITY': 'QUALITY',
}


def fill_rectified_camera_info(
        message: CameraInfo, width: int, height: int,
        fx: float, fy: float, cx: float, cy: float) -> None:
    """Fill CameraInfo for an already-rectified left optical image."""
    values = (fx, fy, cx, cy)
    if width <= 0 or height <= 0 or not all(
            math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('invalid rectified camera calibration')
    message.width = int(width)
    message.height = int(height)
    message.distortion_model = 'plumb_bob'
    message.d = [0.0] * 5
    message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
    message.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]


class ZedSdkSourceNode(Node):
    """Own one ZED camera and publish synchronized RGB-D sensor messages."""

    def __init__(self) -> None:
        super().__init__('zed_sdk_source')
        self._declare_parameters()

        # pyzed is supplied by the target's ZED SDK, not by rosdep/pip.
        import pyzed.sl as sl

        self._sl = sl
        self._bridge = CvBridge()
        self._camera = sl.Camera()
        self._image = sl.Mat()
        self._depth = sl.Mat()
        self._closed = False
        self._frames = 0
        self._errors = 0
        self._last_publish_s = -math.inf
        self._last_report_s = time.monotonic()
        self._report_frames = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        diagnostic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._image_pub = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value), sensor_qos)
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter('depth_topic').value), sensor_qos)
        self._info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), sensor_qos)
        self._diagnostics_pub = self.create_publisher(
            String, '/zed/sdk_source/diagnostics', diagnostic_qos)

        self._open_camera()
        self.get_logger().info(
            'ZED SDK source ready; direct camera owner, no inference or actuator path')

    def _declare_parameters(self) -> None:
        values = {
            'image_topic': '/zed/zed_node/left/image_rect_color',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/left/camera_info',
            'frame_id': 'zed_left_camera_optical_frame',
            'camera_resolution': 'HD720',
            'camera_fps': 30,
            'publish_fps': 15.0,
            'depth_mode': 'NEURAL_LIGHT',
            'depth_minimum_m': 0.15,
            'depth_maximum_m': 8.0,
            'maximum_consecutive_errors': 30,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)
        resolution = str(self.get_parameter('camera_resolution').value).upper()
        depth_mode = str(self.get_parameter('depth_mode').value).upper()
        camera_fps = int(self.get_parameter('camera_fps').value)
        publish_fps = float(self.get_parameter('publish_fps').value)
        minimum = float(self.get_parameter('depth_minimum_m').value)
        maximum = float(self.get_parameter('depth_maximum_m').value)
        if resolution not in RESOLUTIONS:
            raise ValueError(f'unsupported camera_resolution: {resolution}')
        if depth_mode not in DEPTH_MODES:
            raise ValueError(f'unsupported depth_mode: {depth_mode}')
        if camera_fps <= 0 or publish_fps <= 0.0 or publish_fps > camera_fps:
            raise ValueError('publish_fps must be positive and no greater than camera_fps')
        if minimum <= 0.0 or maximum <= minimum:
            raise ValueError('invalid ZED depth range')

    def _open_camera(self) -> None:
        sl = self._sl
        resolution_name = str(
            self.get_parameter('camera_resolution').value).upper()
        depth_mode_name = str(self.get_parameter('depth_mode').value).upper()
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, RESOLUTIONS[resolution_name])
        init.camera_fps = int(self.get_parameter('camera_fps').value)
        init.depth_mode = getattr(sl.DEPTH_MODE, DEPTH_MODES[depth_mode_name])
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        init.depth_minimum_distance = float(
            self.get_parameter('depth_minimum_m').value)
        init.depth_maximum_distance = float(
            self.get_parameter('depth_maximum_m').value)
        status = self._camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'ZED camera open failed: {status}')

        camera = self._camera.get_camera_information().camera_configuration
        calibration = camera.calibration_parameters.left_cam
        resolution = camera.resolution
        self._camera_info = CameraInfo()
        fill_rectified_camera_info(
            self._camera_info,
            int(resolution.width), int(resolution.height),
            float(calibration.fx), float(calibration.fy),
            float(calibration.cx), float(calibration.cy),
        )
        self._runtime = sl.RuntimeParameters()
        self._runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA

    def _stamp(self):
        timestamp_ns = int(self._camera.get_timestamp(
            self._sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        if timestamp_ns <= 0:
            return self.get_clock().now().to_msg()
        stamp = self.get_clock().now().to_msg()
        stamp.sec = timestamp_ns // 1_000_000_000
        stamp.nanosec = timestamp_ns % 1_000_000_000
        return stamp

    def capture_and_publish(self) -> bool:
        """Grab one SDK frame; publish at the configured throttled rate."""
        status = self._camera.grab(self._runtime)
        if status != self._sl.ERROR_CODE.SUCCESS:
            self._errors += 1
            if self._errors >= int(
                    self.get_parameter('maximum_consecutive_errors').value):
                raise RuntimeError(f'repeated ZED grab failure: {status}')
            return False
        self._errors = 0

        now_s = time.monotonic()
        period = 1.0 / float(self.get_parameter('publish_fps').value)
        if now_s - self._last_publish_s < period:
            return False
        self._last_publish_s = now_s

        image_status = self._camera.retrieve_image(
            self._image, self._sl.VIEW.LEFT)
        depth_status = self._camera.retrieve_measure(
            self._depth, self._sl.MEASURE.DEPTH)
        if (image_status != self._sl.ERROR_CODE.SUCCESS or
                depth_status != self._sl.ERROR_CODE.SUCCESS):
            self._errors += 1
            return False

        bgra = np.asarray(self._image.get_data())
        depth = np.asarray(self._depth.get_data(), dtype=np.float32)
        bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        stamp = self._stamp()
        frame_id = str(self.get_parameter('frame_id').value)

        image_message = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        image_message.header.stamp = stamp
        image_message.header.frame_id = frame_id
        depth_message = self._bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = frame_id
        info_message = CameraInfo()
        info_message.header.stamp = stamp
        info_message.header.frame_id = frame_id
        info_message.height = self._camera_info.height
        info_message.width = self._camera_info.width
        info_message.distortion_model = self._camera_info.distortion_model
        info_message.d = list(self._camera_info.d)
        info_message.k = list(self._camera_info.k)
        info_message.r = list(self._camera_info.r)
        info_message.p = list(self._camera_info.p)

        self._image_pub.publish(image_message)
        self._depth_pub.publish(depth_message)
        self._info_pub.publish(info_message)
        self._frames += 1
        self._report_frames += 1
        self._publish_diagnostics(now_s, depth)
        return True

    def _publish_diagnostics(self, now_s: float, depth: np.ndarray) -> None:
        elapsed = now_s - self._last_report_s
        if elapsed < 1.0:
            return
        finite = np.isfinite(depth) & (depth > 0.0)
        message = String()
        message.data = json.dumps({
            'status': 'ok',
            'published_fps': round(self._report_frames / elapsed, 2),
            'total_frames': self._frames,
            'valid_depth_fraction': round(float(np.mean(finite)), 4),
            'frame_id': str(self.get_parameter('frame_id').value),
        }, allow_nan=False)
        self._diagnostics_pub.publish(message)
        self._last_report_s = now_s
        self._report_frames = 0

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._camera.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ZedSdkSourceNode()
        while rclpy.ok():
            node.capture_and_publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal and launch may both send SIGINT. Finish SDK close once begun.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
