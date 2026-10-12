#!/usr/bin/env python3
"""State-aware YOLO detector / YOLO-Pose coordinator for the ZED camera.

Only the model required by the current mission state runs:

* ball states -> three-class detector + registered depth back-projection;
* gate states -> four-corner gate Pose + optional calibrated planar PnP;
* timed/idle states -> no inference.

Inference runs on a worker thread and always replaces queued input with the
newest frame, preventing stale camera frames from accumulating.
"""

from __future__ import annotations

import json
import math
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from auv_competition_control.pose_geometry import gate_image_geometry


BALL_STATES = {'SEARCH_BALL', 'ALIGN_BALL', 'APPROACH_BALL'}
GATE_POSE_STATES = {'ALIGN_GATE', 'APPROACH_GATE'}


class DualModelPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('dual_model_perception')
        self._declare_parameters()

        # Importing Ultralytics can initialize CUDA, so do it after ROS
        # parameters have been validated and keep it out of pure unit tests.
        from ultralytics import YOLO

        detector_path = str(self.get_parameter('detector_model_path').value)
        pose_path = str(self.get_parameter('gate_pose_model_path').value)
        if not detector_path or not pose_path:
            raise ValueError('both detector_model_path and gate_pose_model_path are required')
        self.get_logger().info(f'Loading ball detector: {detector_path}')
        self._detector = YOLO(detector_path)
        self.get_logger().info(f'Loading gate Pose model: {pose_path}')
        self._pose_model = YOLO(pose_path)
        self._prewarm_models()

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame_event = threading.Event()
        self._stop_event = threading.Event()
        self._latest_image = None
        self._latest_image_frame_id = 'zed_left_camera_optical_frame'
        self._latest_image_s = -math.inf
        self._latest_depth = None
        self._latest_depth_s = -math.inf
        self._camera_matrix = None
        self._distortion = None
        self._mission_state = 'IDLE'
        self._last_debug_s = -math.inf

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        image_topic = str(self.get_parameter('image_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, sensor_qos)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, sensor_qos)
        self._state_sub = self.create_subscription(
            String, '/auv/mission_state', self._state_callback, state_qos)
        self._ball_pub = self.create_publisher(
            String, '/auv/perception/ball_observations', state_qos)
        self._gate_pub = self.create_publisher(
            String, '/auv/perception/gate_pose_observations', state_qos)
        self._diagnostics_pub = self.create_publisher(
            String, '/auv/perception/diagnostics', state_qos)
        self._debug_pub = self.create_publisher(
            CompressedImage, '/auv/perception/debug/image/compressed', sensor_qos)

        self._worker = threading.Thread(
            target=self._worker_loop,
            name='dual-model-inference',
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(
            'Dual-model coordinator ready; inference is gated by mission state')

    def _declare_parameters(self) -> None:
        values = {
            'detector_model_path': '',
            'gate_pose_model_path': '',
            'image_topic': '/zed/zed_node/left/image_rect_color',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/left/camera_info',
            'target_ball_color': 'red',
            'red_ball_class': 'red_ball',
            'blue_ball_class': 'blue_ball',
            'gate_class': 'gate',
            'inference_device': '0',
            'inference_imgsz': 640,
            'use_half_precision': True,
            'ball_confidence': 0.45,
            'gate_confidence': 0.50,
            'gate_keypoint_confidence': 0.50,
            'maximum_depth_m': 8.0,
            'depth_max_age_s': 0.20,
            'depth_roi_fraction': 0.30,
            'enable_gate_pnp': False,
            # Competition nominal gate size. Keep PnP disabled until the
            # labelled corner convention and underwater calibration are checked.
            'gate_width_m': 0.70,
            'gate_height_m': 0.50,
            'maximum_reprojection_error_px': 12.0,
            'publish_debug_image': True,
            'debug_jpeg_quality': 80,
            'debug_max_fps': 5.0,
            'vision_debug_all_models': False,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)
        color = str(self.get_parameter('target_ball_color').value).lower()
        if color not in ('red', 'blue'):
            raise ValueError('target_ball_color must be red or blue')
        if int(self.get_parameter('inference_imgsz').value) <= 0:
            raise ValueError('inference_imgsz must be positive')

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _state_callback(self, message: String) -> None:
        with self._lock:
            self._mission_state = message.data

    def _image_callback(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:  # cv_bridge exception types vary by distro
            self.get_logger().warning(f'Image conversion failed: {error}')
            return
        with self._lock:
            self._latest_image = image.copy()
            self._latest_image_frame_id = (
                message.header.frame_id or 'zed_left_camera_optical_frame')
            self._latest_image_s = self._now_s()
        self._frame_event.set()

    def _depth_callback(self, message: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            depth = np.asarray(depth, dtype=np.float32)
            # ZED 32FC1 is metres; 16UC1 convention is millimetres.
            if message.encoding in ('16UC1', 'mono16'):
                depth *= 0.001
        except Exception as error:
            self.get_logger().warning(f'Depth conversion failed: {error}')
            return
        with self._lock:
            self._latest_depth = depth.copy()
            self._latest_depth_s = self._now_s()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(message.d, dtype=np.float64)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        with self._lock:
            self._camera_matrix = matrix
            self._distortion = distortion

    def _predict(self, model, image, confidence: float):
        return model.predict(
            source=image,
            conf=confidence,
            imgsz=int(self.get_parameter('inference_imgsz').value),
            device=str(self.get_parameter('inference_device').value),
            half=bool(self.get_parameter('use_half_precision').value),
            verbose=False,
        )[0]

    def _prewarm_models(self) -> None:
        """Force lazy CUDA/model initialization before mission arming."""
        size = int(self.get_parameter('inference_imgsz').value)
        dummy = np.zeros((size, size, 3), dtype=np.uint8)
        for name, model, confidence in (
                ('ball detector', self._detector,
                 float(self.get_parameter('ball_confidence').value)),
                ('gate Pose', self._pose_model,
                 float(self.get_parameter('gate_confidence').value))):
            started = self._now_s()
            self._predict(model, dummy, confidence)
            self.get_logger().info(
                f'Prewarmed {name} in {(self._now_s() - started) * 1000.0:.1f} ms')

    def _depth_position(self, box, depth, matrix):
        if depth is None or matrix is None:
            return None
        x1, y1, x2, y2 = (float(value) for value in box)
        fraction = float(self.get_parameter('depth_roi_fraction').value)
        fraction = max(0.05, min(0.8, fraction))
        centre_x = 0.5 * (x1 + x2)
        centre_y = 0.5 * (y1 + y2)
        half_w = max(1, int((x2 - x1) * fraction * 0.5))
        half_h = max(1, int((y2 - y1) * fraction * 0.5))
        left = max(0, int(centre_x) - half_w)
        right = min(depth.shape[1], int(centre_x) + half_w + 1)
        top = max(0, int(centre_y) - half_h)
        bottom = min(depth.shape[0], int(centre_y) + half_h + 1)
        samples = depth[top:bottom, left:right]
        maximum = float(self.get_parameter('maximum_depth_m').value)
        samples = samples[np.isfinite(samples) & (samples > 0.05) & (samples < maximum)]
        if samples.size < 5:
            return None
        z = float(np.median(samples))
        fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
        cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
        return (
            (centre_x - cx) * z / fx,
            (centre_y - cy) * z / fy,
            z,
        )

    def _run_ball(self, image, frame_id, image_s, depth, depth_s, matrix):
        result = self._predict(
            self._detector, image,
            float(self.get_parameter('ball_confidence').value))
        color = str(self.get_parameter('target_ball_color').value).lower()
        target_class = str(self.get_parameter(
            'red_ball_class' if color == 'red' else 'blue_ball_class').value)
        observations = []
        best_box = None
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            candidates = []
            depth_fresh = (
                depth is not None
                and abs(image_s - depth_s)
                <= float(self.get_parameter('depth_max_age_s').value)
            )
            for box, confidence, class_id in zip(boxes, confidences, class_ids):
                class_name = str(result.names[int(class_id)])
                if bool(self.get_parameter('publish_debug_image').value):
                    x1, y1, x2, y2 = (int(value) for value in box)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (150, 200, 50), 2)
                    cv2.putText(image, f'{class_name} {confidence:.2f} [2D]',
                                (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (150, 200, 50), 2)
                if class_name != target_class:
                    continue
                position = self._depth_position(
                    box, depth if depth_fresh else None, matrix)
                if position is None:
                    continue
                candidates.append((float(confidence), position[2], box, position))
            if candidates:
                confidence, _, best_box, position = max(
                    candidates, key=lambda item: (item[0], -item[1]))
                observations.append({
                    'class_name': target_class,
                    'confidence': confidence,
                    'position_m': list(position),
                })
        self._publish_json(self._ball_pub, {
            'schema': 'auv.ball_observations.v1',
            'frame_id': frame_id,
            'units': 'm',
            'observations': observations,
        })
        if best_box is not None:
            x1, y1, x2, y2 = (int(value) for value in best_box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 80, 255), 2)
            z = observations[0]['position_m'][2]
            cv2.putText(image, f'{target_class} {z:.2f}m', (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 255), 2)
        return len(observations)

    @staticmethod
    def _solve_pnp(points, matrix, distortion, width_m, height_m):
        object_points = np.asarray([
            [-width_m / 2.0, -height_m / 2.0, 0.0],
            [width_m / 2.0, -height_m / 2.0, 0.0],
            [width_m / 2.0, height_m / 2.0, 0.0],
            [-width_m / 2.0, height_m / 2.0, 0.0],
        ], dtype=np.float64)
        image_points = np.asarray(points, dtype=np.float64)
        ok, rotation, translation = cv2.solvePnP(
            object_points, image_points, matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE)
        if not ok or float(translation[2, 0]) <= 0.0:
            return None
        projected, _ = cv2.projectPoints(
            object_points, rotation, translation, matrix, distortion)
        error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
        return translation.reshape(3), error

    def _run_gate(self, image, frame_id, matrix, distortion, inference_image=None):
        result = self._predict(
            self._pose_model, image if inference_image is None else inference_image,
            float(self.get_parameter('gate_confidence').value))
        observations = []
        height, width = image.shape[:2]
        if matrix is None:
            fx = max(1.0, width / (2.0 * math.tan(math.radians(55.0))))
            cx = width / 2.0
        else:
            fx = float(matrix[0, 0])
            cx = float(matrix[0, 2])
        if result.boxes is not None and result.keypoints is not None:
            box_confidences = result.boxes.conf.cpu().numpy()
            points_all = result.keypoints.xy.cpu().numpy()
            keypoint_confidences = result.keypoints.conf
            if keypoint_confidences is None:
                scores_all = np.ones((len(points_all), 4), dtype=np.float32)
            else:
                scores_all = keypoint_confidences.cpu().numpy()
            for confidence, points, scores in zip(
                    box_confidences, points_all, scores_all):
                if len(points) != 4 or len(scores) != 4:
                    continue
                geometry = gate_image_geometry(points, width, height, fx, cx)
                minimum_score = float(np.min(scores))
                geometry_valid = (
                    geometry.valid
                    and minimum_score >= float(self.get_parameter(
                        'gate_keypoint_confidence').value)
                )
                translation = None
                reprojection = None
                if (
                    geometry_valid
                    and bool(self.get_parameter('enable_gate_pnp').value)
                    and matrix is not None
                    and distortion is not None
                ):
                    solution = self._solve_pnp(
                        points, matrix, distortion,
                        float(self.get_parameter('gate_width_m').value),
                        float(self.get_parameter('gate_height_m').value))
                    if solution is not None:
                        candidate, reprojection = solution
                        if reprojection <= float(self.get_parameter(
                                'maximum_reprojection_error_px').value):
                            translation = candidate.tolist()
                observation = {
                    'confidence': float(confidence),
                    'keypoints': [
                        {
                            'x_norm': float(point[0] / width),
                            'y_norm': float(point[1] / height),
                            'confidence': float(score),
                        }
                        for point, score in zip(points, scores)
                    ],
                    'center_error_norm': [
                        geometry.center_x_norm, geometry.center_y_norm],
                    'size_norm': [geometry.width_norm, geometry.height_norm],
                    'bearing_x_rad': geometry.bearing_x_rad,
                    'geometry_valid': geometry_valid,
                    'translation_m': translation,
                    'reprojection_error_px': reprojection,
                }
                observations.append(observation)
                colour = (0, 220, 0) if geometry_valid else (0, 0, 255)
                polygon = np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(image, [polygon], True, colour, 2)
                for index, point in enumerate(points):
                    xy = (int(point[0]), int(point[1]))
                    cv2.circle(image, xy, 5, colour, -1)
                    cv2.putText(image, str(index + 1), xy,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
        self._publish_json(self._gate_pub, {
            'schema': 'auv.gate_pose_observations.v1',
            'frame_id': frame_id,
            'observations': observations,
        })
        return sum(1 for item in observations if item['geometry_valid'])

    def _run_gate_search(self, image, frame_id, matrix, distortion):
        """Use the detector as a cheap proposal before running gate Pose."""
        result = self._predict(
            self._detector, image,
            float(self.get_parameter('gate_confidence').value))
        gate_class = str(self.get_parameter('gate_class').value)
        candidates = []
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            for box, confidence, class_id in zip(boxes, confidences, class_ids):
                if str(result.names[int(class_id)]) == gate_class:
                    candidates.append((float(confidence), box))
        if not candidates:
            self._publish_json(self._gate_pub, {
                'schema': 'auv.gate_pose_observations.v1',
                'frame_id': frame_id,
                'observations': [],
            })
            return 0, 'gate_detector'

        confidence, box = max(candidates, key=lambda item: item[0])
        x1, y1, x2, y2 = (int(value) for value in box)
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 180, 0), 2)
        cv2.putText(
            image, f'gate proposal {confidence:.2f}',
            (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 180, 0), 2)
        count = self._run_gate(image, frame_id, matrix, distortion)
        return count, 'gate_detector+gate_pose'

    def _publishing_allowed(self) -> bool:
        """Return false once either the worker or ROS context is stopping."""
        return not self._stop_event.is_set() and self.context.ok()

    def _publish_json(self, publisher, payload) -> bool:
        if not self._publishing_allowed():
            return False
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        try:
            publisher.publish(message)
        except Exception:
            # SIGINT can invalidate the rcl context between the check above
            # and publish(). This is a normal shutdown race, not an inference
            # failure and must not trigger a second fallback publication.
            if not self._publishing_allowed():
                return False
            raise
        return True

    def _publish_debug(self, image, frame_id: str) -> None:
        if (
            not self._publishing_allowed()
            or not bool(self.get_parameter('publish_debug_image').value)
        ):
            return
        now = self._now_s()
        fps = float(self.get_parameter('debug_max_fps').value)
        if not math.isfinite(fps) or fps <= 0:
            return
        if now - self._last_debug_s < 1.0 / fps:
            return
        self._last_debug_s = now
        quality = int(self.get_parameter('debug_jpeg_quality').value)
        quality = max(20, min(100, quality))
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        try:
            self._debug_pub.publish(message)
        except Exception:
            if self._publishing_allowed():
                raise

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._frame_event.wait(timeout=0.2):
                # No new camera frame: never rebroadcast the previous image as live.
                continue
            self._frame_event.clear()
            if not self._publishing_allowed():
                break
            with self._lock:
                if self._latest_image is None:
                    continue
                image = self._latest_image.copy()
                frame_id = self._latest_image_frame_id
                image_s = self._latest_image_s
                depth = None if self._latest_depth is None else self._latest_depth.copy()
                depth_s = self._latest_depth_s
                matrix = None if self._camera_matrix is None else self._camera_matrix.copy()
                distortion = None if self._distortion is None else self._distortion.copy()
                state = self._mission_state
            started = self._now_s()
            if started - image_s > 2.0:
                continue
            try:
                count = 0
                model_name = 'idle'
                if bool(self.get_parameter('vision_debug_all_models').value):
                    # Dedicated perception-only launch; never predict on drawn boxes.
                    clean_image = image.copy()
                    count = self._run_ball(
                        image, frame_id, image_s, depth, depth_s, matrix)
                    count += self._run_gate(
                        image, frame_id, matrix, distortion, inference_image=clean_image)
                    model_name = 'VISION_DEBUG ball+gate_pose'
                elif state in BALL_STATES:
                    model_name = 'ball_detector'
                    count = self._run_ball(
                        image, frame_id, image_s, depth, depth_s, matrix)
                elif state == 'SEARCH_GATE':
                    count, model_name = self._run_gate_search(
                        image, frame_id, matrix, distortion)
                elif state in GATE_POSE_STATES:
                    model_name = 'gate_pose'
                    count = self._run_gate(image, frame_id, matrix, distortion)
                else:
                    # Waiting/timed phases show fresh camera frames, no inference.
                    if not bool(self.get_parameter('publish_debug_image').value):
                        continue
                cv2.putText(
                    image, f'{state} | {model_name}', (16, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                self._publish_debug(image, frame_id)
                self._publish_json(self._diagnostics_pub, {
                    'state': state,
                    'active_model': model_name,
                    'accepted_observations': count,
                    'inference_ms': (self._now_s() - started) * 1000.0,
                    'frame_age_ms': max(0.0, (self._now_s() - image_s) * 1000.0),
                })
            except Exception as error:
                if not self._publishing_allowed():
                    break
                self.get_logger().error(
                    f'{state} perception failed; publishing empty frame: {error}')
                if state in BALL_STATES:
                    self._publish_json(self._ball_pub, {
                        'schema': 'auv.ball_observations.v1',
                        'frame_id': frame_id, 'units': 'm', 'observations': []})
                elif state == 'SEARCH_GATE' or state in GATE_POSE_STATES:
                    self._publish_json(self._gate_pub, {
                        'schema': 'auv.gate_pose_observations.v1',
                        'frame_id': frame_id, 'observations': []})

    def stop(self) -> None:
        self._stop_event.set()
        self._frame_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualModelPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may deliver another SIGINT while resources are being
        # destroyed. Keep normal supervised shutdown free of a traceback.
        try:
            node.stop()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
