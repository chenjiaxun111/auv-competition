#!/usr/bin/env python3
"""ROS 2 adapter for the competition planner core."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import PoseStamped, Wrench
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool

from auv_competition_control.controller_core import (
    BallObservation,
    CompetitionPlanner,
    GatePoseObservation,
    PlannerConfig,
)
from auv_competition_control.perception_adapter import (
    parse_ball_observations,
    parse_gate_pose_observations,
    parse_objects3d,
)


class CompetitionControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('competition_controller')
        defaults = PlannerConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('depth_publish_period_s', 0.25)
        self.declare_parameter('depth_publish_change_m', 0.01)
        self.declare_parameter('objects_topic', '/zed/objects3d')
        self.declare_parameter(
            'ball_observations_topic', '/auv/perception/ball_observations')
        self.declare_parameter(
            'gate_pose_observations_topic',
            '/auv/perception/gate_pose_observations')
        self.declare_parameter('use_legacy_objects3d', False)
        self.declare_parameter('shadow_mode', False)
        self.declare_parameter('shadow_auto_start', False)
        self.declare_parameter('shadow_assume_initial_depth', False)
        self.declare_parameter('shadow_log_rate_hz', 2.0)

        self._shadow_mode = bool(self.get_parameter('shadow_mode').value)
        self._shadow_auto_start = bool(
            self.get_parameter('shadow_auto_start').value)
        self._shadow_assume_initial_depth = bool(
            self.get_parameter('shadow_assume_initial_depth').value)
        if (self._shadow_auto_start or self._shadow_assume_initial_depth) \
                and not self._shadow_mode:
            raise ValueError('shadow-only options require shadow_mode=true')

        config_values = {
            name: self.get_parameter(name).value for name in vars(defaults)
        }
        if self._shadow_mode:
            # The controller itself only publishes into the isolated shadow
            # namespace. A dedicated bench launch may explicitly bridge that
            # namespace after applying its own interlocks and output limits.
            config_values['require_bridge_connection'] = False
        self._planner = CompetitionPlanner(PlannerConfig(**config_values))
        self._frame_id = 'zed_left_camera_optical_frame'
        self._last_state = None
        self._last_startup_status = None
        self._last_depth_sent = None
        self._last_depth_sent_s = -math.inf
        self._last_perception_s = None
        self._raw_object_count = 0
        self._accepted_object_count = 0
        self._rejected_object_count = 0
        self._parse_error_count = 0
        self._last_shadow_log_s = -math.inf

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._objects_sub = None
        if bool(self.get_parameter('use_legacy_objects3d').value):
            self._objects_sub = self.create_subscription(
                String, str(self.get_parameter('objects_topic').value),
                self._objects_callback, sensor_qos)
            self.get_logger().warning(
                'Legacy objects3d adapter enabled; use it only during migration')
        self._ball_sub = self.create_subscription(
            String,
            str(self.get_parameter('ball_observations_topic').value),
            self._ball_callback,
            sensor_qos,
        )
        self._gate_sub = self.create_subscription(
            String,
            str(self.get_parameter('gate_pose_observations_topic').value),
            self._gate_callback,
            sensor_qos,
        )
        self._depth_sub = self.create_subscription(
            Float32, '/auv/depth_m', self._depth_callback, sensor_qos)
        self._connected_sub = self.create_subscription(
            Bool, '/auv/connected', self._connected_callback, sensor_qos)
        output_prefix = '/auv/shadow' if self._shadow_mode else '/auv'
        self._command_pub = self.create_publisher(
            Wrench, f'{output_prefix}/control_command', command_qos)
        enable_name = (
            f'{output_prefix}/requested_motion_enable'
            if self._shadow_mode else f'{output_prefix}/motion_enable'
        )
        self._enable_pub = self.create_publisher(Bool, enable_name, command_qos)
        self._target_depth_pub = self.create_publisher(
            Float32, f'{output_prefix}/target_depth_m', command_qos)
        self._state_pub = self.create_publisher(
            String, '/auv/mission_state', command_qos)
        # Keep the existing state string contract unchanged for perception.
        self._progress_pub = self.create_publisher(
            String, f'{output_prefix}/mission_progress', command_qos)
        self._target_pub = self.create_publisher(
            PoseStamped, '/auv/selected_target', sensor_qos)
        self._decision_pub = None
        if self._shadow_mode:
            self._decision_pub = self.create_publisher(
                String, '/auv/shadow/decision', sensor_qos)
        self._arm_service = self.create_service(
            SetBool, '/auv/mission_enable', self._arm_callback)

        rate = float(self.get_parameter('control_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self._timer = self.create_timer(1.0 / rate, self._control_tick)
        if self._shadow_mode:
            if self._shadow_assume_initial_depth:
                self._planner.set_depth(self._planner.config.initial_depth_m)
            if self._shadow_auto_start:
                self._planner.set_armed(True, self._now_s())
            self.get_logger().warning(
                'SHADOW MODE: this node creates no actuator-topic publishers; '
                'intent is published only under /auv/shadow/*')
        else:
            self.get_logger().info(
                'Competition controller ready but DISARMED; call '
                '/auv/mission_enable before motion')

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _objects_callback(self, message: String) -> None:
        now_s = self._now_s()
        try:
            frame = parse_objects3d(message.data, now_s)
            self._frame_id = frame.frame_id
            self._last_perception_s = now_s
            self._raw_object_count = frame.raw_object_count
            self._accepted_object_count = len(frame.detections)
            self._rejected_object_count = frame.rejected_object_count
            self._planner.update_detections(frame.detections, now_s)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self._parse_error_count += 1
            self.get_logger().warning(f'Ignored malformed objects3d message: {error}')

    def _ball_callback(self, message: String) -> None:
        now_s = self._now_s()
        try:
            frame = parse_ball_observations(message.data, now_s)
            self._frame_id = frame.frame_id
            self._last_perception_s = now_s
            self._raw_object_count = frame.raw_count
            self._accepted_object_count = len(frame.observations)
            self._rejected_object_count = frame.rejected_count
            self._planner.update_ball_observations(frame.observations, now_s)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self._parse_error_count += 1
            self.get_logger().warning(
                f'Ignored malformed ball observation: {error}')

    def _gate_callback(self, message: String) -> None:
        now_s = self._now_s()
        try:
            frame = parse_gate_pose_observations(message.data, now_s)
            self._frame_id = frame.frame_id
            self._last_perception_s = now_s
            self._raw_object_count = frame.raw_count
            self._accepted_object_count = len(frame.observations)
            self._rejected_object_count = frame.rejected_count
            self._planner.update_gate_observations(frame.observations, now_s)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self._parse_error_count += 1
            self.get_logger().warning(
                f'Ignored malformed gate Pose observation: {error}')

    def _depth_callback(self, message: Float32) -> None:
        self._planner.set_depth(float(message.data))

    def _connected_callback(self, message: Bool) -> None:
        self._planner.set_connected(bool(message.data), self._now_s())

    def _arm_callback(self, request: SetBool.Request, response: SetBool.Response):
        self._planner.set_armed(bool(request.data), self._now_s())
        response.success = True
        if self._shadow_mode:
            response.message = (
                'shadow mission running; publishing isolated intent'
                if request.data else 'shadow mission stopped'
            )
        else:
            response.message = 'mission armed' if request.data else 'mission disarmed'
        return response

    @staticmethod
    def _decision_reason(output) -> str:
        if not output.motion_enable:
            return 'motion_not_requested_in_current_state'
        if output.selected_target is not None:
            return 'visual_target_selected'
        if output.state.value.startswith('SEARCH_'):
            return 'searching_for_confirmed_target'
        if output.state.value in ('HIT_BALL', 'RETREAT', 'PASS_GATE'):
            return 'timed_mission_segment'
        return 'planner_output'

    def _publish_shadow_decision(self, output, now_s: float) -> None:
        if self._decision_pub is None:
            return
        target_data = None
        if output.selected_target is not None:
            item = output.selected_target
            if isinstance(item, BallObservation):
                target_data = {
                    'type': 'ball',
                    'class_name': item.class_name,
                    'confidence': item.confidence,
                    'x_m': item.x,
                    'y_m': item.y,
                    'z_m': item.z,
                    'age_s': max(0.0, now_s - item.received_s),
                    'frame_id': self._frame_id,
                }
            else:
                target_data = {
                    'type': 'gate_pose',
                    'confidence': item.confidence,
                    'minimum_keypoint_confidence': (
                        item.minimum_keypoint_confidence),
                    'center_error_norm': [
                        item.center_x_norm, item.center_y_norm],
                    'size_norm': [item.width_norm, item.height_norm],
                    'bearing_x_rad': item.bearing_x_rad,
                    'range_m': item.range_m,
                    'age_s': max(0.0, now_s - item.received_s),
                    'frame_id': self._frame_id,
                }
        perception_age = (
            None if self._last_perception_s is None
            else max(0.0, now_s - self._last_perception_s)
        )
        decision = {
            'mode': 'SHADOW_NO_ACTUATION',
            'state': output.state.value,
            'armed': self._planner.armed,
            'requested_motion_enable': output.motion_enable,
            'requested_command': {
                'surge': output.surge,
                'sway': output.sway,
                'heave': output.heave,
                'yaw': output.yaw,
            },
            'target_depth_m': output.target_depth_m,
            'selected_target': target_data,
            'perception': {
                'message_age_s': perception_age,
                'raw_objects': self._raw_object_count,
                'accepted_objects': self._accepted_object_count,
                'rejected_objects': self._rejected_object_count,
                'parse_errors': self._parse_error_count,
            },
            'reason': self._decision_reason(output),
        }
        message = String()
        message.data = json.dumps(
            decision, ensure_ascii=False, separators=(',', ':'))
        self._decision_pub.publish(message)

        log_rate = float(self.get_parameter('shadow_log_rate_hz').value)
        if not math.isfinite(log_rate) or log_rate <= 0.0:
            log_rate = 2.0
        if now_s - self._last_shadow_log_s >= 1.0 / log_rate:
            command = decision['requested_command']
            if target_data is None:
                target_text = 'none'
            elif target_data['type'] == 'ball':
                target_text = (
                    f"{target_data['class_name']} "
                    f"xyz=({target_data['x_m']:+.2f},"
                    f"{target_data['y_m']:+.2f},{target_data['z_m']:.2f})m"
                )
            else:
                target_text = (
                    'gate_pose center=(%+.2f,%+.2f) width=%.2f range=%s' % (
                        target_data['center_error_norm'][0],
                        target_data['center_error_norm'][1],
                        target_data['size_norm'][0],
                        'none' if target_data['range_m'] is None
                        else f"{target_data['range_m']:.2f}m",
                    )
                )
            self.get_logger().info(
                'SHADOW state=%s target=%s intent='
                '(surge=%+.1f sway=%+.1f heave=%+.1f yaw=%+.1f) '
                'depth=%s perception_age=%s' % (
                    output.state.value,
                    target_text,
                    command['surge'], command['sway'], command['heave'],
                    command['yaw'],
                    ('disabled' if output.target_depth_m is None
                     else f'{output.target_depth_m:.2f}m'),
                    'none' if perception_age is None
                    else f'{perception_age:.3f}s',
                ))
            self._last_shadow_log_s = now_s

    def _publish_depth_if_needed(self, target_depth_m: float, now_s: float) -> None:
        period = float(self.get_parameter('depth_publish_period_s').value)
        threshold = float(self.get_parameter('depth_publish_change_m').value)
        changed = (
            self._last_depth_sent is None
            or abs(target_depth_m - self._last_depth_sent) >= threshold
        )
        if changed and now_s - self._last_depth_sent_s >= period:
            message = Float32()
            message.data = float(target_depth_m)
            self._target_depth_pub.publish(message)
            self._last_depth_sent = target_depth_m
            self._last_depth_sent_s = now_s

    def _control_tick(self) -> None:
        now_s = self._now_s()
        output = self._planner.tick(now_s)

        command = Wrench()
        command.force.x = float(output.surge)
        command.force.y = float(output.sway)
        command.force.z = float(output.heave)
        command.torque.z = float(output.yaw)
        self._command_pub.publish(command)

        enabled = Bool()
        enabled.data = output.motion_enable
        self._enable_pub.publish(enabled)

        if output.target_depth_m is not None and output.motion_enable:
            self._publish_depth_if_needed(output.target_depth_m, now_s)

        state = String()
        state.data = output.state.value
        self._state_pub.publish(state)
        progress = self._planner.startup_progress(now_s)
        progress_message = String()
        progress_message.data = json.dumps(progress, allow_nan=False)
        self._progress_pub.publish(progress_message)
        remaining = progress['remaining_s']
        startup_status = None
        if remaining is not None:
            seconds = math.ceil(remaining)
            if state.data == 'START_DELAY':
                startup_status = f'出发等待：剩余 {seconds} 秒（零指令，运动使能关闭）'
            elif state.data == 'DEPARTURE_FORWARD':
                startup_status = (
                    f'直行中：剩余 {seconds} 秒；前进指令 surge='
                    f'{progress["departure_surge"]:g}（非速度/转速反馈）')
        # At most once per countdown second, not 20 log lines per second.
        if startup_status is not None and startup_status != self._last_startup_status:
            self.get_logger().info(startup_status)
        self._last_startup_status = startup_status
        if state.data != self._last_state:
            self.get_logger().info(
                f'State -> {state.data}; gates={self._planner.gates_passed}')
            self._last_state = state.data

        if output.selected_target is not None:
            item = output.selected_target
            position = None
            if isinstance(item, BallObservation):
                position = (item.x, item.y, item.z)
            elif (
                isinstance(item, GatePoseObservation)
                and item.range_m is not None
                and item.translation_x_m is not None
                and item.translation_y_m is not None
            ):
                position = (
                    item.translation_x_m,
                    item.translation_y_m,
                    item.range_m,
                )
            if position is not None:
                target = PoseStamped()
                target.header.stamp = self.get_clock().now().to_msg()
                target.header.frame_id = self._frame_id
                target.pose.position.x = position[0]
                target.pose.position.y = position[1]
                target.pose.position.z = position[2]
                target.pose.orientation.w = 1.0
                self._target_pub.publish(target)
        if self._shadow_mode:
            self._publish_shadow_decision(output, now_s)

    def stop(self) -> None:
        command = Wrench()
        self._command_pub.publish(command)
        enabled = Bool()
        enabled.data = False
        self._enable_pub.publish(enabled)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may deliver SIGINT to the child while cleanup is already
        # in progress.  Keep a second interrupt from producing a misleading
        # traceback after the controller has stopped normally.
        try:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
