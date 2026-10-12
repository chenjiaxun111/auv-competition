#!/usr/bin/env python3
"""ROS 2 bridge between Jetson autonomy and the legacy ``rov-master``."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import replace
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Wrench
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rov_thruster_bridge.motion_mapper import MotionCommand, MotionMapper
from rov_thruster_bridge.rpc_client import RovRpcClient, RovRpcError, RovTelemetry
from std_msgs.msg import Bool, Float32, String


class RovBridgeNode(Node):
    """Own all network I/O and expose a non-blocking ROS-facing interface."""

    def __init__(self) -> None:
        super().__init__('rov_bridge')

        self.declare_parameter('rov_host', '192.168.137.219')
        self.declare_parameter('rov_port', 8888)
        self.declare_parameter('rpc_timeout_s', 0.20)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('telemetry_rate_hz', 10.0)
        self.declare_parameter('propeller_feedback_rate_hz', 0.0)
        self.declare_parameter('command_timeout_s', 0.30)
        self.declare_parameter('telemetry_timeout_s', 0.50)
        self.declare_parameter('reconnect_interval_s', 1.0)
        self.declare_parameter('minimum_depth_m', 0.15)
        self.declare_parameter('maximum_depth_m', 1.10)
        self.declare_parameter('onboard_depth_hold_enabled', False)
        self.declare_parameter('onboard_direction_hold_enabled', False)
        # The competition stack uses positive-down depth. auvmaster 2.x is
        # currently configured with negative-down depth, hence the default -1.
        self.declare_parameter('sensor_sign.depth', -1.0)

        # Defaults match the existing auv_pid_tracker output limits.
        self.declare_parameter('command_limits.max_surge', 100.0)
        self.declare_parameter('command_limits.max_sway', 100.0)
        self.declare_parameter('command_limits.max_heave', 250.0)
        self.declare_parameter('command_limits.max_yaw', 30.0)
        self.declare_parameter('axis_sign.surge', 1.0)
        self.declare_parameter('axis_sign.sway', 1.0)
        self.declare_parameter('axis_sign.heave', 1.0)
        self.declare_parameter('axis_sign.yaw', 1.0)

        self._send_rate = self._positive_parameter('send_rate_hz')
        self._telemetry_rate = self._positive_parameter('telemetry_rate_hz')
        self._propeller_feedback_rate = float(
            self.get_parameter('propeller_feedback_rate_hz').value)
        if (not math.isfinite(self._propeller_feedback_rate)
                or self._propeller_feedback_rate < 0.0):
            raise ValueError(
                'propeller_feedback_rate_hz must be finite and non-negative')
        self._command_timeout = self._positive_parameter('command_timeout_s')
        self._telemetry_timeout = self._positive_parameter('telemetry_timeout_s')
        self._reconnect_interval = self._positive_parameter('reconnect_interval_s')
        self._minimum_depth = float(self.get_parameter('minimum_depth_m').value)
        self._maximum_depth = float(self.get_parameter('maximum_depth_m').value)
        if not 0.0 <= self._minimum_depth < self._maximum_depth:
            raise ValueError('depth limits must satisfy 0 <= minimum < maximum')
        self._onboard_depth_hold_enabled = bool(
            self.get_parameter('onboard_depth_hold_enabled').value)
        self._onboard_direction_hold_enabled = bool(
            self.get_parameter('onboard_direction_hold_enabled').value)
        self._depth_sign = float(self.get_parameter('sensor_sign.depth').value)
        if self._depth_sign not in (-1.0, 1.0):
            raise ValueError('parameter sensor_sign.depth must be -1 or 1')

        self._mapper = MotionMapper(
            max_surge=self._positive_parameter('command_limits.max_surge'),
            max_sway=self._positive_parameter('command_limits.max_sway'),
            max_heave=self._positive_parameter('command_limits.max_heave'),
            max_yaw=self._positive_parameter('command_limits.max_yaw'),
            surge_sign=float(self.get_parameter('axis_sign.surge').value),
            sway_sign=float(self.get_parameter('axis_sign.sway').value),
            heave_sign=float(self.get_parameter('axis_sign.heave').value),
            yaw_sign=float(self.get_parameter('axis_sign.yaw').value),
        )

        self._rpc = RovRpcClient(
            host=str(self.get_parameter('rov_host').value),
            port=int(self.get_parameter('rov_port').value),
            timeout=self._positive_parameter('rpc_timeout_s'),
        )

        self._state_lock = threading.Lock()
        self._latest_command = MotionCommand.zero()
        self._last_command_time: Optional[float] = None
        self._motion_enabled = False
        self._pending_target_depth: Optional[float] = None
        self._latest_telemetry: Optional[RovTelemetry] = None
        self._last_telemetry_time: Optional[float] = None
        self._last_propeller_power = None
        self._connected = False
        self._last_reported_motion_enabled: Optional[bool] = None
        self._stop_event = threading.Event()

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        telemetry_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._command_sub = self.create_subscription(
            Wrench, 'control_command', self._command_callback, command_qos)
        self._enable_sub = self.create_subscription(
            Bool, 'motion_enable', self._enable_callback, command_qos)
        self._target_depth_sub = self.create_subscription(
            Float32, 'target_depth_m', self._target_depth_callback, command_qos)

        self._depth_pub = self.create_publisher(Float32, 'depth_m', telemetry_qos)
        self._yaw_pub = self.create_publisher(Float32, 'yaw_deg', telemetry_qos)
        self._temperature_pub = self.create_publisher(
            Float32, 'water_temperature_c', telemetry_qos)
        self._connected_pub = self.create_publisher(Bool, 'connected', telemetry_qos)
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, 'bridge_diagnostics', telemetry_qos)
        self._propeller_power_pub = self.create_publisher(
            String, 'propeller_power', telemetry_qos)

        self._worker = threading.Thread(
            target=self._io_loop, name='rov-json-rpc', daemon=True)
        self._worker.start()
        self._connection_timer = self.create_timer(
            0.5, self._publish_connection_status)

        self.get_logger().info(
            f'ROV bridge started for {self._rpc.host}:{self._rpc.port}; '
            'motion is DISABLED until motion_enable=true')

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'parameter {name} must be finite and greater than zero')
        return value

    def _safe_publish(self, publisher, message) -> None:
        """Ignore only the expected publish race during ROS context shutdown."""
        try:
            publisher.publish(message)
        except Exception:
            if self._stop_event.is_set() or not rclpy.ok():
                return
            raise

    def _command_callback(self, message: Wrench) -> None:
        command = self._mapper.map(
            surge=message.force.x,
            sway=message.force.y,
            heave=message.force.z,
            yaw=message.torque.z,
        )
        with self._state_lock:
            self._latest_command = command
            self._last_command_time = time.monotonic()

    def _enable_callback(self, message: Bool) -> None:
        enabled = bool(message.data)
        with self._state_lock:
            self._motion_enabled = enabled
            if not self._motion_enabled:
                self._latest_command = MotionCommand.zero()
                self._last_command_time = None
        if enabled != self._last_reported_motion_enabled:
            self.get_logger().info(
                f'motion {"ENABLED" if enabled else "DISABLED"}')
            self._last_reported_motion_enabled = enabled

    def _target_depth_callback(self, message: Float32) -> None:
        if not self._onboard_depth_hold_enabled:
            self.get_logger().warning(
                'ignored target depth because onboard depth hold is disabled')
            return
        target = float(message.data)
        if not math.isfinite(target):
            self.get_logger().error('rejected non-finite target depth')
            return
        if not self._minimum_depth <= target <= self._maximum_depth:
            self.get_logger().error(
                f'rejected target depth {target:.3f} m outside '
                f'[{self._minimum_depth:.3f}, {self._maximum_depth:.3f}] m')
            return
        with self._state_lock:
            self._pending_target_depth = target
        self.get_logger().info(f'queued absolute target depth {target:.3f} m')

    def _snapshot(self):
        with self._state_lock:
            return (
                self._latest_command,
                self._last_command_time,
                self._motion_enabled,
                self._pending_target_depth,
                self._latest_telemetry,
                self._last_telemetry_time,
            )

    def _set_connected(self, connected: bool) -> None:
        changed = connected != self._connected
        self._connected = connected
        self._publish_connection_status()
        if changed:
            # Humble binds a call site to its first severity and raises when the
            # same call site later logs at another severity. Keep two explicit
            # call sites so a normal connect/disconnect cycle cannot kill the
            # I/O worker thread.
            if connected:
                self.get_logger().info('ROV link connected')
            else:
                self.get_logger().error('ROV link disconnected')

    def _publish_connection_status(self) -> None:
        message = Bool()
        message.data = self._connected
        self._safe_publish(self._connected_pub, message)

    def _publish_telemetry(self, telemetry: RovTelemetry) -> None:
        if telemetry.depth_valid:
            depth = Float32()
            depth.data = telemetry.depth_m
            self._safe_publish(self._depth_pub, depth)
        if telemetry.yaw_valid:
            yaw = Float32()
            yaw.data = telemetry.yaw_deg
            self._safe_publish(self._yaw_pub, yaw)
        temperature = Float32()
        temperature.data = telemetry.temperature_c
        self._safe_publish(self._temperature_pub, temperature)

    def _publish_propeller_power(self, power) -> None:
        keys = (
            'left_up', 'right_up', 'left_middle',
            'right_middle', 'left_down', 'right_down',
        )
        try:
            normalized = {name: float(power[name]) for name in keys}
        except (KeyError, TypeError, ValueError) as exc:
            raise RovRpcError(
                f'invalid propeller_power response: {exc}') from exc
        if not all(math.isfinite(value) for value in normalized.values()):
            raise RovRpcError('propeller_power contains a non-finite value')
        message = String()
        message.data = json.dumps(normalized, separators=(',', ':'))
        self._safe_publish(self._propeller_power_pub, message)
        if normalized != self._last_propeller_power:
            self.get_logger().info(
                'NanoPi PWM offsets: '
                + ' '.join(
                    f'{name}={normalized[name]:+.0f}' for name in keys))
            self._last_propeller_power = normalized

    def _normalize_telemetry(self, telemetry: RovTelemetry) -> RovTelemetry:
        return replace(telemetry, depth_m=self._depth_sign * telemetry.depth_m)

    def _publish_diagnostic(self, level: int, message: str) -> None:
        now = time.monotonic()
        _, last_command, enabled, target, telemetry, last_telemetry = self._snapshot()
        command_age = math.inf if last_command is None else now - last_command
        telemetry_age = math.inf if last_telemetry is None else now - last_telemetry
        status = DiagnosticStatus()
        status.name = f'{self.get_namespace()}/rov_bridge'
        status.hardware_id = f'{self._rpc.host}:{self._rpc.port}'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='connected', value=str(self._connected).lower()),
            KeyValue(key='motion_enabled', value=str(enabled).lower()),
            KeyValue(key='command_age_s', value=f'{command_age:.3f}'),
            KeyValue(key='telemetry_age_s', value=f'{telemetry_age:.3f}'),
            # Report validity with the value: yaw_deg alone stops publishing
            # on sensor failure, so a viewer must not reuse its last value.
            KeyValue(key='yaw_valid', value=str(
                telemetry is not None and telemetry.yaw_valid).lower()),
            KeyValue(key='yaw_deg', value=str(
                telemetry.yaw_deg if telemetry is not None else math.nan)),
            KeyValue(key='pending_target_depth_m', value=str(target)),
            KeyValue(
                key='depth_target_rpc_enabled',
                value=str(self._onboard_depth_hold_enabled).lower()),
            KeyValue(
                key='heading_target_rpc_enabled',
                value=str(self._onboard_direction_hold_enabled).lower()),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._safe_publish(self._diagnostic_pub, array)

    def _initialize_remote(self) -> None:
        # ``status`` is the auvmaster 2.x heartbeat source. Read it before the
        # initial zero so the embedded AUV controller is in a known live state.
        telemetry = self._normalize_telemetry(self._rpc.get_telemetry())
        received_at = time.monotonic()
        with self._state_lock:
            self._latest_telemetry = telemetry
            self._last_telemetry_time = received_at
        self._publish_telemetry(telemetry)
        self._rpc.stop()

    def _set_remote_enable(self, enabled: bool) -> None:
        # auvmaster 2.x has no remote PID enable switches. In AUV mode it holds
        # valid depth/yaw sensors automatically whenever the corresponding
        # manual axis is zero. A transition still sends a zero before changing
        # local command ownership.
        del enabled
        self._rpc.stop()

    def _try_send_target_depth(
        self,
        target: Optional[float],
        telemetry: Optional[RovTelemetry],
        telemetry_time: Optional[float],
        enabled: bool,
    ) -> None:
        if (target is None or telemetry is None or telemetry_time is None
                or not telemetry.depth_valid or not enabled):
            return
        if time.monotonic() - telemetry_time > self._telemetry_timeout:
            return
        delta = target - telemetry.depth_m
        if abs(delta) >= 0.005:
            # Convert the ROS positive-down delta back to the firmware's depth
            # sign convention before calling ``auv_target``.
            self._rpc.change_depth(self._depth_sign * delta)
        with self._state_lock:
            if self._pending_target_depth == target:
                self._pending_target_depth = None
        self.get_logger().info(
            f'sent target depth {target:.3f} m as relative change {delta:+.3f} m')

    def _io_loop(self) -> None:
        send_period = 1.0 / self._send_rate
        telemetry_period = 1.0 / self._telemetry_rate
        next_send = time.monotonic()
        next_telemetry = time.monotonic()
        next_propeller = time.monotonic()
        remote_enabled: Optional[bool] = None

        while not self._stop_event.is_set():
            if not self._connected:
                try:
                    self._initialize_remote()
                    remote_enabled = False
                    self._set_connected(True)
                    self._publish_diagnostic(DiagnosticStatus.OK, 'connected and disarmed')
                    next_send = time.monotonic()
                    next_telemetry = time.monotonic()
                    next_propeller = time.monotonic()
                except RovRpcError as exc:
                    self._set_connected(False)
                    self._publish_diagnostic(DiagnosticStatus.ERROR, str(exc))
                    self._stop_event.wait(self._reconnect_interval)
                    continue

            now = time.monotonic()
            try:
                if now >= next_telemetry:
                    telemetry = self._normalize_telemetry(
                        self._rpc.get_telemetry())
                    received_at = time.monotonic()
                    with self._state_lock:
                        self._latest_telemetry = telemetry
                        self._last_telemetry_time = received_at
                    self._publish_telemetry(telemetry)
                    next_telemetry = received_at + telemetry_period

                if (self._propeller_feedback_rate > 0.0
                        and now >= next_propeller):
                    self._publish_propeller_power(
                        self._rpc.get_propeller_power())
                    next_propeller = (
                        time.monotonic()
                        + 1.0 / self._propeller_feedback_rate
                    )

                snapshot = self._snapshot()
                command, command_time, enabled, target, telemetry, telemetry_time = snapshot

                if remote_enabled != enabled:
                    self._set_remote_enable(enabled)
                    remote_enabled = enabled

                self._try_send_target_depth(
                    target, telemetry, telemetry_time, enabled)

                now = time.monotonic()
                if now >= next_send:
                    command_is_fresh = (
                        command_time is not None
                        and now - command_time <= self._command_timeout)
                    output = command if enabled and command_is_fresh else MotionCommand.zero()
                    self._rpc.move(
                        sway=output.sway,
                        surge=output.surge,
                        heave=output.heave,
                        yaw=output.yaw,
                    )
                    next_send = time.monotonic() + send_period
                    if enabled and not command_is_fresh:
                        self._publish_diagnostic(
                            DiagnosticStatus.WARN, 'command stale; zero motion sent')
                    elif telemetry is not None and (
                            not telemetry.depth_valid or not telemetry.yaw_valid):
                        self._publish_diagnostic(
                            DiagnosticStatus.WARN,
                            'one or more controller sensors are invalid')
                    else:
                        self._publish_diagnostic(DiagnosticStatus.OK, 'operational')
            except RovRpcError as exc:
                self._set_connected(False)
                remote_enabled = None
                self._publish_diagnostic(DiagnosticStatus.ERROR, str(exc))
                continue

            next_event = min(next_send, next_telemetry)
            if self._propeller_feedback_rate > 0.0:
                next_event = min(next_event, next_propeller)
            self._stop_event.wait(max(0.001, min(0.02, next_event - time.monotonic())))

    def destroy_node(self) -> None:
        context_ok = rclpy.ok()
        if context_ok:
            self.get_logger().info(
                'stopping ROV bridge and sending best-effort zero motion')
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=max(1.0, 4.0 * self._rpc.timeout))
        try:
            self._rpc.stop()
        except RovRpcError as exc:
            if context_ok:
                self.get_logger().error(
                    f'final stop was not acknowledged: {exc}')
        self._rpc.close()
        super().destroy_node()


# Preserve the old class name for external imports.
ThrusterBridgeNode = RovBridgeNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RovBridgeNode()
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
