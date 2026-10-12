#!/usr/bin/env python3
"""Operator-triggered bench test through the real competition ROS command path."""

from __future__ import annotations

import argparse
import json
import math
import time

from geometry_msgs.msg import Wrench
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


AXIS_LIMITS = {
    'surge': 100.0,
    'sway': 100.0,
    'heave': 250.0,
    'yaw': 30.0,
}

PROPELLERS = (
    'left_up', 'right_up', 'left_middle',
    'right_middle', 'left_down', 'right_down',
)


def bounded_float(minimum: float, maximum: float):
    def parse(value: str) -> float:
        parsed = float(value)
        if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(
                f'value must be in [{minimum}, {maximum}]')
        return parsed
    return parse


class CompetitionCommandProbe(Node):
    def __init__(self) -> None:
        super().__init__('competition_command_test')
        self.connected = False
        self.last_power: dict[str, float] | None = None
        self._capture_power = False
        self._power_samples: list[dict[str, float]] = []
        self.command_pub = self.create_publisher(
            Wrench, '/auv/control_command', 1)
        self.enable_pub = self.create_publisher(
            Bool, '/auv/motion_enable', 1)
        self.create_subscription(
            Bool, '/auv/connected', self._connected_callback, 5)
        self.create_subscription(
            String, '/auv/propeller_power', self._power_callback, 5)

    def _connected_callback(self, message: Bool) -> None:
        self.connected = bool(message.data)

    def _power_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            self.last_power = {name: float(data[name]) for name in PROPELLERS}
            if self._capture_power:
                self._power_samples.append(dict(self.last_power))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.last_power = None

    def publish(self, axis: str | None = None, effort: float = 0.0,
                enabled: bool = False) -> None:
        command = Wrench()
        if axis == 'surge':
            command.force.x = effort
        elif axis == 'sway':
            command.force.y = effort
        elif axis == 'heave':
            command.force.z = effort
        elif axis == 'yaw':
            command.torque.z = effort
        self.command_pub.publish(command)
        permit = Bool()
        permit.data = enabled
        self.enable_pub.publish(permit)

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def wait_ready(self, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.connected and self.command_pub.get_subscription_count() == 1
                    and self.enable_pub.get_subscription_count() == 1):
                return
        raise RuntimeError(
            'bridge not ready: require connected=true and exactly one subscriber '
            'on each competition command topic')

    def stop_and_verify(self, rate: float) -> None:
        period = 1.0 / rate
        for _ in range(6):
            self.publish(enabled=False)
            rclpy.spin_once(self, timeout_sec=period)
        self.spin_for(0.25)
        if self.last_power is None:
            raise RuntimeError(
                'no /auv/propeller_power feedback; launch the bridge with '
                'propeller_feedback_rate_hz greater than zero')
        nonzero = {
            name: value for name, value in self.last_power.items()
            if abs(value) > 1e-6
        }
        if nonzero:
            raise RuntimeError(f'final zero verification failed: {nonzero}')

    def run_stage(self, axis: str, effort: float,
                  duration: float, rate: float) -> dict[str, float]:
        period = 1.0 / rate
        deadline = time.monotonic() + duration
        self._power_samples.clear()
        self._capture_power = True
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                started = time.monotonic()
                self.publish(axis=axis, effort=effort, enabled=True)
                rclpy.spin_once(self, timeout_sec=0.0)
                remaining = period - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
            self.spin_for(0.12)
        finally:
            self._capture_power = False
        nonzero_samples = [
            sample for sample in self._power_samples
            if any(abs(value) > 1e-6 for value in sample.values())
        ]
        if not nonzero_samples:
            raise RuntimeError(
                f'no non-zero propeller feedback observed during {axis} stage')
        return max(
            nonzero_samples,
            key=lambda sample: sum(abs(value) for value in sample.values()),
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Publish guarded Wrench commands through the same ROS topics used '
            'by the competition controller. Every motion stage requires an '
            'operator confirmation and is followed by verified zero output.'
        ))
    parser.add_argument(
        '--axes', nargs='+', choices=tuple(AXIS_LIMITS), default=['surge'],
        help='competition axes to test; the safest first run is surge only')
    parser.add_argument(
        '--fraction', type=bounded_float(0.25, 0.35), default=0.25,
        help='fraction of the configured competition command limit')
    parser.add_argument(
        '--duration', type=bounded_float(0.30, 1.00), default=0.50,
        help='seconds per direction')
    parser.add_argument(
        '--rate', type=bounded_float(10.0, 30.0), default=20.0,
        help='command publication rate in Hz')
    parser.add_argument(
        '--allow-motion', action='store_true',
        help='required because this tool deliberately rotates real thrusters')
    return parser


def main(args=None) -> int:
    options = build_parser().parse_args(args)
    if not options.allow_motion:
        raise SystemExit(
            'motion refused: restrain the vehicle, clear the propellers, keep '
            'the power cut-off in hand, then add --allow-motion')

    rclpy.init()
    node = CompetitionCommandProbe()
    result = 0
    try:
        node.wait_ready()
        node.stop_and_verify(options.rate)
        print('Bridge ready and six software outputs are zero.')
        print('Ctrl+C at any time disables motion and sends repeated zero commands.')

        for axis in options.axes:
            for direction in (1.0, -1.0):
                effort = direction * options.fraction * AXIS_LIMITS[axis]
                token = f'RUN {axis} {"+" if direction > 0 else "-"}'
                entered = input(
                    f'\nNext: {axis} effort={effort:+.2f} for '
                    f'{options.duration:.2f}s. Type "{token}" to continue: '
                ).strip()
                if entered != token:
                    print('Stage skipped; command did not match exactly.')
                    continue
                print('Motion starts in 3 seconds...')
                for remaining in (3, 2, 1):
                    print(remaining, flush=True)
                    time.sleep(1.0)
                stage_power = node.run_stage(
                    axis, effort, options.duration, options.rate)
                node.stop_and_verify(options.rate)
                print(f'PASS {axis} {direction:+.0f}; '
                      f'stage_power={stage_power}; zero={node.last_power}')
    except KeyboardInterrupt:
        print('\nInterrupted by operator; returning to zero.')
    except RuntimeError as exc:
        print(f'FAIL: {exc}')
        result = 1
    finally:
        try:
            node.stop_and_verify(options.rate)
            print(f'FINAL ZERO VERIFIED: {node.last_power}')
        except RuntimeError as exc:
            print(f'FINAL ZERO FAILED: {exc}')
            result = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
