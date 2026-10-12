#!/usr/bin/env python3
"""Run a guarded four-axis PWM/mixer bench test against auvmaster 2.x."""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Mapping

from rov_thruster_bridge.rpc_client import RovRpcClient, RovRpcError


PROPELLERS = (
    'left_up', 'right_up', 'left_middle',
    'right_middle', 'left_down', 'right_down',
)

# Canonical mixer signs for a positive command. Zero means that axis must not
# affect the corresponding propeller. Negative commands invert every sign.
POSITIVE_PATTERNS = {
    'sway': {
        'left_up': 1, 'right_up': -1, 'left_middle': 0,
        'right_middle': 0, 'left_down': -1, 'right_down': 1,
    },
    'surge': {
        'left_up': 1, 'right_up': 1, 'left_middle': 0,
        'right_middle': 0, 'left_down': 1, 'right_down': 1,
    },
    'heave': {
        'left_up': 0, 'right_up': 0, 'left_middle': 1,
        'right_middle': 1, 'left_down': 0, 'right_down': 0,
    },
    'yaw': {
        'left_up': 1, 'right_up': -1, 'left_middle': 0,
        'right_middle': 0, 'left_down': 1, 'right_down': -1,
    },
}


def safe_amplitude(value: str) -> float:
    result = float(value)
    # Horizontal mixer deadband is 30/127. Keep the lower bound above it and
    # cap this bench tool at half command even if the API supports [-1, 1].
    if not math.isfinite(result) or not 0.25 <= result <= 0.50:
        raise argparse.ArgumentTypeError('amplitude must be in [0.25, 0.50]')
    return result


def positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError('value must be finite and greater than zero')
    return result


def expected_signs(axis: str, direction: int) -> dict[str, int]:
    return {
        name: sign * direction
        for name, sign in POSITIVE_PATTERNS[axis].items()
    }


def normalize_power(power: Mapping[str, object]) -> dict[str, int]:
    missing = set(PROPELLERS).difference(power)
    if missing:
        raise RuntimeError(f'propeller_power missing fields: {sorted(missing)}')
    normalized = {}
    for name in PROPELLERS:
        value = power[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f'{name} is not numeric: {value!r}')
        if not math.isfinite(float(value)):
            raise RuntimeError(f'{name} is not finite: {value!r}')
        normalized[name] = int(value)
    return normalized


def validate_pattern(axis: str, direction: int, power: Mapping[str, int]) -> None:
    mismatches = []
    for name, expected in expected_signs(axis, direction).items():
        actual = power[name]
        actual_sign = 1 if actual > 0 else -1 if actual < 0 else 0
        if actual_sign != expected:
            mismatches.append(f'{name}: expected sign {expected:+d}, got {actual:+d}')
    if mismatches:
        raise RuntimeError(
            f'{axis} {direction:+d} mixer mismatch: ' + '; '.join(mismatches))


def format_power(power: Mapping[str, int]) -> str:
    fields = []
    for name in PROPELLERS:
        offset = power[name]
        pulse_us = 1500 + offset
        ticks = int((pulse_us / 1_000_000.0) * 50.0 * 4096.0)
        fields.append(f'{name}={offset:+d} ({pulse_us}us/{ticks}tick)')
    return ', '.join(fields)


def stop_and_verify(client: RovRpcClient, settle: float) -> dict[str, int]:
    for _ in range(3):
        client.stop()
        time.sleep(0.05)
    time.sleep(settle)
    power = normalize_power(client.get_propeller_power())
    nonzero = {name: value for name, value in power.items() if value != 0}
    if nonzero:
        raise RuntimeError(f'final zero verification failed: {nonzero}')
    return power


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Sweep sway/surge/heave/yaw in both directions, read the six '
            'auvmaster mixer outputs, and return to zero after every phase.'
        ),
    )
    parser.add_argument('--host', default='192.168.137.219')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--axes', nargs='+', choices=tuple(POSITIVE_PATTERNS),
                        default=list(POSITIVE_PATTERNS))
    parser.add_argument('--amplitude', type=safe_amplitude, default=0.30)
    parser.add_argument('--duration', type=positive, default=0.60)
    parser.add_argument('--rate', type=positive, default=20.0)
    parser.add_argument('--settle', type=positive, default=0.12)
    parser.add_argument(
        '--observe-only', action='store_true',
        help='print values without enforcing the canonical mixer sign pattern',
    )
    parser.add_argument(
        '--allow-motion', action='store_true',
        help='required because this test deliberately emits non-zero PWM targets',
    )
    return parser


def main(args=None) -> int:
    options = build_parser().parse_args(args)
    if not options.allow_motion:
        raise SystemExit(
            'refusing PWM sweep: disconnect all thrusters, then add --allow-motion')

    client = RovRpcClient(options.host, options.port, timeout=0.5)
    period = 1.0 / options.rate
    return_code = 0
    try:
        status = client.status()
        health = client.get_pid_runtime_status()
        print(f'Connected to {options.host}:{options.port}; status={dict(status)}')
        print(
            'Sensor health: '
            f'depth={health.get("depth_position", {}).get("sensor_valid")}, '
            f'yaw={health.get("yaw_angle", {}).get("sensor_valid")}')
        print('Starting in 3 seconds. Press Ctrl-C at any time to send zero.')
        time.sleep(3.0)

        for axis in options.axes:
            for direction in (1, -1):
                value = direction * options.amplitude
                command = {
                    'sway': 0.0, 'surge': 0.0,
                    'heave': 0.0, 'yaw': 0.0,
                }
                command[axis] = value
                print(f'\nTEST {axis} {value:+.2f}')
                deadline = time.monotonic() + options.duration
                next_heartbeat = 0.0
                while time.monotonic() < deadline:
                    started = time.monotonic()
                    if started >= next_heartbeat:
                        client.status()
                        next_heartbeat = started + 0.4
                    client.move(
                        command['sway'], command['surge'],
                        command['heave'], command['yaw'])
                    remaining = period - (time.monotonic() - started)
                    if remaining > 0.0:
                        time.sleep(remaining)

                time.sleep(options.settle)
                power = normalize_power(client.get_propeller_power())
                print(format_power(power))
                if not options.observe_only:
                    validate_pattern(axis, direction, power)
                    print('PASS mixer sign pattern')
                zero = stop_and_verify(client, options.settle)
                print(f'PASS zero: {format_power(zero)}')
    except KeyboardInterrupt:
        print('\nInterrupted by operator.')
    except (RovRpcError, RuntimeError) as exc:
        print(f'FAIL: {exc}')
        return_code = 1
    finally:
        try:
            zero = stop_and_verify(client, options.settle)
            print(f'FINAL ZERO VERIFIED: {format_power(zero)}')
        except (RovRpcError, RuntimeError) as exc:
            print(f'FINAL ZERO FAILED: {exc}')
            return_code = 1
        client.close()
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
