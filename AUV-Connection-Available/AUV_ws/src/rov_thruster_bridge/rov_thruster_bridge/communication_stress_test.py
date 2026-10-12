#!/usr/bin/env python3
"""Stress the Jetson-to-NanoPi RPC link at competition control rates."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

from rov_thruster_bridge.rpc_client import RovRpcClient, RovRpcError


PROPELLERS = (
    'left_up', 'right_up', 'left_middle',
    'right_middle', 'left_down', 'right_down',
)


def positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError('value must be finite and greater than zero')
    return result


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_ms(values: list[float]) -> dict[str, float | int | None]:
    return {
        'count': len(values),
        'mean_ms': statistics.fmean(values) if values else None,
        'p50_ms': percentile(values, 0.50),
        'p95_ms': percentile(values, 0.95),
        'p99_ms': percentile(values, 0.99),
        'max_ms': max(values) if values else None,
    }


class StressRunner:
    def __init__(self, options, client=None, clock=time.monotonic, sleeper=time.sleep):
        self.options = options
        self.client = client or RovRpcClient(
            options.host, options.port, timeout=options.timeout)
        self.clock = clock
        self.sleep = sleeper
        self.latencies = {'move': [], 'status': [], 'pid_runtime_status': []}
        self.errors = []
        self.success = {'move': 0, 'status': 0, 'pid_runtime_status': 0}
        self.move_starts = []
        self.move_lateness_ms = []
        self.telemetry_lateness_ms = []
        self.skipped_move_slots = 0
        self.skipped_telemetry_slots = 0
        self.consecutive_failures = 0
        self.max_consecutive_failures = 0

    def call(self, name, function):
        started = self.clock()
        try:
            result = function()
        except (RovRpcError, OSError, ValueError) as exc:
            elapsed_ms = (self.clock() - started) * 1000.0
            self.consecutive_failures += 1
            self.max_consecutive_failures = max(
                self.max_consecutive_failures, self.consecutive_failures)
            self.errors.append({
                'elapsed_s': started - self.start_time,
                'method': name,
                'latency_ms': elapsed_ms,
                'error': str(exc),
            })
            return False, None
        elapsed_ms = (self.clock() - started) * 1000.0
        self.latencies[name].append(elapsed_ms)
        self.success[name] += 1
        self.consecutive_failures = 0
        return True, result

    @staticmethod
    def advance_deadline(deadline, period, now):
        deadline += period
        skipped = 0
        if deadline < now:
            skipped = math.floor((now - deadline) / period) + 1
            deadline += skipped * period
        return deadline, skipped

    def verify_zero(self):
        try:
            self.client.status()
            power = self.client.get_propeller_power()
        except RovRpcError as exc:
            return False, f'final status/propeller_power failed: {exc}'
        missing = set(PROPELLERS).difference(power)
        if missing:
            return False, f'propeller_power missing {sorted(missing)}'
        nonzero = {
            name: power[name] for name in PROPELLERS
            if not isinstance(power[name], (int, float)) or power[name] != 0
        }
        if nonzero:
            return False, f'non-zero final propeller targets: {nonzero}'
        return True, None

    def send_final_zero(self):
        results = []
        for _ in range(3):
            started = self.clock()
            try:
                self.client.stop()
            except RovRpcError as exc:
                results.append({
                    'ok': False,
                    'latency_ms': (self.clock() - started) * 1000.0,
                    'error': str(exc),
                })
            else:
                results.append({
                    'ok': True,
                    'latency_ms': (self.clock() - started) * 1000.0,
                })
            self.sleep(0.05)
        self.sleep(0.12)
        verified, error = self.verify_zero()
        return results, verified, error

    def run(self):
        self.start_time = self.clock()
        started_utc = datetime.now(timezone.utc).isoformat()
        end_time = self.start_time + self.options.duration
        move_period = 1.0 / self.options.send_rate
        telemetry_period = 1.0 / self.options.telemetry_rate
        next_move = self.start_time
        next_telemetry = self.start_time
        next_progress = self.start_time + self.options.progress_interval
        interrupted = False

        try:
            while self.clock() < end_time:
                now = self.clock()
                if now >= next_telemetry and next_telemetry <= next_move:
                    self.telemetry_lateness_ms.append(
                        max(0.0, now - next_telemetry) * 1000.0)
                    self.call('status', self.client.status)
                    self.call(
                        'pid_runtime_status', self.client.get_pid_runtime_status)
                    next_telemetry, skipped = self.advance_deadline(
                        next_telemetry, telemetry_period, self.clock())
                    self.skipped_telemetry_slots += skipped
                elif now >= next_move:
                    self.move_lateness_ms.append(
                        max(0.0, now - next_move) * 1000.0)
                    self.move_starts.append(now)
                    self.call('move', self.client.stop)
                    next_move, skipped = self.advance_deadline(
                        next_move, move_period, self.clock())
                    self.skipped_move_slots += skipped
                else:
                    self.sleep(min(next_move, next_telemetry, end_time) - now)

                now = self.clock()
                if now >= next_progress:
                    print(
                        f'{now - self.start_time:8.1f}s  '
                        f'ok={sum(self.success.values())} errors={len(self.errors)} '
                        f'move={self.success["move"]} '
                        f'telemetry={self.success["status"]}/'
                        f'{self.success["pid_runtime_status"]}',
                        flush=True,
                    )
                    next_progress = now + self.options.progress_interval
        except KeyboardInterrupt:
            interrupted = True
            print('\nInterrupted; sending final zero commands.', flush=True)
        finally:
            test_finished = self.clock()
            final_zero, zero_verified, zero_error = self.send_final_zero()
            self.client.close()

        elapsed = test_finished - self.start_time
        total_elapsed = self.clock() - self.start_time
        intervals_ms = [
            (right - left) * 1000.0
            for left, right in zip(self.move_starts, self.move_starts[1:])
        ]
        achieved_move_rate = self.success['move'] / elapsed if elapsed else 0.0
        telemetry_success = min(
            self.success['status'], self.success['pid_runtime_status'])
        achieved_telemetry_rate = telemetry_success / elapsed if elapsed else 0.0
        pass_reasons = []
        if self.errors:
            pass_reasons.append(f'{len(self.errors)} RPC failures')
        if achieved_move_rate < self.options.send_rate * 0.95:
            pass_reasons.append(
                f'move rate {achieved_move_rate:.2f}Hz below 95% target')
        if achieved_telemetry_rate < self.options.telemetry_rate * 0.95:
            pass_reasons.append(
                f'telemetry rate {achieved_telemetry_rate:.2f}Hz below 95% target')
        if not zero_verified:
            pass_reasons.append(zero_error or 'final zero not verified')
        if any(not item['ok'] for item in final_zero):
            pass_reasons.append('one or more final zero submissions failed')

        return {
            'passed': not pass_reasons and not interrupted,
            'failure_reasons': pass_reasons,
            'interrupted': interrupted,
            'started_utc': started_utc,
            'elapsed_s': elapsed,
            'elapsed_with_final_zero_s': total_elapsed,
            'target': {
                'host': self.options.host,
                'port': self.options.port,
                'rpc_timeout_s': self.options.timeout,
                'duration_s': self.options.duration,
                'move_rate_hz': self.options.send_rate,
                'telemetry_rate_hz': self.options.telemetry_rate,
                'expected_rpc_rate_hz': (
                    self.options.send_rate + 2.0 * self.options.telemetry_rate),
                'motion_command': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            },
            'requests': {
                'total': sum(self.success.values()) + len(self.errors),
                'success': dict(self.success),
                'failures': len(self.errors),
                'max_consecutive_failures': self.max_consecutive_failures,
                'achieved_move_rate_hz': achieved_move_rate,
                'achieved_telemetry_rate_hz': achieved_telemetry_rate,
                'skipped_move_slots': self.skipped_move_slots,
                'skipped_telemetry_slots': self.skipped_telemetry_slots,
            },
            'latency': {
                name: summarize_ms(values)
                for name, values in self.latencies.items()
            },
            'scheduling': {
                'move_interval': summarize_ms(intervals_ms),
                'move_lateness': summarize_ms(self.move_lateness_ms),
                'telemetry_lateness': summarize_ms(self.telemetry_lateness_ms),
            },
            'errors': self.errors,
            'final_zero_submissions': final_zero,
            'final_zero_verified': zero_verified,
            'final_zero_error': zero_error,
        }


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            'Send zero move commands at 20 Hz and status + PID health requests '
            'at 10 Hz, matching the deployed competition bridge load.'
        ),
    )
    parser.add_argument('--host', default='192.168.137.219')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--duration', type=positive, default=600.0)
    parser.add_argument('--send-rate', type=positive, default=20.0)
    parser.add_argument('--telemetry-rate', type=positive, default=10.0)
    parser.add_argument('--timeout', type=positive, default=0.20)
    parser.add_argument('--progress-interval', type=positive, default=10.0)
    parser.add_argument('--report', type=Path)
    return parser


def default_report_path():
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    return Path.home() / 'ros2_ws' / 'stress_reports' / f'rpc-stress-{stamp}.json'


def main(args=None):
    options = build_parser().parse_args(args)
    report_path = options.report or default_report_path()
    print(
        f'Zero-motion RPC stress test: {options.host}:{options.port}, '
        f'{options.duration:.1f}s, move={options.send_rate:.1f}Hz, '
        f'telemetry={options.telemetry_rate:.1f}Hz '
        f'({options.send_rate + 2 * options.telemetry_rate:.1f} RPC/s)')
    print('This tool never sends a non-zero motion command.')
    report = StressRunner(options).run()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(f'Report: {report_path}')
    print(
        f'RESULT: {"PASS" if report["passed"] else "FAIL"}; '
        f'requests={report["requests"]["total"]}, '
        f'failures={report["requests"]["failures"]}, '
        f'move={report["requests"]["achieved_move_rate_hz"]:.2f}Hz, '
        f'telemetry={report["requests"]["achieved_telemetry_rate_hz"]:.2f}Hz, '
        f'final_zero={report["final_zero_verified"]}')
    if report['failure_reasons']:
        print('Reasons: ' + '; '.join(report['failure_reasons']))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
