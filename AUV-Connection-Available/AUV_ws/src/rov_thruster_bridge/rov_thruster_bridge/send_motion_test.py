#!/usr/bin/env python3
"""Safely send normalized bench-test motion commands to the NanoPi."""

from __future__ import annotations

import argparse
import math
import time

from rov_thruster_bridge.rpc_client import RovRpcClient, RovRpcError


def normalized(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not -1.0 <= result <= 1.0:
        raise argparse.ArgumentTypeError('value must be finite and in [-1.0, 1.0]')
    return result


def positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError('value must be finite and greater than zero')
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Send x=sway, y=surge, z=heave and yaw commands to '
            'auvmaster 2.x. Non-zero output requires --allow-motion.'
        ),
    )
    parser.add_argument('--host', default='192.168.137.219')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--sway', type=normalized, default=0.0)
    parser.add_argument('--surge', type=normalized, default=0.0)
    parser.add_argument('--heave', type=normalized, default=0.0)
    parser.add_argument('--yaw', type=normalized, default=0.0)
    parser.add_argument('--duration', type=positive, default=1.0)
    parser.add_argument('--rate', type=positive, default=20.0)
    parser.add_argument(
        '--allow-motion', action='store_true',
        help='required when any requested command is non-zero',
    )
    return parser


def main(args=None) -> int:
    options = build_parser().parse_args(args)
    command = (
        options.sway, options.surge, options.heave, options.yaw)
    if any(abs(value) > 1e-12 for value in command) and not options.allow_motion:
        raise SystemExit(
            'refusing non-zero command: secure the vehicle and add --allow-motion')

    client = RovRpcClient(
        host=options.host, port=options.port, timeout=0.5)
    period = 1.0 / options.rate
    deadline = time.monotonic() + options.duration
    next_heartbeat = 0.0
    sent = 0
    print(
        f'Target {options.host}:{options.port}: '
        f'x(sway)={options.sway:+.3f}, y(surge)={options.surge:+.3f}, '
        f'z(heave)={options.heave:+.3f}, yaw={options.yaw:+.3f}')
    try:
        while time.monotonic() < deadline:
            started = time.monotonic()
            if started >= next_heartbeat:
                client.status()
                next_heartbeat = started + 0.5
            ack = client.move(*command)
            sent += 1
            if ack:
                print(f'TX {sent:03d}: ACK={dict(ack)}')
            else:
                print(f'TX {sent:03d}: accepted=True, result=null')
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print('Interrupted; sending zero motion.')
    except RovRpcError as exc:
        print(f'Communication error: {exc}')
        return_code = 1
    else:
        return_code = 0
    finally:
        # Send several zeros so both a normal stop and the NanoPi watchdog are
        # exercised even if one packet is lost.
        for _ in range(3):
            try:
                ack = client.stop()
                if ack:
                    print(f'STOP: ACK={dict(ack)}')
                else:
                    print('STOP: accepted=True, result=null')
            except RovRpcError as exc:
                print(f'STOP failed: {exc}')
                return_code = 1
            time.sleep(0.05)
        client.close()
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
