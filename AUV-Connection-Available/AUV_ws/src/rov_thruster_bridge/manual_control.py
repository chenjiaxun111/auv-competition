#!/usr/bin/env python3
"""Interactive, normalized four-axis control for auvmaster 2.x.

This tool intentionally uses the same ``move`` RPC as autonomy. The 2.x
firmware no longer exposes direct per-thruster values or a debug-mode switch.
"""

import argparse
import json
import math
import sys
import time

from rov_thruster_bridge.rpc_client import RovRpcClient, RovRpcError


AXES = ('sway', 'surge', 'heave', 'yaw')


def normalized(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or not -1.0 <= value <= 1.0:
        raise ValueError('axis value must be finite and in [-1, 1]')
    return value


def print_help() -> None:
    print("""
Commands:
  move <sway> <surge> <heave> <yaw>  set all four normalized axes
  sway|surge|heave|yaw <value>        set one normalized axis
  zero                                set all axes to zero
  status                              read sensors and PID health
  power                               read latest six computed thrust offsets
  help                                show this text
  quit                                send zero three times and exit
""")


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Interactive auvmaster 2.x four-axis bench control')
    parser.add_argument('--host', default='192.168.137.219')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--timeout', type=float, default=0.5)
    parser.add_argument(
        '--allow-motion', action='store_true',
        help='required before any non-zero command can be sent')
    options = parser.parse_args()

    client = RovRpcClient(
        host=options.host, port=options.port, timeout=options.timeout)
    state = dict.fromkeys(AXES, 0.0)
    return_code = 0

    def send() -> None:
        if any(state.values()) and not options.allow_motion:
            raise ValueError(
                'non-zero motion refused; restart with --allow-motion')
        # ``status`` is also the embedded heartbeat.
        client.status()
        client.move(
            sway=state['sway'], surge=state['surge'],
            heave=state['heave'], yaw=state['yaw'])
        print('sent:', json.dumps(state))

    try:
        telemetry = client.get_telemetry()
        print(f'connected to {options.host}:{options.port}: {telemetry}')
        print_help()
        while True:
            parts = input('\n> ').strip().lower().split()
            if not parts:
                continue
            command = parts[0]
            if command in ('quit', 'exit', 'q'):
                break
            if command in ('help', 'h'):
                print_help()
            elif command == 'status':
                print(client.get_telemetry())
            elif command == 'power':
                print(json.dumps(client.get_propeller_power(), ensure_ascii=False))
            elif command in ('zero', 'z'):
                state.update(dict.fromkeys(AXES, 0.0))
                send()
            elif command == 'move' and len(parts) == 5:
                state.update(zip(AXES, map(normalized, parts[1:])))
                send()
            elif command in AXES and len(parts) == 2:
                state[command] = normalized(parts[1])
                send()
            else:
                print('invalid command; enter help')
    except (EOFError, KeyboardInterrupt):
        print('\ninterrupted')
    except (OSError, RovRpcError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return_code = 1
    finally:
        for _ in range(3):
            try:
                client.stop()
            except RovRpcError as exc:
                print(f'final stop failed: {exc}', file=sys.stderr)
                return_code = 1
            time.sleep(0.05)
        client.close()
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
