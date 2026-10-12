#!/usr/bin/env python3
"""Submit only zero motion and verify firmware power feedback, not motor RPM."""
import argparse
import json
import math
import sys
import time

CHANNELS = ('left_up', 'right_up', 'left_middle', 'right_middle',
            'left_down', 'right_down')


def verify_zero(client):
    for _ in range(3):
        client.stop()
        time.sleep(0.1)
    power = client.get_propeller_power()
    for channel in CHANNELS:
        value = power.get(channel)
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value) or value != 0):
            raise RuntimeError(f'Nonzero/invalid power feedback: {power!r}')
    return power


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.137.219')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--client-root', default='/home/auv/ros2_ws/src/rov_thruster_bridge')
    args = parser.parse_args()
    sys.path.insert(0, args.client_root)
    from rov_thruster_bridge.rpc_client import RovRpcClient
    client = RovRpcClient(args.host, args.port, timeout=1.0)
    try:
        print(json.dumps(verify_zero(client)))
        print('PASS: zero accepted; six firmware power values are zero. Not measured RPM.')
    finally:
        client.close()


if __name__ == '__main__':
    main()
