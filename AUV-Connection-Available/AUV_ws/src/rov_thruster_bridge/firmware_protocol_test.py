#!/usr/bin/env python3
"""Bench protocol test for NanoPi auvmaster 2.x."""

import argparse
import json
import socket
import time


def recv_response(sock):
    data = bytearray()
    while b'\r\n\r\n' not in data:
        chunk = sock.recv(4096)
        if not chunk:
            raise RuntimeError('connection closed before HTTP headers')
        data.extend(chunk)
    header, body = data.split(b'\r\n\r\n', 1)
    content_length = None
    for line in header.split(b'\r\n')[1:]:
        name, separator, value = line.partition(b':')
        if separator and name.strip().lower() == b'content-length':
            content_length = int(value.strip())
    if content_length is None:
        raise RuntimeError('response has no Content-Length')
    while len(body) < content_length:
        chunk = sock.recv(4096)
        if not chunk:
            raise RuntimeError('truncated HTTP response')
        body += chunk
    return json.loads(body[:content_length].decode('utf-8'))


def call_fragmented(host, port, method, params, request_id, fragments):
    payload = {'jsonrpc': '2.0', 'method': method, 'id': request_id}
    if params is not None:
        payload['params'] = params
    body = json.dumps(payload, separators=(',', ':')).encode('utf-8')
    header = (
        f'POST / HTTP/1.1\r\nHost: {host}:{port}\r\n'
        'Content-Type: application/json\r\nConnection: close\r\n'
        f'Content-Length: {len(body)}\r\n\r\n'
    ).encode('ascii')
    pieces = [header]
    start = 0
    for size in fragments:
        pieces.append(body[start:start + size])
        start += size
    pieces.append(body[start:])
    with socket.create_connection((host, port), timeout=1.0) as sock:
        sock.settimeout(1.0)
        for piece in pieces:
            if piece:
                sock.sendall(piece)
                time.sleep(0.02)
        response = recv_response(sock)
    if response.get('id') != request_id:
        raise RuntimeError(f'ID mismatch: {response!r}')
    if 'error' in response:
        raise RuntimeError(f'RPC error: {response["error"]!r}')
    return response.get('result')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='192.168.137.219')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument(
        '--allow-motion', action='store_true',
        help='send a short surge command; otherwise test with zero motion only')
    args = parser.parse_args()

    status = call_fragmented(
        args.host, args.port, 'status', None, 1, [1, 2, 3])
    required = {'temperature_celsius', 'heading_degrees', 'depth_meters'}
    missing = required.difference(status)
    if missing:
        raise RuntimeError(f'status missing fields: {sorted(missing)}')
    print('PASS fragmented status: '
          f'depth={status["depth_meters"]}, '
          f'heading={status["heading_degrees"]}')

    runtime = call_fragmented(
        args.host, args.port, 'pid_runtime_status', None, 2, [1, 3, 2])
    for group in ('depth_position', 'yaw_angle'):
        if group not in runtime or not isinstance(runtime[group], dict):
            raise RuntimeError(f'pid_runtime_status missing {group}')
        for field in ('active', 'sensor_valid'):
            if not isinstance(runtime[group].get(field), bool):
                raise RuntimeError(f'{group}.{field} must be boolean')
    print('PASS PID sensor health: '
          f'depth_valid={runtime["depth_position"]["sensor_valid"]}, '
          f'yaw_valid={runtime["yaw_angle"]["sensor_valid"]}')

    surge = 0.25 if args.allow_motion else 0.0
    result = call_fragmented(
        args.host, args.port, 'move',
        {'x': 0.0, 'y': surge, 'z': 0.0, 'yaw': 0.0}, 3,
        [1, 1, 2, 3, 5],
    )
    if result is not None:
        raise RuntimeError(f'move expected JSON null result, got {result!r}')
    print(f'PASS move: surge={surge:+.2f}, result=null')

    power = call_fragmented(
        args.host, args.port, 'propeller_power', None, 4, [2, 1, 4])
    required_power = {
        'left_up', 'right_up', 'left_middle', 'right_middle',
        'left_down', 'right_down'}
    if not isinstance(power, dict) or required_power.difference(power):
        raise RuntimeError(f'invalid propeller_power response: {power!r}')
    print(f'PASS propeller_power: {power}')

    stop_result = call_fragmented(
        args.host, args.port, 'move',
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}, 5, [1, 3, 2],
    )
    if stop_result is not None:
        raise RuntimeError(f'stop expected JSON null result, got {stop_result!r}')
    print('PASS final zero command')


if __name__ == '__main__':
    main()
