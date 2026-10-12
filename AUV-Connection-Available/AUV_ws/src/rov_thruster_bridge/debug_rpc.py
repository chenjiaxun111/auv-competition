#!/usr/bin/env python3
"""Raw HTTP debug tool.

Send one JSON-RPC request and print the full
HTTP request/response to diagnose -32700 parse errors.
"""
import json
import socket
import sys

HOST = sys.argv[1] if len(sys.argv) > 1 else '192.168.137.219'
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8888

# Pick a test
tests = [
    ('status', None),
    ('pid_runtime_status', None),
    ('propeller_power', None),
    ('move', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}),
]

for method, params in tests:
    payload = {'jsonrpc': '2.0', 'method': method, 'id': 1}
    if params is not None:
        payload['params'] = params

    body = json.dumps(payload, ensure_ascii=False)
    body_bytes = body.encode('utf-8')

    request = (
        f'POST / HTTP/1.1\r\n'
        f'Host: {HOST}:{PORT}\r\n'
        f'Content-Type: application/json\r\n'
        f'Content-Length: {len(body_bytes)}\r\n'
        f'Connection: close\r\n'
        f'\r\n'
        f'{body}'
    )

    print('=' * 60)
    print(f'>>> REQUEST ({method})')
    print(request)
    print('=' * 60)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((HOST, PORT))
        sock.sendall(request.encode('utf-8'))

        # Read response
        response = b''
        while True:
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                response += chunk
            except socket.timeout:
                break
        sock.close()

        print(f'<<< RESPONSE ({len(response)} bytes)')
        print(response.decode('utf-8', errors='replace'))
        print()

    except Exception as e:
        print(f'ERROR: {e}')
        print()

print('DONE')
