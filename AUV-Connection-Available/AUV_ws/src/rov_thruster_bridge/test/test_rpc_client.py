import json
import socketserver
import threading

import pytest

from rov_thruster_bridge.rpc_client import (
    parse_legacy_info,
    parse_v2_telemetry,
    RovRpcClient,
    RovRpcError,
)


class RpcHandler(socketserver.BaseRequestHandler):
    def handle(self):
        data = bytearray()
        while b'\r\n\r\n' not in data:
            data.extend(self.request.recv(4096))
        header, body = bytes(data).split(b'\r\n\r\n', 1)
        content_length = 0
        for line in header.split(b'\r\n')[1:]:
            name, _, value = line.partition(b':')
            if name.lower() == b'content-length':
                content_length = int(value.strip())
        while len(body) < content_length:
            body += self.request.recv(4096)
        request = json.loads(body[:content_length].decode())
        self.server.requests.append(request)

        if request['method'] == 'fail':
            response = {
                'jsonrpc': '2.0',
                'error': {'code': -1, 'message': 'expected failure'},
                'id': request['id'],
            }
        elif request['method'] == 'status':
            response = {
                'jsonrpc': '2.0',
                'result': {
                    'temperature_celsius': 28.5,
                    'heading_degrees': 12.25,
                    'depth_meters': -0.705,
                },
                'id': request['id'],
            }
        elif request['method'] == 'pid_runtime_status':
            response = {
                'jsonrpc': '2.0',
                'result': {
                    'timestamp_ms': 123,
                    'depth_position': {'active': True, 'sensor_valid': True},
                    'yaw_angle': {'active': False, 'sensor_valid': False},
                },
                'id': request['id'],
            }
        else:
            response = {'jsonrpc': '2.0', 'result': None, 'id': request['id']}
        encoded = json.dumps(response, ensure_ascii=False).encode('utf-8')
        wire = (
            b'HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n'
            + f'Content-Length: {len(encoded)}\r\nConnection: close\r\n\r\n'.encode()
            + encoded
        )
        # Fragment the response to exercise header/body accumulation.
        midpoint = len(wire) // 2
        self.request.sendall(wire[:midpoint])
        self.request.sendall(wire[midpoint:])


@pytest.fixture
def rpc_server():
    server = socketserver.ThreadingTCPServer(('127.0.0.1', 0), RpcHandler)
    server.requests = []
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


def test_legacy_info_units():
    telemetry = parse_legacy_info({
        '温度': '24.5 ℃', '航向角': '-15.2 °', '深度': '83.0 cm'})
    assert telemetry.depth_m == pytest.approx(0.83)
    assert telemetry.yaw_deg == pytest.approx(-15.2)
    assert telemetry.temperature_c == pytest.approx(24.5)
    assert telemetry.valid


def test_machine_readable_info_preserves_sensor_validity():
    telemetry = parse_legacy_info({
        'cpu_temperature_c': 31.0,
        'yaw_deg': 0.0,
        'depth_m': 0.0,
        'yaw_valid': False,
        'depth_valid': False,
    })
    assert telemetry.depth_m == 0.0
    assert telemetry.depth_valid is False
    assert telemetry.yaw_valid is False
    assert telemetry.valid is False


def test_machine_readable_info_rejects_non_boolean_validity():
    with pytest.raises(ValueError, match='validity fields must be boolean'):
        parse_legacy_info({
            'cpu_temperature_c': 31.0,
            'yaw_deg': 0.0,
            'depth_m': 0.0,
            'yaw_valid': 0,
            'depth_valid': 'false',
        })


def test_v2_telemetry_combines_status_and_runtime_health():
    telemetry = parse_v2_telemetry(
        {
            'temperature_celsius': 21.5,
            'heading_degrees': -45.0,
            'depth_meters': -0.80,
        },
        {
            'depth_position': {'active': True, 'sensor_valid': True},
            'yaw_angle': {'active': False, 'sensor_valid': False},
        },
    )
    assert telemetry.depth_m == pytest.approx(-0.80)
    assert telemetry.yaw_deg == pytest.approx(-45.0)
    assert telemetry.temperature_c == pytest.approx(21.5)
    assert telemetry.depth_valid is True
    assert telemetry.yaw_valid is False
    assert telemetry.depth_hold_active is True
    assert telemetry.yaw_hold_active is False


def test_v2_telemetry_rejects_invalid_health_type():
    with pytest.raises(ValueError, match='sensor_valid must be boolean'):
        parse_v2_telemetry(
            {
                'temperature_celsius': 21.5,
                'heading_degrees': 0.0,
                'depth_meters': 0.0,
            },
            {
                'depth_position': {'active': True, 'sensor_valid': 1},
                'yaw_angle': {'active': True, 'sensor_valid': True},
            },
        )


def test_rpc_methods_and_fragmented_response(rpc_server):
    client = RovRpcClient(
        host='127.0.0.1', port=rpc_server.server_address[1], timeout=0.5)
    ack = client.move(sway=0.1, surge=0.2, heave=-0.3, yaw=0.4)
    assert ack == {}
    telemetry = client.get_telemetry()
    assert telemetry.depth_m == pytest.approx(-0.705)
    assert telemetry.depth_valid is True
    assert telemetry.yaw_valid is False
    assert rpc_server.requests[0]['method'] == 'move'
    assert rpc_server.requests[0]['params'] == {
        'x': 0.1, 'y': 0.2, 'z': -0.3, 'yaw': 0.4}
    assert rpc_server.requests[1]['method'] == 'status'
    assert rpc_server.requests[2]['method'] == 'pid_runtime_status'


def test_v2_auv_target_methods(rpc_server):
    client = RovRpcClient(
        host='127.0.0.1', port=rpc_server.server_address[1], timeout=0.5)
    client.change_depth(-0.2)
    client.change_heading(30.0)
    assert rpc_server.requests[0] == {
        'jsonrpc': '2.0', 'method': 'auv_target', 'id': 1,
        'params': {'relative_depth_meters': -0.2},
    }
    assert rpc_server.requests[1]['params'] == {
        'relative_heading_degrees': 30.0}


def test_empty_auv_target_is_rejected_before_transport(rpc_server):
    client = RovRpcClient(
        host='127.0.0.1', port=rpc_server.server_address[1], timeout=0.5)
    with pytest.raises(ValueError, match='at least one'):
        client.set_auv_target()
    assert rpc_server.requests == []


def test_rpc_error_is_raised(rpc_server):
    client = RovRpcClient(
        host='127.0.0.1', port=rpc_server.server_address[1], timeout=0.5)
    with pytest.raises(RovRpcError, match='expected failure'):
        client._call('fail')
