#!/usr/bin/env python3
"""JSON-RPC 2.0 client for the NanoPi ``auvmaster`` 2.x HTTP server."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import socket
import threading
from typing import Any, Mapping


class RovRpcError(RuntimeError):
    """The ROV transport or JSON-RPC endpoint rejected a request."""


@dataclass(frozen=True)
class RovTelemetry:
    """Combined view of the firmware ``status`` and PID health responses."""

    depth_m: float
    yaw_deg: float
    temperature_c: float
    depth_valid: bool = True
    yaw_valid: bool = True
    depth_hold_active: bool = False
    yaw_hold_active: bool = False

    @property
    def valid(self) -> bool:
        return (
            self.depth_valid
            and self.yaw_valid
            and all(math.isfinite(value) for value in (
                self.depth_m, self.yaw_deg, self.temperature_c))
        )


_NUMBER_RE = re.compile(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)')


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{field} is boolean, expected a number')
    if isinstance(value, (int, float)):
        result = float(value)
    elif isinstance(value, str):
        match = _NUMBER_RE.search(value)
        if match is None:
            raise ValueError(f'{field} contains no number: {value!r}')
        result = float(match.group(0))
    else:
        raise ValueError(f'{field} has unsupported type {type(value).__name__}')
    if not math.isfinite(result):
        raise ValueError(f'{field} is not finite')
    return result


def parse_legacy_info(payload: Mapping[str, Any]) -> RovTelemetry:
    """Parse the unit-suffixed response produced by ``get_info``.

    Legacy firmware reports depth as a string in centimetres. Numeric depth
    values are treated as metres so new firmware can migrate without changing
    the ROS-facing API.
    """
    if not isinstance(payload, Mapping):
        raise ValueError('get_info result must be a JSON object')
    # Prefer the machine-readable fields added by the hardened firmware while
    # retaining compatibility with the original unit-suffixed Chinese fields.
    if all(name in payload for name in (
            'depth_m', 'yaw_deg', 'cpu_temperature_c')):
        depth_valid = payload.get('depth_valid', True)
        yaw_valid = payload.get('yaw_valid', True)
        if not isinstance(depth_valid, bool) or not isinstance(yaw_valid, bool):
            raise ValueError('sensor validity fields must be boolean')
        return RovTelemetry(
            depth_m=_number(payload['depth_m'], 'depth_m'),
            yaw_deg=_number(payload['yaw_deg'], 'yaw_deg'),
            temperature_c=_number(
                payload['cpu_temperature_c'], 'cpu_temperature_c'),
            depth_valid=depth_valid,
            yaw_valid=yaw_valid,
        )

    try:
        raw_depth = payload['深度']
        raw_yaw = payload['航向角']
        raw_temperature = payload['温度']
    except KeyError as exc:
        raise ValueError(f'missing telemetry field: {exc.args[0]}') from exc

    depth = _number(raw_depth, 'depth')
    if isinstance(raw_depth, str):
        lower = raw_depth.lower()
        if 'mm' in lower:
            depth /= 1000.0
        elif 'cm' in lower or '厘米' in lower:
            depth /= 100.0

    return RovTelemetry(
        depth_m=depth,
        yaw_deg=_number(raw_yaw, 'yaw'),
        temperature_c=_number(raw_temperature, 'temperature'),
    )


def _boolean(value: Any, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f'{field} must be boolean')
    return value


def parse_v2_telemetry(
    status: Mapping[str, Any],
    runtime: Mapping[str, Any],
) -> RovTelemetry:
    """Parse the two telemetry responses produced by auvmaster 2.x."""
    if not isinstance(status, Mapping):
        raise ValueError('status result must be a JSON object')
    if not isinstance(runtime, Mapping):
        raise ValueError('pid_runtime_status result must be a JSON object')
    try:
        depth_runtime = runtime['depth_position']
        yaw_runtime = runtime['yaw_angle']
    except KeyError as exc:
        raise ValueError(f'missing PID runtime group: {exc.args[0]}') from exc
    if not isinstance(depth_runtime, Mapping) or not isinstance(yaw_runtime, Mapping):
        raise ValueError('PID runtime groups must be JSON objects')
    try:
        return RovTelemetry(
            depth_m=_number(status['depth_meters'], 'depth_meters'),
            yaw_deg=_number(status['heading_degrees'], 'heading_degrees'),
            temperature_c=_number(
                status['temperature_celsius'], 'temperature_celsius'),
            depth_valid=_boolean(
                depth_runtime['sensor_valid'],
                'depth_position.sensor_valid'),
            yaw_valid=_boolean(
                yaw_runtime['sensor_valid'], 'yaw_angle.sensor_valid'),
            depth_hold_active=_boolean(
                depth_runtime['active'], 'depth_position.active'),
            yaw_hold_active=_boolean(
                yaw_runtime['active'], 'yaw_angle.active'),
        )
    except KeyError as exc:
        raise ValueError(f'missing telemetry field: {exc.args[0]}') from exc


def _recv_http_response(sock: socket.socket, max_response_bytes: int) -> dict:
    buffer = bytearray()
    header_end = -1
    content_length = None

    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buffer.extend(chunk)
        if len(buffer) > max_response_bytes:
            raise RovRpcError('ROV response exceeded configured size limit')

        if header_end < 0:
            header_end = buffer.find(b'\r\n\r\n')
            if header_end >= 0:
                header_text = bytes(buffer[:header_end]).decode('iso-8859-1')
                lines = header_text.split('\r\n')
                status_parts = lines[0].split(' ', 2)
                if len(status_parts) < 2 or status_parts[1] != '200':
                    raise RovRpcError(f'HTTP request failed: {lines[0]}')
                for line in lines[1:]:
                    name, separator, value = line.partition(':')
                    if separator and name.strip().lower() == 'content-length':
                        content_length = int(value.strip())
                        if content_length < 0 or content_length > max_response_bytes:
                            raise RovRpcError('invalid HTTP Content-Length')
                        break

        if header_end >= 0 and content_length is not None:
            body_start = header_end + 4
            if len(buffer) - body_start >= content_length:
                break

    if header_end < 0:
        raise RovRpcError('malformed HTTP response: headers are incomplete')
    body_start = header_end + 4
    if content_length is None:
        body = bytes(buffer[body_start:])
    else:
        if len(buffer) - body_start < content_length:
            raise RovRpcError('truncated HTTP response body')
        body = bytes(buffer[body_start:body_start + content_length])
    if not body:
        raise RovRpcError('empty JSON-RPC response body')
    try:
        decoded = json.loads(body.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RovRpcError(f'invalid JSON-RPC response: {exc}') from exc
    if not isinstance(decoded, dict):
        raise RovRpcError('JSON-RPC response must be an object')
    return decoded


class RovRpcClient:
    """Thread-safe, one-request-per-connection client.

    The embedded server supports HTTP keep-alive incompletely, so deliberately
    closing each request is more predictable than reusing a socket.
    """

    def __init__(
        self,
        host: str = '192.168.137.219',
        port: int = 8888,
        timeout: float = 0.25,
        max_response_bytes: int = 65536,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.max_response_bytes = int(max_response_bytes)
        self._id = 0
        self._lock = threading.Lock()

    def _call(self, method: str, params: Any = None) -> Any:
        with self._lock:
            self._id += 1
            request_id = self._id
            payload = {'jsonrpc': '2.0', 'method': method, 'id': request_id}
            if params is not None:
                payload['params'] = params
            body = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
            request = (
                f'POST / HTTP/1.1\r\n'
                f'Host: {self.host}:{self.port}\r\n'
                'Content-Type: application/json\r\n'
                'Accept: application/json\r\n'
                f'Content-Length: {len(body)}\r\n'
                'Connection: close\r\n'
                '\r\n'
            ).encode('ascii') + body
            try:
                with socket.create_connection(
                        (self.host, self.port),
                        timeout=self.timeout) as sock:
                    sock.settimeout(self.timeout)
                    sock.sendall(request)
                    response = _recv_http_response(sock, self.max_response_bytes)
            except RovRpcError:
                raise
            except (OSError, ValueError) as exc:
                raise RovRpcError(f'{method} transport failed: {exc}') from exc

        if response.get('id') != request_id:
            raise RovRpcError(
                f'{method} response id mismatch: expected {request_id}, '
                f'got {response.get("id")!r}')
        if 'error' in response:
            error = response['error'] or {}
            raise RovRpcError(
                f'{method} RPC error {error.get("code")}: {error.get("message")}')
        return response.get('result')

    def move(
        self, sway: float, surge: float, heave: float, yaw: float,
    ) -> Mapping[str, Any]:
        result = self._call('move', {
            'x': float(sway),
            'y': float(surge),
            'z': float(heave),
            'yaw': float(yaw),
        })
        # auvmaster 2.x acknowledges a valid command with a JSON null result.
        # Also accept a future acknowledgement object without another client
        # compatibility break.
        if result is None:
            return {}
        if not isinstance(result, Mapping):
            raise RovRpcError(
                f'move acknowledgement must be null or an object, got {result!r}')
        return result

    def stop(self) -> Mapping[str, Any]:
        return self.move(0.0, 0.0, 0.0, 0.0)

    def change_depth(self, delta_m: float) -> None:
        self.set_auv_target(relative_depth_m=delta_m)

    def change_heading(self, delta_deg: float) -> None:
        self.set_auv_target(relative_heading_deg=delta_deg)

    def set_auv_target(
        self,
        *,
        relative_depth_m: float | None = None,
        relative_heading_deg: float | None = None,
    ) -> None:
        params = {}
        if relative_depth_m is not None:
            params['relative_depth_meters'] = float(relative_depth_m)
        if relative_heading_deg is not None:
            params['relative_heading_degrees'] = float(relative_heading_deg)
        if not params:
            raise ValueError('at least one AUV relative target is required')
        self._call('auv_target', params)

    def status(self) -> Mapping[str, Any]:
        """Read basic sensors and renew the NanoPi heartbeat."""
        result = self._call('status')
        if not isinstance(result, Mapping):
            raise RovRpcError(f'status result must be an object, got {result!r}')
        return result

    def get_pid_runtime_status(self) -> Mapping[str, Any]:
        result = self._call('pid_runtime_status')
        if not isinstance(result, Mapping):
            raise RovRpcError(
                f'pid_runtime_status result must be an object, got {result!r}')
        return result

    def get_telemetry(self) -> RovTelemetry:
        status = self.status()
        runtime = self.get_pid_runtime_status()
        try:
            return parse_v2_telemetry(status, runtime)
        except ValueError as exc:
            raise RovRpcError(f'auvmaster telemetry parse failed: {exc}') from exc

    # Keep the historical method name for callers while using the 2.x API.
    def get_info(self) -> RovTelemetry:
        return self.get_telemetry()

    def get_propeller_power(self) -> Mapping[str, Any]:
        result = self._call('propeller_power')
        if not isinstance(result, Mapping):
            raise RovRpcError(
                f'propeller_power result must be an object, got {result!r}')
        return result

    def close(self) -> None:
        """No persistent socket is held; present for API compatibility."""
