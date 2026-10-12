import threading
import time

from geometry_msgs.msg import Wrench
import pytest
import rclpy
from rov_thruster_bridge.rpc_client import RovTelemetry
import rov_thruster_bridge.thruster_bridge_node as bridge_module
from std_msgs.msg import Bool, Float32


class FakeRpcClient:
    def __init__(self, host, port, timeout):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.calls = []
        self.lock = threading.Lock()

    def _record(self, name, *values):
        with self.lock:
            self.calls.append((name, *values))

    def stop(self):
        self._record('stop')

    def move(self, sway, surge, heave, yaw):
        self._record('move', sway, surge, heave, yaw)

    def get_telemetry(self):
        self._record('get_telemetry')
        # auvmaster 2.x currently reports negative-down depth. The bridge
        # converts this to the ROS positive-down convention.
        return RovTelemetry(depth_m=-0.50, yaw_deg=12.0, temperature_c=28.0)

    def change_depth(self, delta_m):
        self._record('change_depth', delta_m)

    def close(self):
        self._record('close')


def wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_motion_gate_and_absolute_depth(monkeypatch, ros_context):
    monkeypatch.setattr(bridge_module, 'RovRpcClient', FakeRpcClient)
    node = bridge_module.RovBridgeNode()
    node._onboard_depth_hold_enabled = True
    node._onboard_direction_hold_enabled = True
    rpc = node._rpc
    try:
        assert wait_until(lambda: ('get_telemetry',) in rpc.calls)

        enable = Bool()
        enable.data = True
        node._enable_callback(enable)

        command = Wrench()
        command.force.x = 50.0
        command.force.z = 25.0
        command.torque.z = 3.0
        node._command_callback(command)

        target = Float32()
        target.data = 0.70
        node._target_depth_callback(target)

        assert wait_until(
            lambda: any(call[0] == 'change_depth' for call in rpc.calls))
        assert wait_until(
            lambda: ('move', 0.0, 0.5, 0.1, 0.1) in rpc.calls)
        depth_changes = [call[1] for call in rpc.calls if call[0] == 'change_depth']
        assert depth_changes[-1] == pytest.approx(-0.20)
    finally:
        node.destroy_node()


def test_connect_disconnect_logging_does_not_raise(monkeypatch, ros_context):
    """Regression test for rclpy Humble's fixed-severity call-site rule."""
    monkeypatch.setattr(bridge_module, 'RovRpcClient', FakeRpcClient)
    node = bridge_module.RovBridgeNode()
    try:
        node._set_connected(True)
        node._set_connected(False)
        node._set_connected(True)
    finally:
        node.destroy_node()


def test_missing_sensors_keep_onboard_locks_disabled(monkeypatch, ros_context):
    monkeypatch.setattr(bridge_module, 'RovRpcClient', FakeRpcClient)
    node = bridge_module.RovBridgeNode()
    rpc = node._rpc
    try:
        enable = Bool()
        enable.data = True
        node._enable_callback(enable)
        assert wait_until(lambda: ('stop',) in rpc.calls)

        before = len([call for call in rpc.calls if call[0] == 'change_depth'])
        target = Float32()
        target.data = 0.70
        node._target_depth_callback(target)
        time.sleep(0.05)
        after = len([call for call in rpc.calls if call[0] == 'change_depth'])
        assert after == before
    finally:
        node.destroy_node()
