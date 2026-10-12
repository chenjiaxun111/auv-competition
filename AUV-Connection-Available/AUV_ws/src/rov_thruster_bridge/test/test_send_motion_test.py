import pytest

import rov_thruster_bridge.send_motion_test as motion_test
from rov_thruster_bridge.send_motion_test import build_parser


def test_normalized_arguments_are_accepted():
    options = build_parser().parse_args([
        '--sway', '-1', '--surge', '1', '--heave', '0.25', '--yaw', '0'])
    assert options.sway == -1.0
    assert options.surge == 1.0
    assert options.heave == 0.25


def test_out_of_range_command_is_rejected():
    with pytest.raises(SystemExit):
        build_parser().parse_args(['--surge', '1.01'])


def test_non_positive_duration_is_rejected():
    with pytest.raises(SystemExit):
        build_parser().parse_args(['--duration', '0'])


def test_sender_renews_v2_heartbeat_and_stops_three_times(monkeypatch):
    instances = []

    class FakeClient:
        def __init__(self, **kwargs):
            del kwargs
            self.calls = []
            instances.append(self)

        def status(self):
            self.calls.append(('status',))
            return {}

        def move(self, *command):
            self.calls.append(('move', *command))
            return {}

        def stop(self):
            self.calls.append(('stop',))
            return {}

        def close(self):
            self.calls.append(('close',))

    monkeypatch.setattr(motion_test, 'RovRpcClient', FakeClient)
    assert motion_test.main([
        '--duration', '0.01', '--rate', '100']) == 0
    calls = instances[0].calls
    assert ('status',) in calls
    assert any(call[0] == 'move' for call in calls)
    assert calls.count(('stop',)) == 3
    assert calls[-1] == ('close',)
