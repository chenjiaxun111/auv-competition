"""Actuation-free local checks for supervisor guards and zero-feedback helper."""
import subprocess
import shlex
from pathlib import Path

import pytest

from competition_zero_check import CHANNELS, verify_zero

SCRIPT = Path(__file__).with_name('auv_competition.sh')


class FakeClient:
    def __init__(self, power):
        self.power = power
        self.stops = 0

    def stop(self):
        self.stops += 1

    def get_propeller_power(self):
        return self.power


def test_verified_zero(monkeypatch):
    monkeypatch.setattr('competition_zero_check.time.sleep', lambda _: None)
    client = FakeClient(dict.fromkeys(CHANNELS, 0.0))
    assert verify_zero(client) == client.power
    assert client.stops == 3


@pytest.mark.parametrize('bad', [None, True, '0', 1.0, float('nan'), float('inf')])
def test_invalid_feedback_rejected(monkeypatch, bad):
    monkeypatch.setattr('competition_zero_check.time.sleep', lambda _: None)
    power = dict.fromkeys(CHANNELS, 0)
    power[CHANNELS[0]] = bad
    with pytest.raises(RuntimeError):
        verify_zero(FakeClient(power))


@pytest.mark.parametrize('args', [ ['start'], ['__run'],
    ['start', '--prepare-only', '--target', 'invalid'],
    ['start', '--prepare-only', '--duration', '-1'] ])
def test_guards_before_hardware(args):
    result = subprocess.run(['bash', str(SCRIPT), *args], capture_output=True)
    assert result.returncode == 2


def test_shell_syntax():
    subprocess.run(['bash', '-n', str(SCRIPT)], check=True)


def test_exit_cleanup_survives_function_return(tmp_path):
    # Reproduce the target-machine startup failure without ROS or hardware.
    definitions = SCRIPT.read_text().rsplit('\naction=', 1)[0]
    directory = shlex.quote(str(tmp_path))
    code = definitions + f'''
runtime_dir={directory}
state_file="$runtime_dir/state"
pid_file="$runtime_dir/pid"
source_ros() {{ :; }}
basic_preflight() {{ return 7; }}
send_final_zero() {{ touch "$runtime_dir/zero-submitted"; }}
AUV_SUPERVISOR_INTERNAL=1
internal_run --report-dir {directory}
'''
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True)
    assert result.returncode == 7, result.stderr
    assert '未绑定' not in result.stderr and 'unbound' not in result.stderr
    assert (tmp_path / 'zero-submitted').exists()
    assert (tmp_path / 'state').read_text().strip() == 'FAILED'
