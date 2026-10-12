import math
import json
from competition_terminal_monitor import StatusView


def make_view():
    view = StatusView()
    view.state = 'ALIGN_GATE'
    view.state_at = 10
    view.diagnostic_at = 10
    view.diagnostic = dict(connected='true', motion_enabled='false',
                           yaw_valid='true', yaw_deg='12.34', telemetry_age_s='0.1')
    return view


def test_valid_heading_and_state():
    line = make_view().line(10.2, 'READY_DISARMED')
    assert 'ALIGN_GATE' in line and '+12.34° 有效' in line and '未解锁' in line


def test_invalid_sensor_hides_previous_heading():
    view = make_view()
    view.diagnostic['yaw_valid'] = 'false'
    assert '传感器未就绪' in view.line(10.2, 'RUNNING')
    assert '12.34' not in view.line(10.2, 'RUNNING')


def test_stale_diagnostic_hides_previous_heading():
    line = make_view().line(13, 'RUNNING')
    assert '过期' in line and '12.34' not in line and '运动=未知' in line


def test_invalid_and_stale_telemetry():
    for age in ('nan', 'inf', '-1', '2', 'not-a-number'):
        view = make_view()
        view.diagnostic['telemetry_age_s'] = age
        assert '12.34' not in view.line(10.2, 'RUNNING')
    view = make_view()
    view.diagnostic['yaw_deg'] = str(math.nan)
    assert '无效数值' in view.line(10.2, 'RUNNING')


def test_missing_diagnostics():
    assert '无数据/过期' in StatusView().line(10, 'STARTING_ROS')


def progress(view, state='START_DELAY', remaining=29.5, duration=30, now=10):
    view.state, view.state_at = state, now
    view.progress_received(json.dumps(dict(
        schema='auv.mission_progress.v1', state=state, remaining_s=remaining,
        duration_s=duration, departure_surge=20)), now)


def test_countdown_and_forward_status():
    view = make_view()
    progress(view)
    assert '出发等待：剩余 30 秒（设定 30 秒）' in view.line(10.1, 'RUNNING')
    progress(view, 'DEPARTURE_FORWARD', 3.2, 5)
    text = view.line(10.1, 'RUNNING')
    assert '直行中：剩余 4 秒' in text and 'surge=20' in text
    assert '非实测速度' in text


def test_countdown_stale_state_mismatch_and_completed_state():
    view = make_view()
    progress(view)
    assert '倒计时未收到/过期' in view.startup_line(10.6)
    view.state = 'DEPARTURE_FORWARD'
    assert '未收到/过期' in view.startup_line(10.1)
    view.state = 'SEARCH_BALL'
    assert view.startup_line(10.1) == ''
    progress(view)
    assert view.startup_line(13) == ''


def test_countdown_invalid_data_clears_old_value():
    view = make_view()
    for value in (-1, 31, math.nan, True, None, '30'):
        progress(view, remaining=value)
        assert '无效' in view.startup_line(10.1)
    progress(view)
    view.progress_received('bad json', 10)
    assert '未收到/过期' in view.startup_line(10.1)


def test_countdown_uses_custom_duration_and_reset():
    view = make_view()
    progress(view, remaining=0.4, duration=1.5)
    assert '剩余 1 秒（设定 1.5 秒）' in view.startup_line(10.1)
    progress(view, remaining=15, duration=15, now=11)
    assert '剩余 15 秒' in view.startup_line(11.1)


def feed(view, kind, items, now=10):
    view.observations_received(kind, json.dumps(dict(
        schema=f'auv.{"ball" if kind == "ball" else "gate_pose"}_observations.v1',
        frame_id='zed_left_camera_optical_frame', observations=items)), now)


def test_ball_distances():
    view = make_view()
    view.state = 'ALIGN_BALL'
    feed(view, 'ball', [dict(class_name='red_ball', confidence=0.9,
                             position_m=[0.3, 0.4, 1.2])])
    line = view.target_line(10.1)
    assert '前向距离=1.200m' in line and '直线距离=1.300m' in line
    assert 'ZED双目' in line and 'zed_left_camera_optical_frame' in line


def test_targets_clear_on_empty_and_stale_frames():
    view = make_view()
    view.state = 'ALIGN_BALL'
    feed(view, 'ball', [dict(confidence=0.9, position_m=[0, 0, 1])])
    assert '过期' in view.target_line(10.4)
    feed(view, 'ball', [])
    assert '未检测' in view.target_line(10.1)
    view.observations_received('ball', 'bad json', 10)
    assert '未检测' in view.target_line(10.1)


def test_invalid_positions_rejected():
    view = make_view()
    view.state = 'APPROACH_BALL'
    for position in ([0, 0, 0], [0, 0, -1], [0, 0, math.nan], [True, 0, 1], [0, 1]):
        feed(view, 'ball', [dict(confidence=0.9, position_m=position)])
        assert '未检测' in view.target_line(10.1)


def test_gate_two_dimensional_then_pnp():
    view = make_view()
    gate = dict(confidence=0.9, geometry_valid=True, center_error_norm=[0.1, -0.2],
                size_norm=[0.4, 0.3], translation_m=None)
    feed(view, 'gate', [gate])
    assert 'Pose二维' in view.target_line(10.1)
    assert '距离不可用' in view.target_line(10.1)
    gate['translation_m'] = [0.3, 0.4, 1.2]
    feed(view, 'gate', [gate])
    assert '门框PnP估计' in view.target_line(10.1)
    assert '直线距离=1.300m' in view.target_line(10.1)


def test_old_ball_hidden_in_gate_and_timed_states():
    view = make_view()
    feed(view, 'ball', [dict(confidence=0.9, position_m=[0, 0, 1])])
    assert '未收到' in view.target_line(10.1)
    view.state = 'HIT_BALL'
    assert '不进行视觉伺服' in view.target_line(10.1)
