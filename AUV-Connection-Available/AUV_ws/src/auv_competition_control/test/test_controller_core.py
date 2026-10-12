from auv_competition_control.controller_core import (
    CompetitionPlanner,
    Detection,
    GateKeypoint,
    GatePoseObservation,
    MissionState,
    PlannerConfig,
)
import pytest


def departure_planner(require_connection=False):
    planner = CompetitionPlanner(PlannerConfig(
        vertical_control_mode='visual_heave', require_bridge_connection=require_connection,
        startup_delay_s=30.0, departure_duration_s=5.0, departure_surge=20.0))
    planner.set_connected(True, 0.0)
    planner.set_armed(True, 0.0)
    return planner


def test_departure_wait_30_then_forward_5_then_search():
    planner = departure_planner()
    for now in (0.0, 1.0, 29.999):
        output = planner.tick(now)
        assert output.state == MissionState.START_DELAY
        assert not output.motion_enable
        assert (output.surge, output.sway, output.heave, output.yaw) == (0, 0, 0, 0)
        assert output.target_depth_m is None
    for now in (30.0, 31.0, 34.999):
        output = planner.tick(now)
        assert output.state == MissionState.DEPARTURE_FORWARD
        assert output.motion_enable and output.surge == 20.0
        assert (output.sway, output.heave, output.yaw) == (0, 0, 0)
    output = planner.tick(35.0)
    assert output.state == MissionState.SEARCH_BALL and output.surge == 0
    assert planner.tick(35.05).yaw == planner.config.search_yaw


def test_disarm_during_departure_cancels_motion():
    planner = departure_planner()
    planner.tick(30.0)
    planner.set_armed(False, 31.0)
    output = planner.tick(50.0)
    assert output.state == MissionState.IDLE and not output.motion_enable
    assert output.surge == 0


def test_startup_progress_uses_state_timer_and_clears_after_finish():
    planner = departure_planner(True)
    assert planner.startup_progress(0)['remaining_s'] == 30
    planner.tick(12.5)
    assert planner.startup_progress(12.5)['remaining_s'] == 17.5
    planner.set_connected(False, 13)
    assert planner.startup_progress(13)['remaining_s'] is None
    planner.set_connected(True, 20)
    planner.tick(20)
    assert planner.startup_progress(20)['remaining_s'] == 30
    planner.tick(50)
    assert planner.startup_progress(50)['remaining_s'] == 5
    assert planner.startup_progress(52)['remaining_s'] == 3
    planner.tick(55)
    assert planner.startup_progress(55)['remaining_s'] is None


def test_fractional_startup_progress_and_disarm():
    planner = CompetitionPlanner(PlannerConfig(
        startup_delay_s=1.5, departure_duration_s=0.5,
        vertical_control_mode='visual_heave', require_bridge_connection=False))
    planner.set_armed(True, 100)
    planner.tick(101)
    assert planner.startup_progress(101)['remaining_s'] == 0.5
    planner.tick(101.5)
    assert planner.startup_progress(101.5)['duration_s'] == 0.5
    planner.set_armed(False, 101.6)
    assert planner.startup_progress(101.6)['remaining_s'] is None


def test_visible_ball_does_not_skip_departure_sequence():
    planner = departure_planner()
    planner.update_detections([detection('red_ball', 1.0, z=0.2)], 1.0)
    assert planner.tick(1.0).state == MissionState.START_DELAY
    assert planner.tick(30.0).state == MissionState.DEPARTURE_FORWARD


def test_completed_departure_does_not_repeat_after_reconnect():
    planner = departure_planner(True)
    planner.tick(30.0)
    planner.tick(35.0)
    planner.set_connected(False, 36.0)
    assert not planner.tick(36.1).motion_enable
    planner.set_connected(True, 40.0)
    assert planner.tick(40.0).state == MissionState.SEARCH_BALL


def test_departure_connection_loss_latches_failsafe():
    planner = departure_planner(True)
    planner.tick(30.0)
    planner.set_connected(False, 31.0)
    output = planner.tick(31.1)
    assert output.state == MissionState.FAILSAFE and not output.motion_enable
    planner.set_connected(True, 40.0)
    output = planner.tick(100.0)
    assert output.state == MissionState.FAILSAFE and output.surge == 0
    planner.set_armed(True, 101.0)  # deliberate new start
    assert planner.tick(101.1).state == MissionState.START_DELAY


def test_wait_restarts_full_delay_after_connection_returns():
    planner = departure_planner(True)
    planner.set_connected(False, 10.0)
    assert not planner.tick(40.0).motion_enable
    planner.set_connected(True, 41.0)
    assert planner.tick(41.0).state == MissionState.START_DELAY
    assert planner.tick(70.999).surge == 0
    assert planner.tick(71.0).state == MissionState.DEPARTURE_FORWARD


@pytest.mark.parametrize('parameter,value', [
    ('startup_delay_s', -1), ('startup_delay_s', float('nan')),
    ('departure_duration_s', float('inf')), ('departure_duration_s', -1),
    ('departure_surge', -1), ('departure_surge', 100),
])
def test_departure_parameters_reject_invalid_values(parameter, value):
    config = PlannerConfig(startup_delay_s=30, departure_duration_s=5)
    setattr(config, parameter, value)
    with pytest.raises(ValueError):
        CompetitionPlanner(config)


def detection(name, now, x=0.0, y=0.0, z=2.0):
    return Detection(name, 0.9, x, y, z, now)


def confirm(planner, name, now, **position):
    for index in range(planner.config.detection_confirmations):
        timestamp = now + index * 0.01
        planner.update_detections(
            [detection(name, timestamp, **position)], timestamp)
    return now + (planner.config.detection_confirmations - 1) * 0.01


def gate_pose(now, cx=0.0, cy=0.0, width=0.30, range_m=None,
              point_confidence=0.9, geometry_valid=True):
    centre_x = 0.5 + 0.5 * cx
    centre_y = 0.5 + 0.5 * cy
    height = width * 0.75
    points = (
        GateKeypoint(centre_x - width / 2, centre_y - height / 2,
                     point_confidence),
        GateKeypoint(centre_x + width / 2, centre_y - height / 2,
                     point_confidence),
        GateKeypoint(centre_x + width / 2, centre_y + height / 2,
                     point_confidence),
        GateKeypoint(centre_x - width / 2, centre_y + height / 2,
                     point_confidence),
    )
    return GatePoseObservation(
        confidence=0.9,
        keypoints=points,
        center_x_norm=cx,
        center_y_norm=cy,
        width_norm=width,
        height_norm=height,
        bearing_x_rad=cx * 0.55,
        received_s=now,
        geometry_valid=geometry_valid,
        range_m=range_m,
    )


def test_disarmed_and_disconnected_outputs_are_zero():
    planner = CompetitionPlanner()
    output = planner.tick(0.0)
    assert not output.motion_enable
    assert output.surge == output.sway == output.yaw == 0.0

    planner.set_armed(True, 1.0)
    assert planner.state == MissionState.WAIT_CONNECTION
    assert not planner.tick(1.1).motion_enable


def test_target_timeout_stops_and_returns_to_search():
    config = PlannerConfig(
        require_bridge_connection=False,
        dive_stable_s=0.0,
        detection_confirmations=1,
        alignment_stable_s=0.0,
        target_timeout_s=0.2,
    )
    planner = CompetitionPlanner(config)
    planner.set_depth(config.initial_depth_m)
    planner.set_armed(True, 0.0)
    planner.tick(0.01)
    planner.tick(0.02)
    assert planner.state == MissionState.SEARCH_BALL
    planner.update_detections([detection('red_ball', 0.03)], 0.03)
    planner.tick(0.03)
    assert planner.state == MissionState.ALIGN_BALL
    planner.tick(0.04)
    assert planner.state == MissionState.APPROACH_BALL
    output = planner.tick(0.30)
    assert planner.state == MissionState.SEARCH_BALL
    assert output.surge == output.sway == output.yaw == 0.0


def test_complete_ball_and_two_gate_mission():
    config = PlannerConfig(
        require_bridge_connection=False,
        dive_stable_s=0.0,
        detection_confirmations=1,
        alignment_stable_s=0.0,
        hit_duration_s=0.1,
        retreat_duration_s=0.1,
        pass_duration_s=0.1,
        required_gate_count=2,
    )
    planner = CompetitionPlanner(config)
    planner.set_depth(config.initial_depth_m)
    planner.set_armed(True, 0.0)
    planner.tick(0.01)
    planner.tick(0.02)

    planner.update_detections([detection('red_ball', 0.03)], 0.03)
    planner.tick(0.03)
    planner.tick(0.04)
    planner.update_detections([
        detection('red_ball', 0.05, z=0.30)], 0.05)
    planner.tick(0.05)
    assert planner.state == MissionState.HIT_BALL
    planner.tick(0.16)
    planner.tick(0.27)
    assert planner.state == MissionState.SEARCH_GATE

    now = 0.30
    for expected_count in (1, 2):
        planner.update_detections([detection('gate', now)], now)
        planner.tick(now)
        planner.tick(now + 0.01)
        planner.update_detections([
            detection('gate', now + 0.02, z=0.90)], now + 0.02)
        planner.tick(now + 0.02)
        assert planner.state == MissionState.PASS_GATE
        planner.tick(now + 0.13)
        assert planner.gates_passed == expected_count
        now += 0.20

    assert planner.state == MissionState.DONE
    assert not planner.tick(now).motion_enable


def test_visual_depth_target_is_rate_limited_and_clamped():
    config = PlannerConfig(
        require_bridge_connection=False,
        dive_stable_s=0.0,
        detection_confirmations=1,
        alignment_stable_s=99.0,
        maximum_depth_rate_mps=0.10,
        visual_depth_rate=10.0,
        initial_depth_m=1.09,
        maximum_depth_m=1.10,
    )
    planner = CompetitionPlanner(config)
    planner.set_depth(config.initial_depth_m)
    planner.set_armed(True, 0.0)
    planner.tick(0.01)
    planner.tick(0.02)
    planner.update_detections([
        detection('red_ball', 0.03, y=1.0)], 0.03)
    planner.tick(0.03)
    planner.tick(0.13)
    assert planner.target_depth_m <= config.maximum_depth_m


def test_visual_heave_mode_skips_depth_dive_and_disables_depth_target():
    config = PlannerConfig(
        require_bridge_connection=False,
        vertical_control_mode='visual_heave',
    )
    planner = CompetitionPlanner(config)
    planner.set_armed(True, 0.0)

    assert planner.state == MissionState.SEARCH_BALL
    output = planner.tick(0.01)
    assert output.motion_enable
    assert output.target_depth_m is None
    assert output.heave == 0.0


def test_visual_heave_tracks_ball_vertical_angle_and_stops_on_loss():
    config = PlannerConfig(
        require_bridge_connection=False,
        vertical_control_mode='visual_heave',
        detection_confirmations=1,
        alignment_stable_s=99.0,
        target_timeout_s=0.20,
        visual_heave_deadband=0.0,
        visual_heave_kd=0.0,
        visual_heave_slew_rate=80.0,
    )
    planner = CompetitionPlanner(config)
    planner.set_armed(True, 0.0)

    planner.update_ball_observations([
        detection('red_ball', 0.01, y=0.50, z=2.0)], 0.01)
    planner.tick(0.01)  # SEARCH_BALL -> ALIGN_BALL
    below = planner.tick(0.06)
    assert below.heave > 0.0
    assert below.target_depth_m is None
    assert below.heave <= config.maximum_visual_heave

    planner.update_ball_observations([
        detection('red_ball', 0.11, y=-0.50, z=2.0)], 0.11)
    planner.tick(0.11)
    above = planner.tick(0.16)
    assert above.heave < 0.0

    lost = planner.tick(0.40)
    assert planner.state == MissionState.SEARCH_BALL
    assert lost.heave == 0.0


def test_visual_heave_tracks_gate_center_and_is_zero_during_blind_pass():
    config = PlannerConfig(
        require_bridge_connection=False,
        vertical_control_mode='visual_heave',
        detection_confirmations=1,
        alignment_stable_s=99.0,
        visual_heave_deadband=0.0,
        visual_heave_kd=0.0,
    )
    planner = CompetitionPlanner(config)
    planner.armed = True
    planner._enter(MissionState.SEARCH_GATE, 1.0)
    planner.update_gate_observations([
        gate_pose(1.01, cy=0.50)], 1.01)
    planner.tick(1.01)
    output = planner.tick(1.06)
    assert output.heave > 0.0
    assert output.target_depth_m is None

    planner._enter(MissionState.PASS_GATE, 1.10)
    blind_pass = planner.tick(1.11)
    assert blind_pass.surge == config.pass_surge
    assert blind_pass.heave == 0.0


def test_pressure_depth_mode_remains_backward_compatible():
    config = PlannerConfig(require_bridge_connection=False)
    planner = CompetitionPlanner(config)
    planner.set_armed(True, 0.0)
    output = planner.tick(0.01)
    assert planner.state == MissionState.DIVE
    assert output.heave == 0.0
    assert output.target_depth_m == config.initial_depth_m


def test_invalid_vertical_control_mode_is_rejected():
    try:
        CompetitionPlanner(PlannerConfig(vertical_control_mode='invalid'))
    except ValueError as error:
        assert 'vertical_control_mode' in str(error)
    else:
        raise AssertionError('invalid vertical mode was accepted')


def test_gate_pose_can_approach_and_pass_without_metric_range():
    config = PlannerConfig(
        require_bridge_connection=False,
        detection_confirmations=1,
        alignment_stable_s=0.0,
        gate_stop_width_norm=0.70,
    )
    planner = CompetitionPlanner(config)
    planner.armed = True
    planner._enter(MissionState.SEARCH_GATE, 1.0)
    planner.update_gate_observations([gate_pose(1.01)], 1.01)
    planner.tick(1.01)
    assert planner.state == MissionState.ALIGN_GATE
    planner.tick(1.02)
    assert planner.state == MissionState.APPROACH_GATE
    output = planner.tick(1.03)
    assert output.surge > 0.0

    planner.update_gate_observations([
        gate_pose(1.04, width=0.72)], 1.04)
    planner.tick(1.04)
    assert planner.state == MissionState.PASS_GATE


def test_gate_pose_rejects_low_corner_confidence():
    config = PlannerConfig(
        require_bridge_connection=False,
        detection_confirmations=1,
    )
    planner = CompetitionPlanner(config)
    planner.armed = True
    planner._enter(MissionState.SEARCH_GATE, 1.0)
    planner.update_gate_observations([
        gate_pose(1.01, point_confidence=0.2)], 1.01)
    output = planner.tick(1.01)
    assert planner.state == MissionState.SEARCH_GATE
    assert output.yaw == config.search_yaw


def test_gate_approach_returns_to_alignment_when_offset_grows():
    config = PlannerConfig(
        require_bridge_connection=False,
        detection_confirmations=1,
        alignment_stable_s=0.0,
    )
    planner = CompetitionPlanner(config)
    planner.armed = True
    planner._enter(MissionState.SEARCH_GATE, 1.0)
    planner.update_gate_observations([gate_pose(1.01)], 1.01)
    planner.tick(1.01)
    planner.tick(1.02)
    assert planner.state == MissionState.APPROACH_GATE
    planner.update_gate_observations([gate_pose(1.03, cx=0.4)], 1.03)
    planner.tick(1.03)
    assert planner.state == MissionState.ALIGN_GATE


def test_recent_large_gate_loss_commits_to_blind_pass():
    config = PlannerConfig(
        require_bridge_connection=False,
        detection_confirmations=1,
        alignment_stable_s=0.0,
        target_timeout_s=0.30,
        gate_loss_commit_timeout_s=0.55,
        gate_commit_width_norm=0.55,
    )
    planner = CompetitionPlanner(config)
    planner.armed = True
    planner._enter(MissionState.SEARCH_GATE, 1.0)
    planner.update_gate_observations([
        gate_pose(1.01, width=0.60)], 1.01)
    planner.tick(1.01)
    planner.tick(1.02)
    assert planner.state == MissionState.APPROACH_GATE
    planner.update_gate_observations([], 1.10)
    planner.tick(1.35)
    assert planner.state == MissionState.PASS_GATE
