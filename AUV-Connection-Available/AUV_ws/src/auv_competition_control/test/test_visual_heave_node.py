import time

import rclpy

from auv_competition_control.competition_controller_node import (
    CompetitionControllerNode,
)
from auv_competition_control.controller_core import BallObservation


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_visual_heave_reaches_wrench_and_absolute_depth_is_not_published():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'shadow_mode:=true',
        '-p', 'vertical_control_mode:=visual_heave',
        '-p', 'detection_confirmations:=1',
        '-p', 'alignment_stable_s:=99.0',
        '-p', 'visual_heave_deadband:=0.0',
        '-p', 'visual_heave_kd:=0.0',
    ])
    node = CompetitionControllerNode()
    node._timer.cancel()
    command = CapturePublisher()
    depth = CapturePublisher()
    node._command_pub = command
    node._target_depth_pub = depth
    node._enable_pub = CapturePublisher()
    node._state_pub = CapturePublisher()
    node._target_pub = CapturePublisher()
    node._decision_pub = CapturePublisher()
    try:
        now_s = time.monotonic()
        node._planner.set_armed(True, now_s)
        node._planner.update_ball_observations([
            BallObservation('red_ball', 0.9, 0.0, 0.5, 2.0, now_s)
        ], now_s)
        node._control_tick()  # SEARCH_BALL -> ALIGN_BALL

        now_s = time.monotonic()
        node._planner.update_ball_observations([
            BallObservation('red_ball', 0.9, 0.0, 0.5, 2.0, now_s)
        ], now_s)
        node._control_tick()

        assert command.messages
        assert command.messages[-1].force.z > 0.0
        assert depth.messages == []
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
