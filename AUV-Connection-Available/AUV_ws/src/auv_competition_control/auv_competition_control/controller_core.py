"""ROS-independent AUV mission state machine and visual-servo controller.

Ball observations remain metric 3-D targets. Gate observations are four-corner
YOLO-Pose results: alignment uses image geometry and approach may use either
PnP range or apparent gate width. Stable depth on a hollow gate is therefore
not required. This module emits body-axis intent only; the NanoPi protocol and
six-thruster mixer remain unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Optional, Tuple, Union


class MissionState(str, Enum):
    IDLE = 'IDLE'
    WAIT_CONNECTION = 'WAIT_CONNECTION'
    START_DELAY = 'START_DELAY'
    DEPARTURE_FORWARD = 'DEPARTURE_FORWARD'
    DIVE = 'DIVE'
    SEARCH_BALL = 'SEARCH_BALL'
    ALIGN_BALL = 'ALIGN_BALL'
    APPROACH_BALL = 'APPROACH_BALL'
    HIT_BALL = 'HIT_BALL'
    RETREAT = 'RETREAT'
    SEARCH_GATE = 'SEARCH_GATE'
    ALIGN_GATE = 'ALIGN_GATE'
    APPROACH_GATE = 'APPROACH_GATE'
    PASS_GATE = 'PASS_GATE'
    DONE = 'DONE'
    FAILSAFE = 'FAILSAFE'


@dataclass(frozen=True)
class BallObservation:
    """Selected red/blue ball in the ZED optical frame, in metres."""

    class_name: str
    confidence: float
    x: float
    y: float
    z: float
    received_s: float


# Compatibility name used by the former objects3d adapter and its callers.
Detection = BallObservation


@dataclass(frozen=True)
class GateKeypoint:
    """One gate corner in normalized image coordinates [0, 1]."""

    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class GatePoseObservation:
    """Full four-corner gate observation produced by YOLO-Pose.

    Corner order is TL, TR, BR, BL. ``center_*_norm`` are signed image
    errors: -1/1 are the left/right or top/bottom borders and zero is the
    optical centre. Metric PnP fields are optional because image width is also
    accepted as a proximity cue.
    """

    confidence: float
    keypoints: Tuple[GateKeypoint, GateKeypoint, GateKeypoint, GateKeypoint]
    center_x_norm: float
    center_y_norm: float
    width_norm: float
    height_norm: float
    bearing_x_rad: float
    received_s: float
    geometry_valid: bool = True
    range_m: Optional[float] = None
    translation_x_m: Optional[float] = None
    translation_y_m: Optional[float] = None
    reprojection_error_px: Optional[float] = None

    @property
    def minimum_keypoint_confidence(self) -> float:
        return min(point.confidence for point in self.keypoints)


SelectedObservation = Union[BallObservation, GatePoseObservation]


@dataclass(frozen=True)
class ControlOutput:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0
    target_depth_m: Optional[float] = None
    motion_enable: bool = False
    state: MissionState = MissionState.IDLE
    selected_target: Optional[SelectedObservation] = None


@dataclass
class PlannerConfig:
    # Mission/classes. This event has one gate; override if the rules change.
    target_ball_color: str = 'red'
    red_ball_class: str = 'red_ball'
    blue_ball_class: str = 'blue_ball'
    gate_class: str = 'gate'
    required_gate_count: int = 1
    require_bridge_connection: bool = True

    # Disabled in generic/simulation defaults; competition YAML opts in.
    # Timers are monotonic state timers, never blocking sleeps in ROS callbacks.
    startup_delay_s: float = 0.0
    departure_duration_s: float = 0.0
    departure_surge: float = 20.0

    # Vertical control. ``pressure_depth`` publishes an absolute depth target.
    # ``visual_heave`` has no absolute-depth observable: it skips DIVE and
    # directly commands heave only while a fresh ball/gate target is visible.
    vertical_control_mode: str = 'pressure_depth'

    # Pressure-depth hold and initial dive.
    initial_depth_m: float = 0.60
    minimum_depth_m: float = 0.15
    maximum_depth_m: float = 1.10
    depth_tolerance_m: float = 0.06
    dive_stable_s: float = 1.0

    # Observation age/debounce.
    target_timeout_s: float = 0.35
    detection_confirmations: int = 3
    minimum_confidence: float = 0.45
    gate_minimum_confidence: float = 0.50
    gate_minimum_keypoint_confidence: float = 0.50

    # Search and ball visual servo.
    search_yaw: float = 8.0
    yaw_kp: float = 28.0
    yaw_kd: float = 4.0
    maximum_yaw: float = 20.0
    sway_kp: float = 30.0
    maximum_sway: float = 25.0
    range_kp: float = 45.0
    maximum_approach_surge: float = 45.0
    alignment_yaw_rad: float = 0.08
    alignment_x_m: float = 0.12
    alignment_y_m: float = 0.10
    alignment_stable_s: float = 0.50
    visual_depth_rate: float = 0.25
    maximum_depth_rate_mps: float = 0.10

    # Target-relative visual heave. Positive optical Y means the target is
    # below the camera centre, therefore positive heave is requested. Confirm
    # the physical heave sign on a restrained vehicle before enabling motion.
    visual_heave_kp: float = 35.0
    visual_heave_kd: float = 0.0
    maximum_visual_heave: float = 40.0
    visual_heave_deadband: float = 0.04
    visual_heave_slew_rate: float = 80.0
    ball_vertical_full_scale_rad: float = 0.35
    ball_vertical_alignment_rad: float = 0.08

    # Gate Pose servo; image errors are resolution independent.
    gate_alignment_bearing_rad: float = 0.08
    gate_alignment_x_norm: float = 0.10
    gate_alignment_y_norm: float = 0.10
    gate_realign_x_norm: float = 0.22
    gate_realign_bearing_rad: float = 0.16
    gate_sway_kp: float = 24.0
    gate_visual_depth_rate: float = 0.20
    gate_no_range_surge: float = 18.0
    gate_stop_width_norm: float = 0.70
    gate_commit_width_norm: float = 0.58
    gate_loss_commit_timeout_s: float = 0.55
    gate_maximum_pnp_range_m: float = 8.0

    # Timed hit/pass segments.
    ball_hit_distance_m: float = 0.32
    hit_surge: float = 60.0
    hit_duration_s: float = 1.0
    retreat_surge: float = -30.0
    retreat_duration_s: float = 1.0
    gate_pass_trigger_m: float = 1.0
    pass_surge: float = 55.0
    pass_duration_s: float = 2.5

    @property
    def ball_class(self) -> str:
        return (
            self.red_ball_class
            if self.target_ball_color.lower() == 'red'
            else self.blue_ball_class
        )


class _PdController:
    def __init__(self, kp: float, kd: float, limit: float) -> None:
        self.kp = kp
        self.kd = kd
        self.limit = abs(limit)
        self._previous_error = 0.0
        self._previous_s: Optional[float] = None

    def reset(self) -> None:
        self._previous_error = 0.0
        self._previous_s = None

    def update(self, error: float, now_s: float) -> float:
        derivative = 0.0
        if self._previous_s is not None:
            dt = now_s - self._previous_s
            if 0.001 <= dt <= 0.5:
                derivative = (error - self._previous_error) / dt
        self._previous_error = error
        self._previous_s = now_s
        value = self.kp * error + self.kd * derivative
        return max(-self.limit, min(self.limit, value))


class CompetitionPlanner:
    """Rolling local visual servo for the ball-hit and gate-pass mission."""

    def __init__(self, config: Optional[PlannerConfig] = None) -> None:
        self.config = config or PlannerConfig()
        for name in ('startup_delay_s', 'departure_duration_s'):
            value = getattr(self.config, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and non-negative')
        if (not math.isfinite(self.config.departure_surge)
                or not 0 < self.config.departure_surge <= self.config.maximum_approach_surge):
            raise ValueError('departure_surge must be positive and <= maximum_approach_surge')
        if self.config.target_ball_color.lower() not in ('red', 'blue'):
            raise ValueError('target_ball_color must be red or blue')
        if self.config.vertical_control_mode not in (
                'pressure_depth', 'visual_heave'):
            raise ValueError(
                'vertical_control_mode must be pressure_depth or visual_heave')
        if self.config.required_gate_count < 0:
            raise ValueError('required_gate_count cannot be negative')
        if self.config.maximum_visual_heave <= 0.0:
            raise ValueError('maximum_visual_heave must be positive')
        if self.config.visual_heave_slew_rate <= 0.0:
            raise ValueError('visual_heave_slew_rate must be positive')
        if self.config.ball_vertical_full_scale_rad <= 0.0:
            raise ValueError('ball_vertical_full_scale_rad must be positive')
        self.state = MissionState.IDLE
        self.armed = False
        self.connected = False
        self.current_depth_m: Optional[float] = None
        self.target_depth_m = self.config.initial_depth_m
        self.gates_passed = 0
        self._startup_pending = False
        self._departure_interrupted = False
        self._ball: Optional[BallObservation] = None
        self._gate: Optional[GatePoseObservation] = None
        self._ball_confirmations = 0
        self._gate_confirmations = 0
        self._state_entered_s = 0.0
        self._stable_since_s: Optional[float] = None
        self._last_tick_s: Optional[float] = None
        self._yaw = _PdController(
            self.config.yaw_kp,
            self.config.yaw_kd,
            self.config.maximum_yaw,
        )
        self._heave = _PdController(
            self.config.visual_heave_kp,
            self.config.visual_heave_kd,
            self.config.maximum_visual_heave,
        )
        self._previous_heave = 0.0
        self._previous_heave_s: Optional[float] = None

    @property
    def visual_heave_enabled(self) -> bool:
        return self.config.vertical_control_mode == 'visual_heave'

    def _initial_motion_state(self) -> MissionState:
        if self._startup_pending:
            return MissionState.START_DELAY
        # With no pressure/depth sensor there is no absolute-depth feedback.
        # The vehicle must be placed near its working depth and starts by
        # searching; visual heave becomes active only after target acquisition.
        return (
            MissionState.SEARCH_BALL
            if self.visual_heave_enabled
            else MissionState.DIVE
        )

    def set_connected(self, connected: bool, now_s: float) -> None:
        self.connected = connected
        if not connected and self.config.require_bridge_connection and self.armed:
            self._connection_lost(now_s)

    def _connection_lost(self, now_s: float) -> None:
        if self.state == MissionState.DEPARTURE_FORWARD:
            self._departure_interrupted = True
        self._enter(
            MissionState.FAILSAFE if self._departure_interrupted
            else MissionState.WAIT_CONNECTION, now_s)

    def set_depth(self, depth_m: float) -> None:
        if math.isfinite(depth_m):
            self.current_depth_m = depth_m

    def set_armed(self, armed: bool, now_s: float) -> None:
        self.armed = armed
        self._departure_interrupted = False
        self._startup_pending = armed and (
            self.config.startup_delay_s > 0 or self.config.departure_duration_s > 0)
        self._ball = self._gate = None
        self._ball_confirmations = self._gate_confirmations = 0
        self.gates_passed = 0
        self.target_depth_m = self.config.initial_depth_m
        if not armed:
            self._enter(MissionState.IDLE, now_s)
        elif self.config.require_bridge_connection and not self.connected:
            self._enter(MissionState.WAIT_CONNECTION, now_s)
        else:
            self._enter(self._initial_motion_state(), now_s)

    def update_ball_observations(
        self, observations: Iterable[BallObservation], now_s: float,
    ) -> None:
        """Select one valid target-colour ball from the current camera frame."""
        valid = []
        for item in observations:
            if item.class_name != self.config.ball_class:
                continue
            if item.confidence < self.config.minimum_confidence:
                continue
            if not all(math.isfinite(v) for v in (item.x, item.y, item.z)):
                continue
            if item.z <= 0.0:
                continue
            valid.append(item)
        if valid:
            self._ball = max(valid, key=lambda item: (item.confidence, -item.z))
            self._ball_confirmations += 1
        else:
            self._ball_confirmations = 0

    def update_gate_observations(
        self, observations: Iterable[GatePoseObservation], now_s: float,
    ) -> None:
        """Select a geometrically valid full gate Pose from this camera frame."""
        valid = []
        for item in observations:
            values = (
                item.confidence, item.center_x_norm, item.center_y_norm,
                item.width_norm, item.height_norm, item.bearing_x_rad,
            )
            if not item.geometry_valid or not all(math.isfinite(v) for v in values):
                continue
            if item.confidence < self.config.gate_minimum_confidence:
                continue
            if (item.minimum_keypoint_confidence <
                    self.config.gate_minimum_keypoint_confidence):
                continue
            if item.width_norm <= 0.0 or item.height_norm <= 0.0:
                continue
            if item.range_m is not None and (
                not math.isfinite(item.range_m)
                or item.range_m <= 0.0
                or item.range_m > self.config.gate_maximum_pnp_range_m
            ):
                # Retain good 2-D geometry when underwater PnP is implausible.
                item = GatePoseObservation(
                    confidence=item.confidence,
                    keypoints=item.keypoints,
                    center_x_norm=item.center_x_norm,
                    center_y_norm=item.center_y_norm,
                    width_norm=item.width_norm,
                    height_norm=item.height_norm,
                    bearing_x_rad=item.bearing_x_rad,
                    received_s=item.received_s,
                    geometry_valid=item.geometry_valid,
                    range_m=None,
                    reprojection_error_px=item.reprojection_error_px,
                )
            valid.append(item)
        if valid:
            self._gate = max(valid, key=lambda item: (
                item.confidence,
                item.minimum_keypoint_confidence,
                item.width_norm * item.height_norm,
            ))
            self._gate_confirmations += 1
        else:
            self._gate_confirmations = 0

    def update_detections(
        self, detections: Iterable[Detection], now_s: float,
    ) -> None:
        """Backward-compatible input for old objects3d bench fixtures."""
        items = tuple(detections)
        self.update_ball_observations(items, now_s)
        legacy_gates = []
        for item in items:
            if item.class_name != self.config.gate_class or item.z <= 0.0:
                continue
            bearing = math.atan2(item.x, item.z)
            cx = max(-1.0, min(1.0, bearing / 0.75))
            cy = max(-1.0, min(1.0, item.y / max(item.z, 0.1)))
            width = max(0.02, min(1.0, 0.9 / item.z))
            points = (
                GateKeypoint(0.5 + 0.5 * cx - 0.25 * width, 0.35 + 0.5 * cy, 1.0),
                GateKeypoint(0.5 + 0.5 * cx + 0.25 * width, 0.35 + 0.5 * cy, 1.0),
                GateKeypoint(0.5 + 0.5 * cx + 0.25 * width, 0.65 + 0.5 * cy, 1.0),
                GateKeypoint(0.5 + 0.5 * cx - 0.25 * width, 0.65 + 0.5 * cy, 1.0),
            )
            legacy_gates.append(GatePoseObservation(
                confidence=item.confidence,
                keypoints=points,
                center_x_norm=cx,
                center_y_norm=cy,
                width_norm=width,
                height_norm=max(0.02, width * 0.75),
                bearing_x_rad=bearing,
                range_m=item.z,
                translation_x_m=item.x,
                translation_y_m=item.y,
                received_s=item.received_s,
            ))
        self.update_gate_observations(legacy_gates, now_s)

    def _enter(self, state: MissionState, now_s: float) -> None:
        self.state = state
        self._state_entered_s = now_s
        self._stable_since_s = None
        self._yaw.reset()
        self._heave.reset()
        self._previous_heave = 0.0
        self._previous_heave_s = None
        if state == MissionState.SEARCH_BALL:
            self._ball_confirmations = 0
            self._ball = None
        elif state == MissionState.SEARCH_GATE:
            self._gate_confirmations = 0
            self._gate = None

    def _fresh_ball(self, now_s: float) -> Optional[BallObservation]:
        if self._ball is None or now_s - self._ball.received_s > self.config.target_timeout_s:
            return None
        return self._ball

    def _fresh_gate(self, now_s: float) -> Optional[GatePoseObservation]:
        if self._gate is None or now_s - self._gate.received_s > self.config.target_timeout_s:
            return None
        return self._gate

    def _confirmed_ball(self, now_s: float) -> Optional[BallObservation]:
        if self._ball_confirmations < self.config.detection_confirmations:
            return None
        return self._fresh_ball(now_s)

    def _confirmed_gate(self, now_s: float) -> Optional[GatePoseObservation]:
        if self._gate_confirmations < self.config.detection_confirmations:
            return None
        return self._fresh_gate(now_s)

    def _integrate_target_depth(self, error: float, gain: float, now_s: float) -> None:
        dt = 0.05 if self._last_tick_s is None else now_s - self._last_tick_s
        dt = max(0.0, min(0.2, dt))
        requested = gain * error * dt
        maximum_step = self.config.maximum_depth_rate_mps * dt
        step = max(-maximum_step, min(maximum_step, requested))
        self.target_depth_m = max(
            self.config.minimum_depth_m,
            min(self.config.maximum_depth_m, self.target_depth_m + step),
        )

    def _stable(self, aligned: bool, now_s: float) -> bool:
        if not aligned:
            self._stable_since_s = None
            return False
        if self._stable_since_s is None:
            self._stable_since_s = now_s
        return now_s - self._stable_since_s >= self.config.alignment_stable_s

    def _ball_aligned(self, target: BallObservation, now_s: float) -> bool:
        yaw_error = math.atan2(target.x, target.z)
        vertical_aligned = (
            abs(math.atan2(target.y, target.z))
            <= self.config.ball_vertical_alignment_rad
            if self.visual_heave_enabled
            else abs(target.y) <= self.config.alignment_y_m
        )
        return self._stable(
            abs(yaw_error) <= self.config.alignment_yaw_rad
            and abs(target.x) <= self.config.alignment_x_m
            and vertical_aligned,
            now_s,
        )

    def _visual_heave(self, error: float, now_s: float) -> float:
        """Return a bounded, slew-limited target-relative heave command."""
        if not math.isfinite(error):
            return 0.0
        if abs(error) <= self.config.visual_heave_deadband:
            error = 0.0
        requested = self._heave.update(error, now_s)
        dt = (
            0.05
            if self._previous_heave_s is None
            else max(0.0, min(0.2, now_s - self._previous_heave_s))
        )
        maximum_step = self.config.visual_heave_slew_rate * dt
        command = max(
            self._previous_heave - maximum_step,
            min(self._previous_heave + maximum_step, requested),
        )
        self._previous_heave = command
        self._previous_heave_s = now_s
        return command

    def _ball_heave(self, target: BallObservation, now_s: float) -> float:
        angle = math.atan2(target.y, target.z)
        error = max(-1.0, min(
            1.0, angle / self.config.ball_vertical_full_scale_rad))
        return self._visual_heave(error, now_s)

    def _gate_heave(self, target: GatePoseObservation, now_s: float) -> float:
        return self._visual_heave(target.center_y_norm, now_s)

    def _gate_aligned(self, target: GatePoseObservation, now_s: float) -> bool:
        return self._stable(
            abs(target.bearing_x_rad) <= self.config.gate_alignment_bearing_rad
            and abs(target.center_x_norm) <= self.config.gate_alignment_x_norm
            and abs(target.center_y_norm) <= self.config.gate_alignment_y_norm,
            now_s,
        )

    def _ball_servo(
        self, target: BallObservation, now_s: float,
    ) -> tuple[float, float]:
        yaw = self._yaw.update(math.atan2(target.x, target.z), now_s)
        alignment = math.exp(
            -((target.x / 0.30) ** 2) - ((target.y / 0.25) ** 2)
        )
        surge = max(0.0, min(
            self.config.maximum_approach_surge,
            self.config.range_kp * (target.z - self.config.ball_hit_distance_m),
        )) * alignment
        return surge, yaw

    def _gate_servo(
        self, target: GatePoseObservation, now_s: float,
    ) -> tuple[float, float, float]:
        yaw = self._yaw.update(target.bearing_x_rad, now_s)
        sway = max(-self.config.maximum_sway, min(
            self.config.maximum_sway,
            self.config.gate_sway_kp * target.center_x_norm,
        ))
        alignment = math.exp(
            -((target.center_x_norm / 0.35) ** 2)
            - ((target.center_y_norm / 0.35) ** 2)
        )
        if target.range_m is not None:
            surge = max(0.0, min(
                self.config.maximum_approach_surge,
                self.config.range_kp * (
                    target.range_m - self.config.gate_pass_trigger_m),
            ))
        else:
            remaining = max(
                0.0,
                1.0 - target.width_norm / self.config.gate_stop_width_norm,
            )
            surge = self.config.gate_no_range_surge * remaining
        return surge * alignment, sway, yaw

    def _gate_should_pass(self, target: GatePoseObservation) -> bool:
        return (
            target.range_m is not None
            and target.range_m <= self.config.gate_pass_trigger_m
        ) or target.width_norm >= self.config.gate_stop_width_norm

    def _output(
        self,
        surge: float = 0.0,
        sway: float = 0.0,
        heave: float = 0.0,
        yaw: float = 0.0,
        target: Optional[SelectedObservation] = None,
    ) -> ControlOutput:
        enabled = self.armed and self.state not in (
            MissionState.IDLE,
            MissionState.WAIT_CONNECTION,
            MissionState.START_DELAY,
            MissionState.DONE,
            MissionState.FAILSAFE,
        )
        return ControlOutput(
            surge=surge,
            sway=sway,
            heave=heave,
            yaw=yaw,
            target_depth_m=(
                None if self.visual_heave_enabled or self.state in (
                    MissionState.START_DELAY, MissionState.DEPARTURE_FORWARD)
                else self.target_depth_m),
            motion_enable=enabled,
            state=self.state,
            selected_target=target,
        )

    def startup_progress(self, now_s: float) -> dict:
        """Read-only timer snapshot, using the same monotonic clock as tick()."""
        duration = {
            MissionState.START_DELAY: self.config.startup_delay_s,
            MissionState.DEPARTURE_FORWARD: self.config.departure_duration_s,
        }.get(self.state)
        remaining = None if duration is None else max(
            0.0, duration - max(0.0, now_s - self._state_entered_s))
        return dict(schema='auv.mission_progress.v1', state=self.state.value,
                    duration_s=duration, remaining_s=remaining,
                    departure_surge=self.config.departure_surge)

    def tick(self, now_s: float) -> ControlOutput:
        """Advance the state machine once; normally called at 20 Hz."""
        if not self.armed:
            return self._output()
        if self._departure_interrupted:
            # A blind timed segment must never automatically replay on reconnect.
            return self._output()
        if self.config.require_bridge_connection and not self.connected:
            if self.state != MissionState.WAIT_CONNECTION:
                self._connection_lost(now_s)
            return self._output()
        if self.state == MissionState.WAIT_CONNECTION:
            self._enter(self._initial_motion_state(), now_s)

        elapsed = now_s - self._state_entered_s
        if self.state == MissionState.START_DELAY:
            if elapsed >= self.config.startup_delay_s:
                if self.config.departure_duration_s > 0:
                    self._enter(MissionState.DEPARTURE_FORWARD, now_s)
                    output = self._output(surge=self.config.departure_surge)
                else:
                    self._startup_pending = False
                    self._enter(self._initial_motion_state(), now_s)
                    output = self._output()
            else:
                # Zero axes AND disabled motion, no search yaw or depth request.
                output = self._output()

        elif self.state == MissionState.DEPARTURE_FORWARD:
            if elapsed >= self.config.departure_duration_s:
                self._startup_pending = False
                self._enter(self._initial_motion_state(), now_s)
                output = self._output()
            else:
                output = self._output(surge=self.config.departure_surge)

        elif self.state == MissionState.DIVE:
            if self.visual_heave_enabled:
                self._enter(MissionState.SEARCH_BALL, now_s)
                output = self._output()
                self._last_tick_s = now_s
                return output
            depth_ok = (
                self.current_depth_m is not None
                and abs(self.current_depth_m - self.target_depth_m)
                <= self.config.depth_tolerance_m
            )
            if self._stable(depth_ok, now_s):
                self._enter(MissionState.SEARCH_BALL, now_s)
            output = self._output()

        elif self.state == MissionState.SEARCH_BALL:
            target = self._confirmed_ball(now_s)
            if target is None:
                output = self._output(yaw=self.config.search_yaw)
            else:
                self._enter(MissionState.ALIGN_BALL, now_s)
                output = self._output(target=target)

        elif self.state in (MissionState.ALIGN_BALL, MissionState.APPROACH_BALL):
            target = self._fresh_ball(now_s)
            if target is None:
                self._enter(MissionState.SEARCH_BALL, now_s)
                output = self._output()
            else:
                heave = 0.0
                if self.visual_heave_enabled:
                    heave = self._ball_heave(target, now_s)
                else:
                    self._integrate_target_depth(
                        target.y, self.config.visual_depth_rate, now_s)
                surge, yaw = self._ball_servo(target, now_s)
                if self.state == MissionState.ALIGN_BALL:
                    if self._ball_aligned(target, now_s):
                        self._enter(MissionState.APPROACH_BALL, now_s)
                    output = self._output(
                        heave=heave, yaw=yaw, target=target)
                elif target.z <= self.config.ball_hit_distance_m:
                    self._enter(MissionState.HIT_BALL, now_s)
                    output = self._output(target=target)
                else:
                    output = self._output(
                        surge=surge, heave=heave, yaw=yaw, target=target)

        elif self.state == MissionState.HIT_BALL:
            if elapsed >= self.config.hit_duration_s:
                self._enter(MissionState.RETREAT, now_s)
                output = self._output()
            else:
                output = self._output(surge=self.config.hit_surge)

        elif self.state == MissionState.RETREAT:
            if elapsed >= self.config.retreat_duration_s:
                self._enter(
                    MissionState.DONE
                    if self.config.required_gate_count == 0
                    else MissionState.SEARCH_GATE,
                    now_s,
                )
                output = self._output()
            else:
                output = self._output(surge=self.config.retreat_surge)

        elif self.state == MissionState.SEARCH_GATE:
            target = self._confirmed_gate(now_s)
            if target is None:
                output = self._output(yaw=self.config.search_yaw)
            else:
                self._enter(MissionState.ALIGN_GATE, now_s)
                output = self._output(target=target)

        elif self.state in (MissionState.ALIGN_GATE, MissionState.APPROACH_GATE):
            target = self._fresh_gate(now_s)
            if target is None:
                # Losing all corners near a large centred gate normally means
                # the camera reached it. Commit only from a recent valid Pose.
                can_commit = (
                    self.state == MissionState.APPROACH_GATE
                    and self._gate is not None
                    and self._gate.width_norm >= self.config.gate_commit_width_norm
                    and now_s - self._gate.received_s
                    <= self.config.gate_loss_commit_timeout_s
                )
                self._enter(
                    MissionState.PASS_GATE if can_commit
                    else MissionState.SEARCH_GATE,
                    now_s,
                )
                output = self._output()
            else:
                heave = 0.0
                if self.visual_heave_enabled:
                    heave = self._gate_heave(target, now_s)
                else:
                    self._integrate_target_depth(
                        target.center_y_norm,
                        self.config.gate_visual_depth_rate,
                        now_s,
                    )
                surge, sway, yaw = self._gate_servo(target, now_s)
                if self.state == MissionState.ALIGN_GATE:
                    if self._gate_aligned(target, now_s):
                        self._enter(MissionState.APPROACH_GATE, now_s)
                    output = self._output(
                        sway=sway, heave=heave, yaw=yaw, target=target)
                elif self._gate_should_pass(target):
                    self._enter(MissionState.PASS_GATE, now_s)
                    output = self._output(target=target)
                elif (
                    abs(target.center_x_norm) > self.config.gate_realign_x_norm
                    or abs(target.bearing_x_rad)
                    > self.config.gate_realign_bearing_rad
                ):
                    self._enter(MissionState.ALIGN_GATE, now_s)
                    output = self._output(
                        sway=sway, heave=heave, yaw=yaw, target=target)
                else:
                    output = self._output(
                        surge=surge, sway=sway, heave=heave,
                        yaw=yaw, target=target)

        elif self.state == MissionState.PASS_GATE:
            if elapsed >= self.config.pass_duration_s:
                self.gates_passed += 1
                self._enter(
                    MissionState.DONE
                    if self.gates_passed >= self.config.required_gate_count
                    else MissionState.SEARCH_GATE,
                    now_s,
                )
                output = self._output()
            else:
                output = self._output(surge=self.config.pass_surge)

        else:
            output = self._output()

        self._last_tick_s = now_s
        return output
