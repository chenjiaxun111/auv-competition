"""Map body-effort commands to the legacy normalized ROV motion axes."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionCommand:
    sway: float
    surge: float
    heave: float
    yaw: float

    @classmethod
    def zero(cls) -> 'MotionCommand':
        return cls(sway=0.0, surge=0.0, heave=0.0, yaw=0.0)


class MotionMapper:
    """Scale configured Wrench limits to the embedded ``[-1, 1]`` API."""

    def __init__(
        self,
        max_surge: float,
        max_sway: float,
        max_heave: float,
        max_yaw: float,
        surge_sign: float = 1.0,
        sway_sign: float = 1.0,
        heave_sign: float = 1.0,
        yaw_sign: float = 1.0,
    ) -> None:
        maxima = (max_surge, max_sway, max_heave, max_yaw)
        if not all(math.isfinite(value) and value > 0.0 for value in maxima):
            raise ValueError('all command maxima must be finite and greater than zero')
        self.max_surge = float(max_surge)
        self.max_sway = float(max_sway)
        self.max_heave = float(max_heave)
        self.max_yaw = float(max_yaw)
        self.surge_sign = self._sign(surge_sign)
        self.sway_sign = self._sign(sway_sign)
        self.heave_sign = self._sign(heave_sign)
        self.yaw_sign = self._sign(yaw_sign)

    @staticmethod
    def _sign(value: float) -> float:
        if value not in (-1, -1.0, 1, 1.0):
            raise ValueError('axis signs must be -1 or 1')
        return float(value)

    @staticmethod
    def _scale(value: float, maximum: float, sign: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-1.0, min(1.0, sign * float(value) / maximum))

    def map(  # noqa: A003
        self, surge: float, sway: float, heave: float, yaw: float,
    ) -> MotionCommand:
        return MotionCommand(
            sway=self._scale(sway, self.max_sway, self.sway_sign),
            surge=self._scale(surge, self.max_surge, self.surge_sign),
            heave=self._scale(heave, self.max_heave, self.heave_sign),
            yaw=self._scale(yaw, self.max_yaw, self.yaw_sign),
        )
