#!/usr/bin/env python3
"""Convert a ROS 2 Wrench to six ROV thruster commands.

Thruster Mapper converts surge, heave, and yaw to 6-channel ROV
thruster PWM values.

Physical layout (6-thruster ROV):
  - 4 horizontal thrusters at corners: front_left, front_right, back_left, back_right
  - 2 vertical thrusters in middle: center_left, center_right

The mapping is symmetric by default; each channel has independent scale,
reverse flag, and clamp limits, configurable via thruster_params.yaml.
"""


class ThrusterMapper:
    THRUSTER_NAMES = [
        'back_left', 'back_right',
        'center_left', 'center_right',
        'front_left', 'front_right',
    ]

    def __init__(self, config: dict):
        self.surge_scale = config.get('surge_scale', 5.0)
        self.yaw_scale = config.get('yaw_scale', 15.0)
        self.heave_scale = config.get('heave_scale', 2.0)
        self.max_pwm = config.get('max_pwm', 500)
        self.min_pwm = config.get('min_pwm', -500)
        self.reverse = config.get('reverse', {})
        self.channel_scale = config.get('channel_scale', {})

    def map(  # noqa: A003
        self, surge: float, heave: float, yaw: float, sway: float = 0.0,
    ) -> dict:
        # Horizontal thrusters: surge contributes forward thrust, yaw contributes rotation
        bL = self.surge_scale * surge + self.yaw_scale * yaw
        bR = self.surge_scale * surge - self.yaw_scale * yaw
        fL = self.surge_scale * surge + self.yaw_scale * yaw
        fR = self.surge_scale * surge - self.yaw_scale * yaw

        # Vertical thrusters: heave only
        cL = self.heave_scale * heave
        cR = self.heave_scale * heave

        values = {
            'back_left': bL, 'back_right': bR,
            'center_left': cL, 'center_right': cR,
            'front_left': fL, 'front_right': fR,
        }

        # Apply per-channel scale
        for name, s in self.channel_scale.items():
            if name in values:
                values[name] *= s

        # Apply reverse
        for name, rev in self.reverse.items():
            if rev and name in values:
                values[name] *= -1

        # Clamp and cast to int
        for name in values:
            values[name] = int(max(self.min_pwm, min(self.max_pwm, values[name])))

        return values

    def zero_command(self) -> dict:
        return {name: 0 for name in self.THRUSTER_NAMES}
