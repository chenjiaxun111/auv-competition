# rov_thruster_bridge

Jetson-side ROS 2 bridge for the `auvmaster` 2.x HTTP/JSON-RPC server on TCP
port 8888. It sends normalized body-motion requests through `move` using the
firmware fields `x`, `y`, `z`, and `yaw`.

In AUV mode the firmware automatically holds valid depth and yaw whenever the
corresponding manual axis is zero. The Jetson parameters named
`onboard_*_hold_enabled` only permit target-changing RPC calls; they cannot
enable or disable the firmware's automatic hold behavior.

## ROS interface

All names below assume the supplied launch file and its `/auv` namespace.

Inputs:

- `/auv/control_command` (`geometry_msgs/Wrench`): body command. `force.x` is
  surge, `force.y` sway, `force.z` heave, and `torque.z` yaw. Configured limits
  map these values to the embedded controller's `[-1, 1]` range.
- `/auv/motion_enable` (`std_msgs/Bool`): explicit motion permission. The node
  starts disabled and sends only zero motion until this is `true`.
- `/auv/target_depth_m` (`std_msgs/Float32`): absolute positive-down depth in
  metres. The bridge converts it to the relative depth change required by the
  current firmware. Requests are rejected while
  `onboard_depth_hold_enabled=false`.

Outputs:

- `/auv/depth_m` (`std_msgs/Float32`)
- `/auv/yaw_deg` (`std_msgs/Float32`)
- `/auv/water_temperature_c` (`std_msgs/Float32`)
- `/auv/connected` (`std_msgs/Bool`)
- `/auv/bridge_diagnostics` (`diagnostic_msgs/DiagnosticArray`)

Network requests run in a dedicated thread; image inference or a slow RPC does
not block ROS callbacks. A stale command is replaced by zero motion after the
configured 0.30 s Jetson timeout. `status` requests also renew the auvmaster
heartbeat; the reviewed firmware uses a separate 3000 ms heartbeat timeout.
The longer embedded timeout remains the NanoPi team's safety responsibility.

## Build and run

Use the system Python that matches ROS 2, not a Conda Python with a different
minor version.

```bash
cd /home/auv/ros2_ws
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
python3 -c 'import sys, rclpy; print(sys.executable, sys.version)'
colcon build --packages-select rov_thruster_bridge
source install/setup.bash
ros2 launch rov_thruster_bridge thruster_bridge.launch.py
```

The deployed Jetson uses ROS 2 Humble and system Python 3.10. A Conda Python
with a different minor version cannot load Humble's `_rclpy` extension. Install
and invoke `colcon` from the same system-Python environment as ROS.

Override the ROV address if necessary:

```bash
ros2 launch rov_thruster_bridge thruster_bridge.launch.py \
  rov_host:=192.168.1.100 rov_port:=8888
```

## Bench sequence

Remove propellers or disconnect motor power for the first protocol test.

For the guarded four-axis RPC/PWM sweep, first run auvmaster in a NanoPi
terminal:

```bash
cd /home/pi/auvmaster-2
sudo ./build/auvmaster
```

Then run this on the Jetson. The tool tests both signs of sway, surge, heave,
and yaw at amplitude 0.30, verifies the canonical six-thruster mixer pattern,
and sends three zero commands after every phase and again on exit:

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
ros2 run rov_thruster_bridge axis_sweep_test --allow-motion
```

Test selected axes or change the bounded amplitude (`0.25` to `0.50`):

```bash
ros2 run rov_thruster_bridge axis_sweep_test \
  --axes surge yaw --amplitude 0.30 --allow-motion
```

Without `--allow-motion` the tool refuses to emit non-zero targets. Use
`--observe-only` only when intentional per-thruster calibration/reversal means
the canonical sign check no longer applies. `propeller_power` and the displayed
microseconds/ticks are commanded values, not electrical feedback; use an
oscilloscope or logic analyser for hardware waveform verification.

## Communication stress test

The deployed bridge sends `move` at 20 Hz and reads telemetry at 10 Hz. Each
telemetry cycle uses both `status` and `pid_runtime_status`, so the normal link
load is approximately 40 independent TCP/HTTP/JSON-RPC requests per second.
Run the same load for ten minutes with zero motion only:

```bash
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
ros2 run rov_thruster_bridge communication_stress_test
```

For a quick one-minute check or a longer 30-minute soak:

```bash
ros2 run rov_thruster_bridge communication_stress_test --duration 60
ros2 run rov_thruster_bridge communication_stress_test --duration 1800
```

The tool has no non-zero motion option. It records per-method success/failure,
consecutive failures, achieved rates, skipped schedule slots, mean/p50/p95/p99/
maximum latency, scheduling lateness, and final-zero verification. JSON reports
are written under `/home/auv/ros2_ws/stress_reports/` unless `--report` is used.

For ROS topic-level testing:

```bash
ros2 topic echo /auv/connected
ros2 topic echo /auv/depth_m
ros2 topic pub --once /auv/motion_enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub -r 10 /auv/control_command geometry_msgs/msg/Wrench \
  '{force: {x: 5.0, y: 0.0, z: 0.0}, torque: {z: 0.0}}'
ros2 topic pub --once /auv/target_depth_m std_msgs/msg/Float32 '{data: 0.70}'
```

Stop the command publisher and verify that the bridge reports a stale command
and sends zero within `command_timeout_s`. Then disconnect Ethernet and verify
the independent NanoPi watchdog actually drives every ESC to its neutral pulse.

## Firmware compatibility notes

This version targets auvmaster 2.x:

- `status` supplies numeric `depth_meters`, `heading_degrees`, and
  `temperature_celsius`, and acts as the heartbeat.
- `pid_runtime_status` supplies the depth/yaw `sensor_valid` and `active` flags.
- `move` accepts normalized `x`, `y`, `z`, and `yaw`; a valid request returns
  JSON `null` rather than a command-sequence object.
- `auv_target` accepts `relative_depth_meters` and
  `relative_heading_degrees`.
- `propeller_power` reports the latest six computed thrust offsets; it does not
  measure PCA9685 electrical output.

The competition stack uses positive-down depth. The reviewed firmware is
configured negative-down, so the bridge applies `sensor_sign.depth: -1.0` to
telemetry and relative depth targets.
