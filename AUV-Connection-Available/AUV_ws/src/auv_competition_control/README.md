# AUV competition control

Target-relative state machine and visual servo for the ball-hit and gate-pass
tasks. The current architecture uses a state-aware dual-model perception node:

- ball states run the red/blue/gate detector and back-project the selected ball
  with registered ZED depth;
- gate states run the four-corner gate YOLO-Pose model;
- timed hit, retreat and pass states run no neural-network inference.

The controller still drives `rov_thruster_bridge` through the unchanged
`/auv/control_command`, `/auv/motion_enable`, and `/auv/target_depth_m` topics.
The current robot configuration uses `vertical_control_mode: visual_heave`:
image-space vertical error is sent in `Wrench.force.z`, while no absolute
depth target is published.

The node starts disarmed. Even after launch it will publish only zero commands
until the mission is explicitly enabled.

## Dual-model shadow launch

First start the ZED ROS wrapper and verify that rectified left image,
registered depth and camera info are present. Then run the new chain in
actuator-isolated shadow mode:

```bash
ros2 launch auv_competition_control dual_model_mission.launch.py \
  shadow_mode:=true \
  auto_start:=true \
  vertical_control_mode:=visual_heave \
  target_ball_color:=red \
  detector_model_path:=/home/auv/jetson-zed-yolo/models/competition_latest.pt \
  gate_pose_model_path:=/home/auv/jetson-zed-yolo/models/gate_pose.pt
```

If the ZED ROS Wrapper is not installed, this package can own the camera
directly through the existing ZED SDK and publish the same three input topics:

```bash
ros2 launch auv_competition_control dual_model_mission.launch.py \
  start_zed_sdk_source:=true \
  shadow_mode:=true \
  auto_start:=true \
  vertical_control_mode:=visual_heave \
  inference_device:=cpu \
  use_half_precision:=false \
  target_ball_color:=red \
  detector_model_path:=/home/auv/jetson-zed-yolo/models/competition_latest.pt \
  gate_pose_model_path:=/home/auv/jetson-zed-yolo/models/gate_pose.pt
```

`zed_sdk_source` publishes rectified left `bgr8`, registered depth `32FC1`
in metres, and rectified `CameraInfo`, all stamped from the same ZED capture.
It performs no inference and owns no actuator topic. It is the sole camera
owner: stop `detect3d.py`, recording, preview, and other ZED processes before
launching it. Replace the CPU fallback with `inference_device:=0` and
`use_half_precision:=true` only after CUDA PyTorch is working on the Jetson.

The annotated stream is
`/auv/perception/debug/image/compressed`. The selected perception contracts are
`/auv/perception/ball_observations` and
`/auv/perception/gate_pose_observations`; coordinator timing is published on
`/auv/perception/diagnostics`.

Gate PnP is disabled by default. The controller can approach using apparent
gate width alone. Enable `enable_gate_pnp` only after entering the measured
inner opening (`gate_width_m`, `gate_height_m`) and validating underwater
camera calibration. `gate_stop_width_norm` is the primary no-PnP transition to
the timed blind-pass segment and must be tuned in a restrained pool test.

The Python interpreter running this node must contain both ROS `rclpy` /
`cv_bridge` and Ultralytics/PyTorch. On Jetson Humble, use a Python 3.10 virtual
environment created with `--system-site-packages`; a Conda Python with a
different ABI cannot import Humble's `rclpy` extension.

## Unified Jetson launcher

The deployed entry point is `/home/auv/run_competition.sh`. Its default mode
is actuator-isolated shadow operation:

```bash
bash /home/auv/run_competition.sh shadow --target-ball-color red
```

With all thrusters physically disconnected, run the bounded PWM bench mode:

```bash
bash /home/auv/run_competition.sh bench \
  --allow-pwm --duration 30 --target-ball-color red
```

Real competition mode performs model, NanoPi RPC, unique topic ownership and
bridge connection checks before it arms. In `pressure_depth` mode it also
requires onboard depth hold and a live depth sample; in `visual_heave` mode
those two pressure-sensor checks are deliberately skipped. It records a rosbag
and performs controlled disarm/final-zero cleanup:

```bash
bash /home/auv/run_competition.sh live \
  --allow-motion --target-ball-color red
```

The currently deployed model hash is blocked in live mode because it produced
a documented high-confidence background false positive. Do not override that
guard for normal operation; deploy and shadow-test a corrected model first.

## Real-ZED shadow mode (no actuator path)

Shadow mode subscribes to the real `/zed/objects3d` stream and runs the same
state machine and control law, but creates publishers only below
`/auv/shadow/*`. It does not launch `rov_thruster_bridge` and therefore cannot
send commands to the NanoPi.

On the Jetson:

```bash
bash /home/auv/run_zed_shadow.sh
```

The competition model uses `red_ball`, `blue_ball`, and `gate`. Select the
required ball color when starting shadow mode:

```bash
bash /home/auv/run_zed_shadow.sh target_ball_color:=red
```

The detector reads
`/home/auv/jetson-zed-yolo/models/competition.onnx` and
`competition_classes.json`. Override them with `AUV_YOLO_MODEL` and
`AUV_YOLO_LABELS` only when deliberately testing another export.

The terminal prints state, selected XYZ, requested surge/sway/heave/yaw,
target depth and perception age. The complete JSON decision stream is
`/auv/shadow/decision`. The helper uses `ROS_DOMAIN_ID=96` so an old mock
publisher on the development computer cannot enter the shadow graph.

## ZED + Jetson + NanoPi bench test

With every thruster physically disconnected and NanoPi `auvmaster` already
running:

```bash
bash /home/auv/run_full_chain_bench.sh --allow-pwm --duration 30
```

Use `--target-ball-color blue` when the mission requires the blue ball.

To watch the annotated YOLO image, object XYZ and distance from another
computer while the test is running:

```bash
bash /home/auv/run_full_chain_bench.sh \
  --allow-pwm --duration 30 --web-preview
```

Open `http://192.168.3.19:8080` in a browser. The preview is read-only and
stops with the test. Use `--jetson-view` instead to open an OpenCV window on
the Jetson desktop. Do not use both preview modes in the same invocation.

This explicitly forwards the shadow intent through a rate-limited bridge,
reads back NanoPi's six `propeller_power` values, records a rosbag, disarms on
exit, and performs an independent final-zero check. Timed hit, retreat and
gate-pass commands are forced to zero. Translational commands are limited to
about 17%, while search yaw is about 27% so it clears NanoPi's input deadband.
Reports are saved below
`/home/auv/ros2_ws/bench_reports/`.

## Offline simulation

On the Jetson, leave the Conda environment before building or running ROS 2;
Humble uses the system Python 3.10 ABI.

```bash
conda deactivate
source /opt/ros/humble/setup.bash
colcon build --packages-select rov_thruster_bridge auv_competition_control
source install/setup.bash
ros2 launch auv_competition_control competition_mission.launch.py use_mock:=true
ros2 service call /auv/mission_enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /auv/mission_state
```

## Real hardware

Confirm the ROV address and all axis signs in `rov_thruster_bridge` first. Keep
the vehicle restrained for the first one-axis-at-a-time test.

```bash
ros2 launch auv_competition_control competition_mission.launch.py \
  target_ball_color:=red
ros2 service call /auv/mission_enable std_srvs/srv/SetBool "{data: true}"
```

Disarm at any time:

```bash
ros2 service call /auv/mission_enable std_srvs/srv/SetBool "{data: false}"
```

In the current `visual_heave` configuration, positive image-down error produces
positive `force.z` and negative image-up error produces negative `force.z`.
The command is slew-limited and returns to zero when the visual target becomes
stale. This is target-relative vertical servoing, not absolute-depth hold: the
AUV must be placed near its working depth and trimmed close to neutral buoyancy.
First verify `axis_sign.heave` with thrusters disconnected and then with the
vehicle physically restrained. Gate range from a hollow frame uses Pose image
geometry/apparent width; raw object-box depth is not considered final.
