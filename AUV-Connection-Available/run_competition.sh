#!/usr/bin/env bash
# Unified Jetson launcher for the AUV ball-and-gate mission.
#
# The default mode is actuator-isolated shadow mode. Real competition mode is
# deliberately guarded by explicit acknowledgements and live preflight checks.

set -eo pipefail

mode='shadow'
target_ball_color='red'
duration=''
preview='true'
preview_host='192.168.3.8'
preview_port='8080'
allow_pwm='false'
allow_motion='false'
allow_unvalidated_model='false'
model='/home/auv/jetson-zed-yolo/models/competition.onnx'
labels='/home/auv/jetson-zed-yolo/models/competition_classes.json'
nanopi_host='192.168.137.219'
nanopi_port='8888'
known_bad_model_sha256='01dfb82710e3f4a03823449b3fb268947acbf8457be2b2401048a88b50776208'

usage() {
    cat <<'EOF'
Usage:
  run_competition.sh shadow [options]
  run_competition.sh bench --allow-pwm [options]
  run_competition.sh live --allow-motion [--allow-unvalidated-model] [options]

Modes:
  shadow   ZED + planner only; never starts the NanoPi bridge (default)
  bench    Bounded no-thruster PWM chain using run_full_chain_bench.sh
  live     Real competition controller and NanoPi bridge

Options:
  --target-ball-color red|blue  Ball selected for this run (default: red)
  --duration SECONDS            Bench default: 30; live default: 0 (until Ctrl-C)
  --no-preview                  Disable browser preview
  --preview-host ADDRESS        Preview address (default: 192.168.3.8)
  --preview-port PORT           Preview port (default: 8080)
  --model PATH                  Competition ONNX path
  --labels PATH                 Competition class JSON path
  --allow-pwm                   Required for bench mode
  --allow-motion                Required for live mode
  --allow-unvalidated-model     Override rejection of the known false-positive model
  -h, --help                    Show this help

Examples:
  bash /home/auv/run_competition.sh shadow --target-ball-color red
  bash /home/auv/run_competition.sh bench --allow-pwm --duration 30
  bash /home/auv/run_competition.sh live --allow-motion
EOF
}

if [[ $# -gt 0 && "$1" != -* ]]; then
    mode="$1"
    shift
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --target-ball-color)
            target_ball_color="${2:?missing target ball color}"
            shift 2
            ;;
        --duration) duration="${2:?missing duration}"; shift 2 ;;
        --no-preview) preview='false'; shift ;;
        --preview-host) preview_host="${2:?missing preview host}"; shift 2 ;;
        --preview-port) preview_port="${2:?missing preview port}"; shift 2 ;;
        --model) model="${2:?missing model path}"; shift 2 ;;
        --labels) labels="${2:?missing labels path}"; shift 2 ;;
        --allow-pwm) allow_pwm='true'; shift ;;
        --allow-motion) allow_motion='true'; shift ;;
        --allow-unvalidated-model) allow_unvalidated_model='true'; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "${mode}" != 'shadow' && "${mode}" != 'bench' && "${mode}" != 'live' ]]; then
    echo "Mode must be shadow, bench, or live; got: ${mode}" >&2
    exit 2
fi
if [[ -z "${duration}" ]]; then
    if [[ "${mode}" == 'bench' ]]; then
        duration='30'
    else
        duration='0'
    fi
fi
if [[ "${target_ball_color}" != 'red' && "${target_ball_color}" != 'blue' ]]; then
    echo 'Target ball color must be red or blue.' >&2
    exit 2
fi
if ! [[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo 'Duration must be a non-negative number.' >&2
    exit 2
fi
if ! [[ "${preview_port}" =~ ^[0-9]+$ ]] \
        || (( preview_port < 1 || preview_port > 65535 )); then
    echo 'Preview port must be an integer in [1, 65535].' >&2
    exit 2
fi

if [[ "${mode}" == 'shadow' ]]; then
    export AUV_YOLO_MODEL="${model}"
    export AUV_YOLO_LABELS="${labels}"
    echo 'Starting actuator-isolated competition shadow mode.'
    exec bash /home/auv/run_zed_shadow.sh \
        "target_ball_color:=${target_ball_color}"
fi

if [[ "${mode}" == 'bench' ]]; then
    if [[ "${allow_pwm}" != 'true' ]]; then
        echo 'REFUSED: bench mode changes NanoPi PWM; add --allow-pwm.' >&2
        exit 2
    fi
    args=(
        --allow-pwm --duration "${duration}"
        --target-ball-color "${target_ball_color}"
        --model "${model}" --labels "${labels}"
    )
    if [[ "${preview}" == 'true' ]]; then
        args+=(--web-preview --preview-host "${preview_host}" \
            --preview-port "${preview_port}")
    fi
    exec bash /home/auv/run_full_chain_bench.sh "${args[@]}"
fi

# Everything below this point is real competition mode.
if [[ "${allow_motion}" != 'true' ]]; then
    echo 'REFUSED: live mode can move the AUV; add --allow-motion.' >&2
    exit 2
fi
if [[ ! -r "${model}" || ! -r "${labels}" ]]; then
    echo "FAILED: model or labels file is not readable: ${model}, ${labels}" >&2
    exit 3
fi
model_sha256="$(sha256sum "${model}" | awk '{print $1}')"
if [[ "${model_sha256}" == "${known_bad_model_sha256}" \
        && "${allow_unvalidated_model}" != 'true' ]]; then
    echo 'REFUSED: this model is known to classify background as blue_ball.' >&2
    echo "Model SHA-256: ${model_sha256}" >&2
    echo 'Deploy a corrected model, or explicitly add --allow-unvalidated-model.' >&2
    exit 3
fi
if pgrep -af 'detect3d.py|competition.*controller|thruster_bridge_node' \
        >/dev/null; then
    echo 'REFUSED: a detector, competition controller, or bridge is running.' >&2
    pgrep -af 'detect3d.py|competition.*controller|thruster_bridge_node' >&2
    exit 4
fi
if ! timeout 2 bash -c "</dev/tcp/${nanopi_host}/${nanopi_port}"; then
    echo "FAILED: NanoPi RPC ${nanopi_host}:${nanopi_port} is not reachable." >&2
    exit 5
fi

export ROS_DOMAIN_ID=96
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
set -u

report_dir="/home/auv/ros2_ws/competition_runs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "${report_dir}"
zed_pid=''
launch_pid=''
bag_pid=''
armed='false'

cleanup() {
    trap - EXIT INT TERM
    set +e
    if [[ "${armed}" == 'true' ]]; then
        timeout 3 ros2 service call /auv/mission_enable \
            std_srvs/srv/SetBool '{data: false}' \
            >"${report_dir}/disarm.log" 2>&1
        armed='false'
        sleep 0.5
    fi
    if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        kill -INT "${bag_pid}" 2>/dev/null
        wait "${bag_pid}" 2>/dev/null
    fi
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -INT -- "-${launch_pid}" 2>/dev/null
        wait "${launch_pid}" 2>/dev/null
    fi
    if timeout 2 bash -c "</dev/tcp/${nanopi_host}/${nanopi_port}" 2>/dev/null; then
        ros2 run rov_thruster_bridge communication_stress_test \
            --host "${nanopi_host}" --port "${nanopi_port}" \
            --duration 1 --progress-interval 1 \
            --report "${report_dir}/final-zero.json" \
            >"${report_dir}/final-zero.log" 2>&1
    fi
    if [[ -n "${zed_pid}" ]] && kill -0 "${zed_pid}" 2>/dev/null; then
        kill -INT -- "-${zed_pid}" 2>/dev/null
        wait "${zed_pid}" 2>/dev/null
    fi
    echo "Competition report: ${report_dir}"
}
trap cleanup EXIT INT TERM

zed_args=(--ros --model "${model}" --labels "${labels}")
if [[ "${preview}" == 'true' ]]; then
    if ss -ltnH "sport = :${preview_port}" | grep -q .; then
        echo "FAILED: preview port ${preview_port} is already in use." >&2
        exit 6
    fi
    zed_args+=(--web --web-host "${preview_host}" --web-port "${preview_port}")
    echo "YOLO preview: http://${preview_host}:${preview_port}"
fi

setsid bash /home/auv/jetson-zed-yolo/run.sh "${zed_args[@]}" \
    >>"${report_dir}/zed.log" 2>&1 &
zed_pid=$!

publisher_count='0'
for _ in $(seq 1 120); do
    kill -0 "${zed_pid}" 2>/dev/null || {
        echo 'FAILED: ZED detector exited.' >&2
        tail -30 "${report_dir}/zed.log" >&2
        exit 7
    }
    publisher_count="$(
        ros2 topic info /zed/objects3d --no-daemon 2>/dev/null \
            | awk '/Publisher count:/ {print $3}' || true
    )"
    [[ "${publisher_count:-0}" -eq 1 ]] && break
    sleep 0.5
done
if [[ "${publisher_count:-0}" -ne 1 ]]; then
    echo 'FAILED: expected exactly one /zed/objects3d publisher.' >&2
    exit 8
fi

setsid ros2 launch auv_competition_control competition_mission.launch.py \
    use_mock:=false "target_ball_color:=${target_ball_color}" \
    >"${report_dir}/launch.log" 2>&1 &
launch_pid=$!

service_ready='false'
for _ in $(seq 1 80); do
    if ros2 service type /auv/mission_enable 2>/dev/null \
            | grep -q 'std_srvs/srv/SetBool'; then
        service_ready='true'
        break
    fi
    kill -0 "${launch_pid}" 2>/dev/null || {
        tail -40 "${report_dir}/launch.log" >&2
        exit 9
    }
    sleep 0.25
done
if [[ "${service_ready}" != 'true' ]]; then
    echo 'FAILED: mission-enable service did not appear.' >&2
    exit 9
fi

connected='false'
for _ in $(seq 1 20); do
    if timeout 2 ros2 topic echo /auv/connected std_msgs/msg/Bool \
            --once --full-length 2>/dev/null | grep -q 'data: true'; then
        connected='true'
        break
    fi
done
if [[ "${connected}" != 'true' ]]; then
    echo 'FAILED: ROS bridge did not connect to NanoPi.' >&2
    exit 10
fi

vertical_control_mode="$(
    ros2 param get /competition_controller vertical_control_mode 2>/dev/null \
        | awk -F': ' '{print $NF}' || true
)"
printf '%s\n' "${vertical_control_mode}" \
    >"${report_dir}/vertical-control-mode.txt"
if [[ "${vertical_control_mode}" == 'visual_heave' ]]; then
    echo 'Vertical control: visual_heave (pressure-depth checks skipped).'
    echo 'WARNING: there is no absolute-depth hold; place the AUV near its working depth.'
elif [[ "${vertical_control_mode}" == 'pressure_depth' ]]; then
    if ! ros2 param get /auv/rov_bridge onboard_depth_hold_enabled 2>/dev/null \
            | grep -qi 'true'; then
        echo 'FAILED: onboard_depth_hold_enabled is false.' >&2
        echo 'Install and validate the depth sensor before pressure-depth mode.' >&2
        exit 11
    fi
    if ! timeout 5 ros2 topic echo /auv/depth_m std_msgs/msg/Float32 \
            --once --full-length >"${report_dir}/initial-depth.txt" 2>&1; then
        echo 'FAILED: no valid /auv/depth_m sample arrived within 5 seconds.' >&2
        exit 12
    fi
else
    echo "FAILED: unsupported vertical_control_mode: ${vertical_control_mode:-unknown}" >&2
    exit 11
fi

for topic in /auv/control_command /auv/motion_enable; do
    ownership="$(ros2 topic info "${topic}" --no-daemon 2>/dev/null || true)"
    printf '%s\n' "${ownership}" \
        >"${report_dir}/$(basename "${topic}")-ownership.txt"
    if ! grep -q 'Publisher count: 1' <<<"${ownership}" \
            || ! grep -q 'Subscription count: 1' <<<"${ownership}"; then
        echo "FAILED: ${topic} ownership is not exactly 1-to-1." >&2
        printf '%s\n' "${ownership}" >&2
        exit 13
    fi
done

ros2 bag record -o "${report_dir}/rosbag" \
    /zed/objects3d /auv/mission_state /auv/selected_target \
    /auv/control_command /auv/motion_enable /auv/target_depth_m \
    /auv/depth_m /auv/connected /auv/propeller_power \
    /auv/bridge_diagnostics \
    >"${report_dir}/rosbag.log" 2>&1 &
bag_pid=$!
sleep 1
if ! kill -0 "${bag_pid}" 2>/dev/null; then
    echo 'FAILED: rosbag recorder exited before mission arm.' >&2
    tail -30 "${report_dir}/rosbag.log" >&2
    exit 14
fi

echo "LIVE COMPETITION: target=${target_ball_color}, domain=${ROS_DOMAIN_ID}"
echo "Model SHA-256: ${model_sha256}"
ros2 service call /auv/mission_enable \
    std_srvs/srv/SetBool '{data: true}' \
    | tee "${report_dir}/arm.log"
armed='true'
echo 'Mission armed. Press Ctrl+C for controlled disarm and final zero.'

if [[ "${duration}" == '0' ]]; then
    while kill -0 "${launch_pid}" 2>/dev/null \
            && kill -0 "${zed_pid}" 2>/dev/null; do
        sleep 1
    done
    echo 'FAILED: detector or launch exited unexpectedly.' >&2
    exit 15
else
    sleep "${duration}"
fi
