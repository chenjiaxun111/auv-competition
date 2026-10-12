#!/usr/bin/env bash
set -eo pipefail

duration='30'
target_ball_color='red'
red_ball_class='red_ball'
blue_ball_class='blue_ball'
gate_class='gate'
model='/home/auv/jetson-zed-yolo/models/competition.onnx'
labels='/home/auv/jetson-zed-yolo/models/competition_classes.json'
allow_pwm='false'
preview='none'
preview_host='192.168.3.19'
preview_port='8080'

usage() {
    cat <<'EOF'
Usage: run_full_chain_bench.sh --allow-pwm [options]

Options:
  --duration SECONDS       Armed test duration (default: 30)
  --target-ball-color C    Ball to hit: red or blue (default: red)
  --red-ball-class NAME    Detector red-ball class (default: red_ball)
  --blue-ball-class NAME   Detector blue-ball class (default: blue_ball)
  --gate-class NAME        Detector gate class (default: gate)
  --model PATH             Detector ONNX model path
  --labels PATH            Detector class JSON path
  --web-preview            Show annotated YOLO video in a browser
  --preview-host ADDRESS   Preview bind/address (default: 192.168.3.19)
  --preview-port PORT      Preview HTTP port (default: 8080)
  --jetson-view            Show an OpenCV window on the Jetson desktop
  --allow-pwm              Required acknowledgement that PWM will change
  -h, --help               Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --allow-pwm) allow_pwm='true'; shift ;;
        --duration) duration="${2:?missing duration}"; shift 2 ;;
        --target-ball-color) target_ball_color="${2:?missing target ball color}"; shift 2 ;;
        --red-ball-class) red_ball_class="${2:?missing red-ball class}"; shift 2 ;;
        --blue-ball-class) blue_ball_class="${2:?missing blue-ball class}"; shift 2 ;;
        --gate-class) gate_class="${2:?missing gate class}"; shift 2 ;;
        --model) model="${2:?missing model path}"; shift 2 ;;
        --labels) labels="${2:?missing labels path}"; shift 2 ;;
        --web-preview) preview='web'; shift ;;
        --preview-host) preview_host="${2:?missing preview host}"; shift 2 ;;
        --preview-port) preview_port="${2:?missing preview port}"; shift 2 ;;
        --jetson-view) preview='view'; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [[ "${allow_pwm}" != 'true' ]]; then
    echo 'REFUSED: this test changes NanoPi PWM outputs.' >&2
    echo 'Disconnect all thrusters, then add --allow-pwm.' >&2
    exit 2
fi
if ! [[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]] \
        || [[ "${duration}" == '0' ]]; then
    echo 'Duration must be a positive number.' >&2
    exit 2
fi
if ! [[ "${preview_port}" =~ ^[0-9]+$ ]] \
        || (( preview_port < 1 || preview_port > 65535 )); then
    echo 'Preview port must be an integer in [1, 65535].' >&2
    exit 2
fi
if [[ "${target_ball_color}" != 'red' && "${target_ball_color}" != 'blue' ]]; then
    echo 'Target ball color must be red or blue.' >&2
    exit 2
fi
if [[ ! -r "${model}" || ! -r "${labels}" ]]; then
    echo "FAILED: model or labels file is not readable: ${model}, ${labels}" >&2
    exit 4
fi

# Use a dedicated graph every time so a value inherited from .bashrc cannot
# silently attach this hardware bench to another ROS system.
export ROS_DOMAIN_ID=96
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
set -u

if pgrep -af 'detect3d.py|competition_(shadow|bench)_controller|thruster_bridge_node' \
    >/dev/null; then
    echo 'REFUSED: a ZED, competition controller, or bridge process is running.' >&2
    exit 3
fi
if ! timeout 2 bash -c '</dev/tcp/192.168.137.219/8888'; then
    echo 'FAILED: NanoPi RPC 192.168.137.219:8888 is not reachable.' >&2
    exit 4
fi

report_dir="/home/auv/ros2_ws/bench_reports/$(date +%Y%m%d-%H%M%S)"
mkdir -p "${report_dir}"
zed_pid=''
launch_pid=''
bag_pid=''
zed_args=(--ros --model "${model}" --labels "${labels}")
if [[ "${preview}" == 'web' ]]; then
    if ss -ltnH "sport = :${preview_port}" | grep -q .; then
        echo "FAILED: preview port ${preview_port} is already in use." >&2
        exit 4
    fi
    zed_args+=(
        --web --web-host "${preview_host}" --web-port "${preview_port}"
    )
    echo "YOLO live preview: http://${preview_host}:${preview_port}"
elif [[ "${preview}" == 'view' ]]; then
    zed_args+=(--view)
    echo 'YOLO live preview will open on the Jetson desktop.'
fi

cleanup() {
    trap - EXIT INT TERM
    set +e
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        timeout 3 ros2 service call /auv/mission_enable \
            std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1
        sleep 0.6
        kill -INT -- "-${launch_pid}" 2>/dev/null
        wait "${launch_pid}" 2>/dev/null
    fi
    if timeout 2 bash -c '</dev/tcp/192.168.137.219/8888' 2>/dev/null; then
        ros2 run rov_thruster_bridge communication_stress_test \
            --duration 1 --progress-interval 1 \
            --report "${report_dir}/final-zero.json"
    fi
    if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        kill -INT "${bag_pid}" 2>/dev/null
        wait "${bag_pid}" 2>/dev/null
    fi
    if [[ -n "${zed_pid}" ]] && kill -0 "${zed_pid}" 2>/dev/null; then
        kill -INT -- "-${zed_pid}" 2>/dev/null
        wait "${zed_pid}" 2>/dev/null
    fi
    echo "Bench report: ${report_dir}"
}
trap cleanup EXIT INT TERM

setsid bash /home/auv/jetson-zed-yolo/run.sh "${zed_args[@]}" \
    >>"${report_dir}/zed.log" 2>&1 &
zed_pid=$!
publisher_count='0'
for _ in $(seq 1 120); do
    kill -0 "${zed_pid}" 2>/dev/null || {
        echo 'FAILED: ZED detector exited.' >&2
        tail -30 "${report_dir}/zed.log" >&2
        exit 5
    }
    publisher_count="$(
        ros2 topic info /zed/objects3d --no-daemon 2>/dev/null \
            | awk '/Publisher count:/ {print $3}' || true
    )"
    [[ "${publisher_count:-0}" -eq 1 ]] && break
    sleep 0.5
done
if [[ "${publisher_count:-0}" -ne 1 ]]; then
    echo 'FAILED: expected exactly one ZED publisher.' >&2
    exit 6
fi

setsid ros2 launch auv_competition_control full_chain_bench.launch.py \
    "target_ball_color:=${target_ball_color}" \
    "red_ball_class:=${red_ball_class}" \
    "blue_ball_class:=${blue_ball_class}" \
    "gate_class:=${gate_class}" \
    >"${report_dir}/launch.log" 2>&1 &
launch_pid=$!

service_ready='false'
for _ in $(seq 1 80); do
    if ros2 service type /auv/mission_enable 2>/dev/null \
            | grep -q 'std_srvs/srv/SetBool'; then
        service_ready='true'
        break
    fi
    kill -0 "${launch_pid}" 2>/dev/null || exit 7
    sleep 0.25
done
[[ "${service_ready}" == 'true' ]] || exit 7

connected='false'
for _ in $(seq 1 40); do
    if timeout 2 ros2 topic echo /auv/connected std_msgs/msg/Bool \
            --once --full-length 2>/dev/null | grep -q 'data: true'; then
        connected='true'
        break
    fi
done
if [[ "${connected}" != 'true' ]]; then
    echo 'FAILED: ROS bridge did not connect to NanoPi.' >&2
    exit 8
fi

ownership=''
ownership_ready='false'
for _ in $(seq 1 20); do
    ownership="$(
        ros2 topic info /auv/shadow/control_command --no-daemon \
            2>/dev/null || true
    )"
    if grep -q 'Publisher count: 1' <<<"${ownership}" \
            && grep -q 'Subscription count: 1' <<<"${ownership}"; then
        ownership_ready='true'
        break
    fi
    sleep 0.25
done
printf '%s\n' "${ownership}" >"${report_dir}/command-topic-ownership.txt"
if [[ "${ownership_ready}" != 'true' ]]; then
    echo 'FAILED: bench command topic ownership is not exactly 1-to-1.' >&2
    printf '%s\n' "${ownership}" >&2
    exit 9
fi

ros2 bag record -o "${report_dir}/rosbag" \
    /zed/objects3d /auv/mission_state /auv/selected_target \
    /auv/shadow/decision /auv/shadow/control_command \
    /auv/shadow/requested_motion_enable /auv/connected \
    /auv/propeller_power /auv/bridge_diagnostics \
    >"${report_dir}/rosbag.log" 2>&1 &
bag_pid=$!

echo "FULL CHAIN BENCH: ${duration}s, domain=${ROS_DOMAIN_ID}"
echo "Model: ${model}"
echo "Classes: target=${target_ball_color}, red=${red_ball_class}, blue=${blue_ball_class}, gate=${gate_class}"
echo 'Translation is limited to about 17%; search yaw is about 27%.'
echo 'Hit/retreat/pass motion segments are disabled.'
ros2 service call /auv/mission_enable \
    std_srvs/srv/SetBool '{data: true}'
sleep "${duration}"
