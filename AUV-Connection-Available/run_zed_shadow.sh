#!/usr/bin/env bash
set -eo pipefail

model="${AUV_YOLO_MODEL:-/home/auv/jetson-zed-yolo/models/competition.onnx}"
labels="${AUV_YOLO_LABELS:-/home/auv/jetson-zed-yolo/models/competition_classes.json}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
source /opt/ros/humble/setup.bash
source /home/auv/ros2_ws/install/setup.bash
set -u

if [[ ! -r "${model}" || ! -r "${labels}" ]]; then
    echo "FAILED: competition model or labels file is not readable." >&2
    echo "Model: ${model}" >&2
    echo "Labels: ${labels}" >&2
    exit 2
fi

if pgrep -af 'rov_thruster_bridge.*thruster_bridge|thruster_bridge_node' \
    >/dev/null; then
    echo 'REFUSED: a real thruster bridge process is running.' >&2
    echo 'Stop it before starting shadow mode.' >&2
    exit 2
fi

zed_pid=''
cleanup() {
    if [[ -n "${zed_pid}" ]] && kill -0 "${zed_pid}" 2>/dev/null; then
        kill -INT "${zed_pid}" 2>/dev/null || true
        wait "${zed_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if pgrep -af '/usr/bin/python3 -u detect3d.py.*--ros' >/dev/null; then
    echo 'REFUSED: a ZED detector is already running with an unknown ROS domain.' >&2
    echo 'Stop it first so shadow-mode source provenance is unambiguous.' >&2
    exit 3
fi

mkdir -p /home/auv/jetson-zed-yolo/logs
bash /home/auv/jetson-zed-yolo/run.sh --ros \
    --model "${model}" --labels "${labels}" \
    >>/home/auv/jetson-zed-yolo/logs/shadow-zed.log 2>&1 &
zed_pid=$!
echo "Started real ZED detector pid=${zed_pid}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

zed_ready='false'
for _ in $(seq 1 120); do
    if ! kill -0 "${zed_pid}" 2>/dev/null; then
        echo 'FAILED: ZED detector exited before publishing objects3d.' >&2
        tail -30 /home/auv/jetson-zed-yolo/logs/shadow-zed.log >&2 || true
        exit 4
    fi
    publisher_count="$(
        ros2 topic info /zed/objects3d --no-daemon 2>/dev/null \
            | awk '/Publisher count:/ {print $3}' || true
    )"
    if [[ "${publisher_count:-0}" -ge 1 ]]; then
        zed_ready='true'
        break
    fi
    sleep 0.5
done
if [[ "${zed_ready}" != 'true' ]]; then
    echo 'FAILED: no /zed/objects3d publisher appeared within 60 seconds.' >&2
    exit 5
fi
echo 'Real ZED objects3d publisher is ready.'

echo 'Starting actuation-isolated shadow controller.'
echo 'Competition classes: red_ball, blue_ball, gate.'
ros2 launch auv_competition_control competition_shadow.launch.py "$@"
