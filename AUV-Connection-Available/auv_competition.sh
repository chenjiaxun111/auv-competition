#!/usr/bin/env bash
# One-command Jetson supervisor for the dual-model AUV competition mission.

set -Eeuo pipefail

runtime_dir='/home/auv/competition_runtime'
pid_file="${runtime_dir}/supervisor.pid"
state_file="${runtime_dir}/state"
latest_link="${runtime_dir}/latest"
ros_domain_id='96'
nanopi_host='192.168.137.219'
nanopi_port='8888'
default_detector='/home/auv/jetson-zed-yolo/models/competition_latest.pt'
default_pose='/home/auv/jetson-zed-yolo/models/gate_pose.pt'
workspace='/home/auv/ros2_ws'
script_path="$(readlink -f "$0")"

usage() {
    cat <<'EOF'
用法：
  bash /home/auv/auv_competition.sh preflight
  bash /home/auv/auv_competition.sh start --allow-motion [选项]
  bash /home/auv/auv_competition.sh status
  bash /home/auv/auv_competition.sh logs
  bash /home/auv/auv_competition.sh monitor
  bash /home/auv/auv_competition.sh preview
  bash /home/auv/auv_competition.sh vision-debug
  bash /home/auv/auv_competition.sh stop

start 选项：
  --target red|blue       本次撞击的球色，默认 red
  --duration 秒           0 表示一直运行到 stop，默认 0
  --detector 路径         球/门检测 PT 模型
  --pose 路径             门框四点 Pose PT 模型
  --debug-image           发布 JPEG 调试图（会降低帧率）
  --allow-motion          必须提供；确认本次启动允许真实运动
  --prepare-only          仅启动相机/模型/控制链，不解锁任务
  --detach                启动完成后返回，不持续显示（默认实时显示）

比赛启动：
  bash /home/auv/auv_competition.sh start --allow-motion --target red

紧急软件停车：
  bash /home/auv/auv_competition.sh stop

注意：软件停车不能代替物理断电/急停。
图像：比赛 start 添加 --debug-image；另一个 SSH 终端运行 preview，
岸上浏览器打开 http://192.168.137.1:8081 。
vision-debug 仅启动相机/双模型/网页，不启动控制器或通信桥；Ctrl+C 停止。
EOF
}

source_ros() {
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source "${workspace}/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ros_domain_id}"
}

supervisor_pid() {
    [[ -r "${pid_file}" ]] || return 1
    local pid
    pid="$(<"${pid_file}")"
    [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
    kill -0 "${pid}" 2>/dev/null || return 1
    tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null \
        | grep -Fq 'auv_competition.sh __run' || return 1
    printf '%s\n' "${pid}"
}

require_no_competing_processes() {
    local matches
    matches="$(pgrep -af \
        '[z]ed_sdk_source|[d]ual_model_perception|[c]ompetition_controller|[t]hruster_bridge_node' \
        || true)"
    if [[ -n "${matches}" ]]; then
        echo '失败：发现已有相机、感知、控制器或 NanoPi 桥进程：' >&2
        printf '%s\n' "${matches}" >&2
        return 1
    fi
}

basic_preflight() {
    local detector="$1"
    local pose="$2"
    [[ -r "${detector}" ]] || { echo "失败：检测模型不可读：${detector}" >&2; return 1; }
    [[ -r "${pose}" ]] || { echo "失败：Pose 模型不可读：${pose}" >&2; return 1; }
    [[ -r /opt/ros/humble/setup.bash ]] || { echo '失败：ROS 2 Humble 不存在。' >&2; return 1; }
    [[ -r "${workspace}/install/setup.bash" ]] || { echo '失败：ROS 工作空间尚未安装。' >&2; return 1; }
    lsusb | grep -qi '2b03:' || { echo '失败：没有发现 ZED USB 设备。' >&2; return 1; }
    timeout 2 bash -c "</dev/tcp/${nanopi_host}/${nanopi_port}" \
        || { echo "失败：NanoPi ${nanopi_host}:${nanopi_port} 不可达。" >&2; return 1; }
    require_no_competing_processes
    source_ros
    ros2 pkg prefix auv_competition_control >/dev/null
    ros2 pkg prefix rov_thruster_bridge >/dev/null
    echo '预检通过：ZED、两个模型、ROS 2 与 NanoPi RPC 均可用。'
}

topic_has_one_to_one() {
    local topic="$1"
    local info attempt
    for ((attempt=0; attempt<3; attempt++)); do
        info="$(timeout 15 ros2 topic info "${topic}" --no-daemon --spin-time 3 2>&1 || true)"
        if grep -Eq '^Publisher count: 1[[:space:]]*$' <<<"${info}" \
                && grep -Eq '^Subscription count: 1[[:space:]]*$' <<<"${info}"; then
            return 0
        fi
        sleep 0.5
    done
    printf '话题 %s 的实际端点：\n%s\n' "${topic}" "${info}" >&2
    return 1
}

wait_for_true_connection() {
    local attempts="$1"
    local i
    for ((i=0; i<attempts; i++)); do
        if timeout 2 ros2 topic echo /auv/connected std_msgs/msg/Bool \
                --once --full-length 2>/dev/null | grep -q 'data: true'; then
            return 0
        fi
    done
    return 1
}

send_final_zero() {
    local report_dir="$1"
    # Remote power feedback is not physical RPM/PWM measurement.
    timeout 12 /usr/bin/python3 "$(dirname "${script_path}")/competition_zero_check.py" \
        --host "${nanopi_host}" --port "${nanopi_port}" \
        --client-root "${workspace}/src/rov_thruster_bridge" \
        >"${report_dir}/final-zero.log" 2>&1
}

stop_child() {
    local pid="$1" n
    [[ -n "${pid}" ]] || return 0
    # ros2 launch forwards SIGINT itself; signalling its whole group twice can
    # interrupt a camera close() halfway through cleanup.
    kill -INT "${pid}" 2>/dev/null || true
    for ((n=0; n<8; n++)); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 1
    done
    kill -TERM -- "-${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${pid}" 2>/dev/null || true
    for ((n=0; n<2; n++)); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "进程 ${pid} 未退出，不能确认清理成功。" >&2
        return 1
    fi
    wait "${pid}" 2>/dev/null || true
}

command_preflight() {
    mkdir -p "${runtime_dir}"
    if supervisor_pid >/dev/null 2>&1; then
        echo '比赛程序已经运行；请使用 status 查看。' >&2
        return 1
    fi
    basic_preflight "${default_detector}" "${default_pose}"
}

command_start() {
    local target='red'
    local duration='0'
    local detector="${default_detector}"
    local pose="${default_pose}"
    local debug_image='false'
    local allow_motion='false'
    local prepare_only='false'
    local detach='false'
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --target) target="${2:?缺少球色}"; shift 2 ;;
            --duration) duration="${2:?缺少时长}"; shift 2 ;;
            --detector) detector="${2:?缺少检测模型路径}"; shift 2 ;;
            --pose) pose="${2:?缺少 Pose 模型路径}"; shift 2 ;;
            --debug-image) debug_image='true'; shift ;;
            --allow-motion) allow_motion='true'; shift ;;
            --prepare-only) prepare_only='true'; shift ;;
            --detach) detach='true'; shift ;;
            -h|--help) usage; return 0 ;;
            *) echo "未知参数：$1" >&2; return 2 ;;
        esac
    done
    [[ "${allow_motion}" == 'true' || "${prepare_only}" == 'true' ]] || {
        echo '拒绝启动：正式模式可驱动推进器，必须添加 --allow-motion。' >&2
        return 2
    }
    [[ "${target}" == 'red' || "${target}" == 'blue' ]] \
        || { echo '球色必须是 red 或 blue。' >&2; return 2; }
    [[ "${duration}" =~ ^[0-9]+$ ]] \
        || { echo 'duration 必须是非负整数秒。' >&2; return 2; }
    mkdir -p "${runtime_dir}"
    exec 9>"${runtime_dir}/start.lock"
    flock -n 9 || { echo '另一条启动命令正在执行。' >&2; return 3; }
    if supervisor_pid >/dev/null 2>&1; then
        echo '拒绝启动：比赛程序已经运行。' >&2
        return 3
    fi
    basic_preflight "${detector}" "${pose}"

    local stamp report_dir pid deadline
    stamp="$(date +%Y%m%d-%H%M%S)"
    report_dir="${workspace}/competition_runs/${stamp}"
    mkdir -p "${report_dir}"
    ln -sfn "${report_dir}" "${latest_link}"
    rm -f "${runtime_dir}/ready" "${runtime_dir}/failed"
    printf 'STARTING\n' >"${state_file}"

    AUV_SUPERVISOR_INTERNAL=1 nohup setsid bash "${script_path}" __run \
        --target "${target}" --duration "${duration}" \
        --detector "${detector}" --pose "${pose}" \
        --debug-image "${debug_image}" --report-dir "${report_dir}" \
        --prepare-only "${prepare_only}" \
        >"${report_dir}/supervisor.log" 2>&1 </dev/null 9>&- &
    pid=$!
    printf '%s\n' "${pid}" >"${pid_file}"

    deadline=$((SECONDS + 240))
    while (( SECONDS < deadline )); do
        if [[ -f "${runtime_dir}/ready" ]]; then
            echo "程序启动完成：$(<"${state_file}")"
            echo "任务球色：${target}"
            echo "运行记录：${report_dir}"
            echo '停车命令：bash /home/auv/auv_competition.sh stop'
            exec 9>&-
            if [[ "${detach}" == 'false' ]]; then
                command_monitor
            fi
            return 0
        fi
        if [[ -f "${runtime_dir}/failed" ]] || ! kill -0 "${pid}" 2>/dev/null; then
            echo '启动失败，最近日志如下：' >&2
            tail -60 "${report_dir}/supervisor.log" >&2 || true
            return 4
        fi
        sleep 1
    done
    echo '启动超时，正在执行安全停车。' >&2
    kill -TERM "${pid}" 2>/dev/null || true
    return 5
}

command_status() {
    local pid=''
    if pid="$(supervisor_pid 2>/dev/null)"; then
        echo "状态：运行中（PID ${pid}）"
        [[ -r "${state_file}" ]] && echo "阶段：$(<"${state_file}")"
        echo "记录：$(readlink -f "${latest_link}" 2>/dev/null || true)"
        source_ros
        timeout 3 ros2 topic echo /auv/mission_state std_msgs/msg/String \
            --once --full-length 2>/dev/null || true
        timeout 3 ros2 topic echo /auv/connected std_msgs/msg/Bool \
            --once --full-length 2>/dev/null || true
        return 0
    fi
    echo '状态：未运行'
    [[ -r "${state_file}" ]] && echo "上次状态：$(<"${state_file}")"
}

command_logs() {
    local report_dir
    report_dir="$(readlink -f "${latest_link}" 2>/dev/null || true)"
    [[ -n "${report_dir}" && -d "${report_dir}" ]] \
        || { echo '还没有比赛运行记录。' >&2; return 1; }
    echo "记录目录：${report_dir}"
    echo '--- supervisor.log ---'
    tail -80 "${report_dir}/supervisor.log" 2>/dev/null || true
    echo '--- launch.log ---'
    tail -80 "${report_dir}/launch.log" 2>/dev/null || true
}

command_monitor() {
    local pid
    pid="$(supervisor_pid)" || { echo '比赛程序未运行，无法监视。' >&2; return 1; }
    source_ros
    /usr/bin/python3 "$(dirname "${script_path}")/competition_terminal_monitor.py" \
        --supervisor-pid "${pid}" --state-file "${state_file}"
}

command_stop() {
    local pid report_dir deadline
    mkdir -p "${runtime_dir}"
    report_dir="$(readlink -f "${latest_link}" 2>/dev/null || true)"
    if ! pid="$(supervisor_pid 2>/dev/null)"; then
        echo '比赛程序未运行。仍尝试独立回零。'
        source_ros
        [[ -n "${report_dir}" && -d "${report_dir}" ]] \
            || report_dir="${runtime_dir}"
        send_final_zero "${report_dir}" || { echo '未能确认 NanoPi 回零，请检查 final-zero.log。' >&2; return 20; }
        return 0
    fi

    source_ros
    timeout 4 ros2 service call /auv/mission_enable \
        std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true
    # nohup may inherit ignored SIGINT; TERM reliably reaches the trap.
    kill -TERM "${pid}" 2>/dev/null || true
    deadline=$((SECONDS + 40))
    while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
        sleep 1
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo '警告：正常停止超时，发送 TERM。' >&2
        kill -TERM "${pid}" 2>/dev/null || true
        sleep 3
    fi
    if supervisor_pid >/dev/null 2>&1; then
        echo '监督进程仍未退出，不能确认已停止；请检查日志并使用物理急停。' >&2
        return 21
    fi
    [[ -n "${report_dir}" && -d "${report_dir}" ]] \
        || report_dir="${runtime_dir}"
    send_final_zero "${report_dir}" || { echo '停车回零检查失败，请查看 final-zero.log。' >&2; return 20; }
    echo '比赛进程已停止；NanoPi 六路功率反馈为零（不是实测转速）。'
}

internal_run() {
    [[ "${AUV_SUPERVISOR_INTERNAL:-}" == 1 ]] || { echo '请使用 start 入口。' >&2; return 2; }
    # EXIT is invoked after this function returns. Cleanup state must outlive
    # function-local scope (otherwise set -u aborts cleanup on a startup error).
    target='red' duration='0' detector='' pose='' debug_image='false' report_dir='' prepare_only='false'
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --target) target="$2"; shift 2 ;;
            --duration) duration="$2"; shift 2 ;;
            --detector) detector="$2"; shift 2 ;;
            --pose) pose="$2"; shift 2 ;;
            --debug-image) debug_image="$2"; shift 2 ;;
            --report-dir) report_dir="$2"; shift 2 ;;
            --prepare-only) prepare_only="$2"; shift 2 ;;
            *) echo "内部参数错误：$1" >&2; return 2 ;;
        esac
    done
    source_ros
    launch_pid='' bag_pid='' armed='false' exit_code=0 started_s=0

    cleanup_internal() {
        local original_status=$?
        if (( original_status != 0 && exit_code == 0 )); then exit_code=${original_status}; fi
        trap - EXIT
        trap '' INT TERM
        set +e
        printf 'STOPPING\n' >"${state_file}"
        if [[ -n "${launch_pid}" ]]; then
            timeout 4 ros2 service call /auv/mission_enable \
                std_srvs/srv/SetBool '{data: false}' \
                >"${report_dir}/disarm.log" 2>&1
            armed='false'
            sleep 0.5
        fi
        stop_child "${launch_pid}" || exit_code=23
        stop_child "${bag_pid}" || exit_code=23
        send_final_zero "${report_dir}" || exit_code=20
        rm -f "${runtime_dir}/ready"
        printf '%s\n' "$([[ ${exit_code} -eq 0 ]] && echo STOPPED || echo FAILED)" \
            >"${state_file}"
        if [[ -r "${pid_file}" && "$(<"${pid_file}")" == "$$" ]]; then
            rm -f "${pid_file}"
        fi
        [[ ${exit_code} -eq 0 ]] || touch "${runtime_dir}/failed"
        exit "${exit_code}"
    }
    trap cleanup_internal EXIT
    trap 'exit 0' INT TERM

    basic_preflight "${detector}" "${pose}"
    sha256sum "${detector}" "${pose}" >"${report_dir}/model-sha256.txt"
    printf 'PREFLIGHT_ZERO\n' >"${state_file}"
    send_final_zero "${report_dir}"

    printf 'STARTING_ROS\n' >"${state_file}"
    # Background/nohup Bash children inherit ignored SIGINT. Reset it before
    # exec so ros2 launch can forward graceful shutdown to its child nodes.
    setsid env --default-signal=INT ros2 launch auv_competition_control dual_model_mission.launch.py \
        shadow_mode:=false start_zed_sdk_source:=true auto_start:=false \
        "target_ball_color:=${target}" vertical_control_mode:=visual_heave \
        "detector_model_path:=${detector}" "gate_pose_model_path:=${pose}" \
        inference_device:=0 use_half_precision:=true \
        "publish_debug_image:=${debug_image}" \
        >"${report_dir}/launch.log" 2>&1 &
    launch_pid=$!

    local service_ready='false'
    for _ in $(seq 1 120); do
        kill -0 "${launch_pid}" 2>/dev/null || {
            echo '失败：ROS launch 在就绪前退出。' >&2
            exit_code=10
            return
        }
        if timeout 3 ros2 service type /auv/mission_enable 2>/dev/null \
                | grep -q 'std_srvs/srv/SetBool'; then
            service_ready='true'
            break
        fi
        sleep 0.5
    done
    [[ "${service_ready}" == 'true' ]] \
        || { echo '失败：任务控制服务未出现。' >&2; exit_code=11; return; }

    timeout 20 ros2 topic echo /zed/sdk_source/diagnostics std_msgs/msg/String \
        --once --full-length >"${report_dir}/zed-ready.txt" 2>&1 \
        || { echo '失败：ZED 没有输出有效帧。' >&2; exit_code=12; return; }
    wait_for_true_connection 15 \
        || { echo '失败：ROS 桥未连接 NanoPi。' >&2; exit_code=13; return; }
    local models_ready='false'
    for _ in $(seq 1 120); do
        if grep -q 'Prewarmed ball detector' "${report_dir}/launch.log" \
                && grep -q 'Prewarmed gate Pose' "${report_dir}/launch.log" \
                && grep -q 'Dual-model coordinator ready' "${report_dir}/launch.log"; then
            models_ready='true'
            break
        fi
        kill -0 "${launch_pid}" 2>/dev/null || break
        sleep 0.5
    done
    [[ "${models_ready}" == 'true' ]] \
        || { echo '失败：两个模型没有完成预热。' >&2; exit_code=16; return; }
    topic_has_one_to_one /auv/control_command \
        || { echo '失败：control_command 不是唯一 1:1 链路。' >&2; exit_code=14; return; }
    topic_has_one_to_one /auv/motion_enable \
        || { echo '失败：motion_enable 不是唯一 1:1 链路。' >&2; exit_code=15; return; }

    setsid env --default-signal=INT ros2 bag record -o "${report_dir}/rosbag" \
        /auv/perception/ball_observations \
        /auv/perception/gate_pose_observations \
        /auv/perception/diagnostics /auv/mission_state \
        /auv/selected_target /auv/control_command /auv/motion_enable \
        /auv/connected /auv/propeller_power /auv/bridge_diagnostics \
        >"${report_dir}/rosbag.log" 2>&1 &
    bag_pid=$!
    sleep 1
    kill -0 "${bag_pid}" 2>/dev/null \
        || { echo '失败：rosbag 未能启动。' >&2; exit_code=18; return; }

    if [[ "${prepare_only}" == 'false' ]]; then
    printf 'ARMING\n' >"${state_file}"
    if ! timeout 5 ros2 service call /auv/mission_enable \
            std_srvs/srv/SetBool '{data: true}' \
            >"${report_dir}/arm.log" 2>&1; then
        echo '失败：任务解锁服务调用失败。' >&2
        exit_code=19
        return
    fi
    grep -Eq 'success=True|success: true' "${report_dir}/arm.log" \
        || { echo '失败：任务控制器拒绝解锁。' >&2; exit_code=19; return; }
    armed='true'
    printf 'RUNNING\n' >"${state_file}"
    else
        printf 'READY_DISARMED\n' >"${state_file}"
    fi
    touch "${runtime_dir}/ready"
    echo "启动完成：prepare_only=${prepare_only}, target=${target}, ROS_DOMAIN_ID=${ros_domain_id}"
    echo "记录目录：${report_dir}"

    started_s="${SECONDS}"
    while kill -0 "${launch_pid}" 2>/dev/null; do
        kill -0 "${bag_pid}" 2>/dev/null || { echo '记录进程退出，停止任务。' >&2; exit_code=22; return; }
        if [[ "${duration}" != '0' ]] \
                && (( SECONDS - started_s >= duration )); then
            return
        fi
        sleep 1
    done
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
        echo '失败：比赛 ROS launch 意外退出。' >&2
        exit_code=21
    fi
}

action="${1:-status}"
[[ $# -gt 0 ]] && shift
case "${action}" in
    preflight) command_preflight "$@" ;;
    start) command_start "$@" ;;
    status) command_status "$@" ;;
    logs) command_logs "$@" ;;
    monitor) command_monitor "$@" ;;
    preview)
        source_ros
        exec ros2 run auv_competition_control perception_web_viewer "$@"
        ;;
    vision-debug)
        require_no_competing_processes
        source_ros
        # Separate DDS domain; this branch never calls RPC or arm services.
        export ROS_DOMAIN_ID=97
        exec ros2 launch auv_competition_control vision_debug.launch.py "$@"
        ;;
    stop) command_stop "$@" ;;
    __run) internal_run "$@" ;;
    -h|--help|help) usage ;;
    *) echo "未知操作：${action}" >&2; usage >&2; exit 2 ;;
esac
