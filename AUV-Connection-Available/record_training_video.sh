#!/usr/bin/env bash
# Record ZED training data on the sealed Jetson as a replayable SVO2 file.
# This script intentionally does not start ROS, YOLO, the mission controller,
# the NanoPi bridge, or any actuator process.

set -eo pipefail

base_dir="${AUV_ZED_DATASET_DIR:-/home/auv/zed_datasets}"
zed_explorer="${ZED_EXPLORER_BIN:-ZED_Explorer}"
active_file="${base_dir}/.active_recording"

usage() {
    cat <<'EOF'
Usage:
  record_training_video.sh start [options]
  record_training_video.sh stop
  record_training_video.sh status
  record_training_video.sh list

Start options:
  --tag NAME           Short scene label (default: training)
  --duration SECONDS   Auto-stop after this time; 0 means until stop (default: 600)
  --resolution MODE    ZED mode passed to ZED_Explorer (default: HD720)
  --fps N              Camera frame rate (default: 30)
  --compression-mode N ZED SVO2 mode 0-6 (default: 2, lossy JPEG)
  --min-free-gb N      Refuse to start below N GiB free (default: 10)
  --web-preview        Record lossless SVO2 and serve a browser preview
  --preview-host ADDR  Preview bind/address (default: 192.168.137.1)
  --preview-port N     Preview HTTP port (default: 8080)
  --preview-fps N      Browser preview FPS (default: 8)
  --preview-width N    Browser preview width (default: 640)
  -h, --help           Show this help

Examples:
  bash /home/auv/record_training_video.sh start --tag empty_pool --duration 600
  bash /home/auv/record_training_video.sh status
  bash /home/auv/record_training_video.sh stop

Each session is stored below /home/auv/zed_datasets/<timestamp>-<tag>/.
The recorder survives an SSH disconnect. It must be stopped cleanly before
Jetson power is removed.
EOF
}

fail() {
    echo "FAILED: $*" >&2
    exit 1
}

ensure_base_dir() {
    mkdir -p "${base_dir}"
}

resolve_zed_explorer() {
    if command -v "${zed_explorer}" >/dev/null 2>&1; then
        zed_explorer="$(command -v "${zed_explorer}")"
    elif [[ "${zed_explorer}" == 'ZED_Explorer' \
            && -x /usr/local/zed/tools/ZED_Explorer ]]; then
        zed_explorer='/usr/local/zed/tools/ZED_Explorer'
    elif [[ ! -x "${zed_explorer}" ]]; then
        fail "ZED Explorer is not executable: ${zed_explorer}"
    fi
}

read_active_state() {
    active_pid=''
    active_session=''
    if [[ -r "${active_file}" ]]; then
        active_pid="$(sed -n '1p' "${active_file}")"
        active_session="$(sed -n '2p' "${active_file}")"
    fi
}

valid_active_state() {
    [[ "${active_pid:-}" =~ ^[0-9]+$ ]] || return 1
    case "${active_session:-}" in
        "${base_dir}"/*) ;;
        *) return 1 ;;
    esac
    kill -0 "${active_pid}" 2>/dev/null
}

clear_stale_state() {
    read_active_state
    if [[ -e "${active_file}" ]] && ! valid_active_state; then
        echo 'Removing stale recorder state file.' >&2
        rm -f -- "${active_file}"
        active_pid=''
        active_session=''
    fi
}

show_status() {
    ensure_base_dir
    clear_stale_state
    read_active_state
    if ! valid_active_state; then
        echo 'Recorder: STOPPED'
        df -h "${base_dir}" | tail -1
        return 0
    fi

    svo_file="${active_session}/raw.svo2"
    echo 'Recorder: RUNNING'
    echo "PID: ${active_pid}"
    echo "Session: ${active_session}"
    if [[ -r "${active_session}/metadata.txt" ]]; then
        sed -n 's/^preview_url=/Live preview: /p' \
            "${active_session}/metadata.txt"
    fi
    ps -o pid=,pgid=,etime=,stat=,cmd= -p "${active_pid}" || true
    if [[ -e "${svo_file}" ]]; then
        ls -lh "${svo_file}"
    else
        echo 'SVO2 file has not appeared yet.'
    fi
    df -h "${base_dir}" | tail -1
}

start_recording() {
    tag='training'
    duration='600'
    resolution='HD720'
    fps='30'
    compression_mode='2'
    min_free_gb='10'
    web_preview='false'
    preview_host='192.168.137.1'
    preview_port='8080'
    preview_fps='8'
    preview_width='640'

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --tag) tag="${2:?missing tag}"; shift 2 ;;
            --duration) duration="${2:?missing duration}"; shift 2 ;;
            --resolution) resolution="${2:?missing resolution}"; shift 2 ;;
            --fps) fps="${2:?missing frame rate}"; shift 2 ;;
            --compression-mode) compression_mode="${2:?missing compression mode}"; shift 2 ;;
            --min-free-gb) min_free_gb="${2:?missing minimum free space}"; shift 2 ;;
            --web-preview) web_preview='true'; shift ;;
            --preview-host) preview_host="${2:?missing preview host}"; shift 2 ;;
            --preview-port) preview_port="${2:?missing preview port}"; shift 2 ;;
            --preview-fps) preview_fps="${2:?missing preview FPS}"; shift 2 ;;
            --preview-width) preview_width="${2:?missing preview width}"; shift 2 ;;
            -h|--help) usage; exit 0 ;;
            *) fail "unknown start option: $1" ;;
        esac
    done

    [[ "${tag}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] \
        || fail 'tag must use 1-64 ASCII letters, digits, dot, underscore, or dash'
    [[ "${duration}" =~ ^[0-9]+$ ]] \
        || fail 'duration must be a non-negative integer number of seconds'
    (( duration == 0 || duration >= 5 )) \
        || fail 'duration must be 0 (manual stop) or at least 5 seconds'
    [[ "${resolution}" =~ ^[A-Za-z0-9_]+$ ]] \
        || fail 'resolution contains unsupported characters'
    [[ "${fps}" =~ ^[0-9]+$ ]] && (( fps > 0 && fps <= 120 )) \
        || fail 'fps must be an integer between 1 and 120'
    [[ "${compression_mode}" =~ ^[0-6]$ ]] \
        || fail 'compression-mode must be an integer between 0 and 6'
    [[ "${min_free_gb}" =~ ^[0-9]+$ ]] \
        || fail 'min-free-gb must be a non-negative integer'
    [[ "${preview_host}" =~ ^[A-Za-z0-9.:_-]+$ ]] \
        || fail 'preview-host contains unsupported characters'
    [[ "${preview_port}" =~ ^[0-9]+$ ]] \
        && (( preview_port >= 1 && preview_port <= 65535 )) \
        || fail 'preview-port must be between 1 and 65535'
    [[ "${preview_fps}" =~ ^[0-9]+([.][0-9]+)?$ ]] \
        || fail 'preview-fps must be a positive number'
    awk -v value="${preview_fps}" \
        'BEGIN { exit !(value > 0 && value <= 30) }' \
        || fail 'preview-fps must be in (0, 30]'
    [[ "${preview_width}" =~ ^[0-9]+$ ]] \
        && (( preview_width >= 160 && preview_width <= 1920 )) \
        || fail 'preview-width must be between 160 and 1920'
    if [[ "${web_preview}" == 'true' ]] && (( min_free_gb < 1 )); then
        fail 'web-preview requires min-free-gb of at least 1'
    fi

    ensure_base_dir
    clear_stale_state
    read_active_state
    if valid_active_state; then
        fail "a recording is already active: PID ${active_pid}, ${active_session}"
    fi

    if [[ "${web_preview}" == 'true' ]]; then
        [[ -x /home/auv/record_zed_preview.py ]] \
            || fail '/home/auv/record_zed_preview.py is not installed or executable'
        if ss -ltnH "sport = :${preview_port}" | grep -q .; then
            fail "preview port ${preview_port} is already in use"
        fi
    else
        resolve_zed_explorer
    fi

    current_year="$(date +%Y)"
    if [[ "${current_year}" =~ ^[0-9]+$ ]] && (( current_year < 2024 )); then
        echo "WARNING: Jetson system clock is $(date --iso-8601=seconds); session timestamps will be wrong." >&2
        echo 'Synchronize the Jetson clock before formal data collection.' >&2
    fi

    conflicts="$(
        pgrep -af '[d]etect3d.py|[Z]ED_Explorer|[r]ecord_zed_preview.py|[a]uv_competition_control|[c]ompetition.*controller|[t]hruster_bridge_node|[r]un_competition.sh' \
            || true
    )"
    if [[ -n "${conflicts}" ]]; then
        echo 'Conflicting camera/control process found:' >&2
        printf '%s\n' "${conflicts}" >&2
        fail 'stop the process cleanly before recording training data'
    fi

    available_kib="$(df -Pk "${base_dir}" | awk 'NR==2 {print $4}')"
    required_kib=$((min_free_gb * 1024 * 1024))
    if ! [[ "${available_kib}" =~ ^[0-9]+$ ]] \
            || (( available_kib < required_kib )); then
        fail "less than ${min_free_gb} GiB is available below ${base_dir}"
    fi
    if [[ "${web_preview}" == 'true' ]] && (( duration > 0 )); then
        # Measured HD720@30 lossless SVO2 is about 55 MiB/s; budget 60 MiB/s
        # plus the requested free-space reserve.
        projected_kib=$((duration * 60 * 1024 + required_kib))
        if (( available_kib < projected_kib )); then
            projected_gib=$(((projected_kib + 1024 * 1024 - 1) / (1024 * 1024)))
            fail "web-preview lossless recording needs about ${projected_gib} GiB free including reserve"
        fi
    fi

    timestamp="$(date +%Y%m%d-%H%M%S)"
    session_dir="${base_dir}/${timestamp}-${tag}"
    suffix=0
    while [[ -e "${session_dir}" ]]; do
        suffix=$((suffix + 1))
        session_dir="${base_dir}/${timestamp}-${tag}-$(printf '%02d' "${suffix}")"
    done
    mkdir "${session_dir}"
    svo_file="${session_dir}/raw.svo2"
    log_file="${session_dir}/record.log"
    metadata_file="${session_dir}/metadata.txt"

    {
        echo "session=${timestamp}-${tag}"
        echo "tag=${tag}"
        echo "started_local=$(date --iso-8601=seconds)"
        echo "hostname=$(hostname)"
        echo "resolution=${resolution}"
        echo "fps=${fps}"
        if [[ "${web_preview}" == 'true' ]]; then
            echo 'compression_mode=LOSSLESS_ZSTD'
            echo "preview_url=http://${preview_host}:${preview_port}"
            echo "preview_fps=${preview_fps}"
            echo "preview_width=${preview_width}"
        else
            echo "compression_mode=${compression_mode}"
        fi
        echo "duration_s=${duration}"
        if (( duration > 0 )); then
            echo "target_frames=$((duration * fps))"
        fi
        if [[ "${web_preview}" == 'true' ]]; then
            echo 'recorder=/home/auv/record_zed_preview.py'
        else
            echo "recorder=${zed_explorer}"
        fi
        echo 'format=SVO2'
        echo 'contains=raw_stereo_camera_stream_and_zed_sensor_metadata'
    } >"${metadata_file}"

    if [[ "${web_preview}" == 'true' ]]; then
        record_command=(
            /usr/bin/python3 /home/auv/record_zed_preview.py
            --output "${svo_file}"
            --duration "${duration}"
            --resolution "${resolution}"
            --fps "${fps}"
            --preview-host "${preview_host}"
            --preview-port "${preview_port}"
            --preview-fps "${preview_fps}"
            --preview-width "${preview_width}"
            --min-free-gb "${min_free_gb}"
        )
    else
        record_command=(
            "${zed_explorer}"
            -r "${resolution}"
            -f "${fps}"
            --cpm "${compression_mode}"
            -o "${svo_file}"
        )
        if (( duration > 0 )); then
            # ZED_Explorer's frame limit counts captured frames, unlike a plain
            # timeout that also consumes the camera initialization time.
            target_frames=$((duration * fps))
            record_command+=(--length "${target_frames}")
        fi
    fi
    if (( duration > 0 )); then
        record_command=(
            timeout --signal=INT --kill-after=20 "$((duration + 30))"
            "${record_command[@]}"
        )
    fi

    # Do not launch this command with shell `&`: a non-interactive shell may
    # make an asynchronous child inherit SIGINT as ignored. `setsid --fork`
    # detaches while preserving the foreground/default signal disposition, so
    # stop can ask ZED_Explorer to finalize the SVO2 index cleanly.
    launcher_pid_file="${session_dir}/recorder.pid"
    setsid --fork bash -c \
        'printf "%s\n" "$$" >"$1"; shift; exec "$@"' \
        record-training-video "${launcher_pid_file}" "${record_command[@]}" \
        >>"${log_file}" 2>&1 </dev/null
    for _ in $(seq 1 20); do
        [[ -s "${launcher_pid_file}" ]] && break
        sleep 0.1
    done
    [[ -s "${launcher_pid_file}" ]] \
        || fail 'detached recorder did not publish its PID'
    recorder_pid="$(sed -n '1p' "${launcher_pid_file}")"
    [[ "${recorder_pid}" =~ ^[0-9]+$ ]] \
        || fail 'detached recorder published an invalid PID'
    if [[ "${web_preview}" == 'true' ]]; then
        # Importing the ZED SDK and OpenCV can take several seconds on Jetson.
        # Do not print the browser URL until the recorder has opened the camera,
        # enabled SVO2 recording and started its HTTP server.
        recorder_ready='false'
        for _ in $(seq 1 150); do
            if grep -q '^READY ' "${log_file}" 2>/dev/null; then
                recorder_ready='true'
                break
            fi
            kill -0 "${recorder_pid}" 2>/dev/null || break
            sleep 0.1
        done
        if [[ "${recorder_ready}" != 'true' ]]; then
            echo 'Preview recorder did not become ready:' >&2
            tail -40 "${log_file}" >&2 || true
            if kill -0 "${recorder_pid}" 2>/dev/null; then
                pgid="$(ps -o pgid= -p "${recorder_pid}" | tr -d ' ')"
                if [[ "${pgid}" == "${recorder_pid}" ]]; then
                    kill -INT -- "-${recorder_pid}" 2>/dev/null || true
                else
                    kill -INT "${recorder_pid}" 2>/dev/null || true
                fi
            fi
            fail 'ZED preview recording did not become ready within 15 seconds'
        fi
    else
        sleep 3
    fi
    if ! kill -0 "${recorder_pid}" 2>/dev/null; then
        echo 'Recorder exited during startup:' >&2
        tail -40 "${log_file}" >&2 || true
        fail 'ZED recording did not start'
    fi

    state_tmp="${active_file}.tmp.$$"
    printf '%s\n%s\n' "${recorder_pid}" "${session_dir}" >"${state_tmp}"
    mv -f -- "${state_tmp}" "${active_file}"
    latest_tmp="${base_dir}/.latest_session.tmp.$$"
    printf '%s\n' "$(basename "${session_dir}")" >"${latest_tmp}"
    mv -f -- "${latest_tmp}" "${base_dir}/.latest_session"

    echo 'ZED training recording started.'
    echo "PID: ${recorder_pid}"
    echo "Session: ${session_dir}"
    echo "SVO2: ${svo_file}"
    if [[ "${web_preview}" == 'true' ]]; then
        echo "Live preview: http://${preview_host}:${preview_port}"
        echo 'Preview mode records lossless ZSTD SVO2 and uses substantially more disk space.'
    fi
    if (( duration > 0 )); then
        echo "Automatic stop: ${duration} seconds"
    else
        echo 'Automatic stop: disabled; run the stop command before power-off'
    fi
    echo 'The process will continue if SSH disconnects.'
}

stop_recording() {
    ensure_base_dir
    clear_stale_state
    read_active_state
    if ! valid_active_state; then
        echo 'Recorder is already stopped.'
        return 0
    fi

    svo_file="${active_session}/raw.svo2"
    cmdline="$(tr '\0' ' ' <"/proc/${active_pid}/cmdline" 2>/dev/null || true)"
    case "${cmdline}" in
        *"${svo_file}"*) ;;
        *) fail "PID ${active_pid} no longer belongs to this recording; refusing to signal it" ;;
    esac

    pgid="$(ps -o pgid= -p "${active_pid}" | tr -d ' ')"
    echo "Stopping recorder PID=${active_pid}, PGID=${pgid:-unknown} ..."
    if [[ "${pgid}" == "${active_pid}" ]]; then
        kill -INT -- "-${active_pid}"
    else
        kill -INT "${active_pid}"
    fi

    for _ in $(seq 1 60); do
        kill -0 "${active_pid}" 2>/dev/null || break
        process_state="$(ps -o stat= -p "${active_pid}" 2>/dev/null || true)"
        [[ "${process_state}" == Z* ]] && break
        sleep 0.5
    done
    process_state="$(ps -o stat= -p "${active_pid}" 2>/dev/null || true)"
    if kill -0 "${active_pid}" 2>/dev/null && [[ "${process_state}" != Z* ]]; then
        fail 'recorder did not stop within 30 seconds; do not cut Jetson power'
    fi

    sync
    echo "stopped_local=$(date --iso-8601=seconds)" \
        >>"${active_session}/metadata.txt"
    rm -f -- "${active_file}"
    echo 'Recorder stopped cleanly.'
    if [[ -e "${svo_file}" ]]; then
        ls -lh "${svo_file}"
    else
        echo 'WARNING: expected SVO2 file is missing.' >&2
        return 1
    fi
    echo "Session: ${active_session}"
}

list_sessions() {
    ensure_base_dir
    find "${base_dir}" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
        | sort -r
}

command_name="${1:-help}"
if [[ $# -gt 0 ]]; then
    shift
fi

case "${command_name}" in
    start) start_recording "$@" ;;
    stop) stop_recording "$@" ;;
    status) show_status "$@" ;;
    list) list_sessions "$@" ;;
    help|-h|--help) usage ;;
    *) fail "unknown command: ${command_name}" ;;
esac
