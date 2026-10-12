#!/usr/bin/env bash
# Export a Jetson SVO2 to an ordinary MP4, then download it to this computer.

set -eo pipefail

jetson_host="${JETSON_HOST:-auv@192.168.137.1}"
remote_base="${AUV_ZED_DATASET_DIR:-/home/auv/zed_datasets}"
local_base="${AUV_ZED_DOWNLOAD_DIR:-${PWD}/zed_recordings}"
session='latest'
with_svo=0

usage() {
    cat <<'EOF'
Usage: fetch_zed_recording.sh [--session NAME|latest] [--output DIR] [--with-svo]

Environment:
  JETSON_HOST             SSH target (default: auv@192.168.137.1)
  AUV_ZED_DATASET_DIR     Remote dataset root (default: /home/auv/zed_datasets)
  AUV_ZED_DOWNLOAD_DIR    Local download root (default: ./zed_recordings)

Examples:
  ./fetch_zed_recording.sh
  ./fetch_zed_recording.sh --session 20260926-163740-final_default
  ./fetch_zed_recording.sh --with-svo
EOF
}

fail() {
    echo "FAILED: $*" >&2
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --session) session="${2:?missing session name}"; shift 2 ;;
        --output) local_base="${2:?missing output directory}"; shift 2 ;;
        --with-svo) with_svo=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unknown option: $1" ;;
    esac
done

ssh_options=(
    -o ConnectTimeout=8
    -o StrictHostKeyChecking=accept-new
    -o ControlMaster=auto
    -o ControlPersist=60
    -o 'ControlPath=/tmp/auv-zed-%C'
)

if [[ "${session}" == 'latest' ]]; then
    session="$(
        ssh "${ssh_options[@]}" "${jetson_host}" \
            "if test -s '${remote_base}/.latest_session'; then cat '${remote_base}/.latest_session'; else find '${remote_base}' -mindepth 1 -maxdepth 1 -type d -printf '%f\\n' | sort -r | head -1; fi"
    )"
fi

[[ "${session}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$ ]] \
    || fail "invalid or missing session name: ${session:-<empty>}"

remote_session="${remote_base}/${session}"
remote_svo="${remote_session}/raw.svo2"
remote_video="${remote_session}/preview.mp4"
remote_complete="${remote_session}/preview.mp4.complete"
local_session="${local_base}/${session}"

echo "Selected session: ${session}"
ssh "${ssh_options[@]}" "${jetson_host}" \
    "test -s '${remote_svo}'" \
    || fail "remote SVO2 is missing or empty: ${remote_svo}"

if ! ssh "${ssh_options[@]}" "${jetson_host}" \
        "test -s '${remote_video}' && test -s '${remote_complete}'"; then
    echo 'Creating MP4 preview on Jetson...'
    ssh "${ssh_options[@]}" "${jetson_host}" \
        "/usr/bin/python3 /home/auv/export_svo.py '${remote_svo}' --video '${remote_video}' --force"
fi

mkdir -p "${local_session}"
echo 'Downloading preview and metadata...'
scp "${ssh_options[@]}" \
    "${jetson_host}:${remote_video}" \
    "${local_session}/preview.mp4"
scp "${ssh_options[@]}" \
    "${jetson_host}:${remote_session}/metadata.txt" \
    "${local_session}/metadata.txt"

if (( with_svo )); then
    echo 'Downloading the large original SVO2...'
    scp "${ssh_options[@]}" \
        "${jetson_host}:${remote_svo}" \
        "${local_session}/raw.svo2"
fi

echo "Downloaded: ${local_session}/preview.mp4"
echo "Play with:  vlc '${local_session}/preview.mp4'"
