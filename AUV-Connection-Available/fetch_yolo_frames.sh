#!/usr/bin/env bash
# Extract left-eye YOLO training frames on Jetson and stream them to this host.

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
jetson_host="${JETSON_HOST:-auv@192.168.137.1}"
remote_base="${AUV_ZED_DATASET_DIR:-/home/auv/zed_datasets}"
local_base="${AUV_YOLO_FRAMES_DIR:-${script_dir}/yolo_frames}"
session='latest'
every_n='10'
jpeg_quality='95'

usage() {
    cat <<'EOF'
Usage: fetch_yolo_frames.sh [options]

Options:
  --session NAME|latest  Recording session (default: latest)
  --every-n N            Save one image every N frames (default: 10)
  --jpeg-quality N       JPEG quality 1-100 (default: 95)
  --output DIR           Local dataset root (default: ./yolo_frames beside script)
  -h, --help             Show this help

Examples:
  ./fetch_yolo_frames.sh
  ./fetch_yolo_frames.sh --every-n 15
  ./fetch_yolo_frames.sh --session 20260926-171500-duration_check
EOF
}

fail() {
    echo "FAILED: $*" >&2
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --session) session="${2:?missing session name}"; shift 2 ;;
        --every-n) every_n="${2:?missing frame interval}"; shift 2 ;;
        --jpeg-quality) jpeg_quality="${2:?missing JPEG quality}"; shift 2 ;;
        --output) local_base="${2:?missing output directory}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) fail "unknown option: $1" ;;
    esac
done

[[ "${every_n}" =~ ^[0-9]+$ ]] && (( every_n >= 1 && every_n <= 10000 )) \
    || fail '--every-n must be between 1 and 10000'
[[ "${jpeg_quality}" =~ ^[0-9]+$ ]] \
    && (( jpeg_quality >= 1 && jpeg_quality <= 100 )) \
    || fail '--jpeg-quality must be between 1 and 100'
[[ "${remote_base}" =~ ^/[A-Za-z0-9_./-]+$ ]] \
    || fail 'remote dataset path contains unsupported characters'

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

config_name="n${every_n}_q${jpeg_quality}"
remote_session="${remote_base}/${session}"
remote_svo="${remote_session}/raw.svo2"
remote_frames="${remote_session}/yolo_frames_${config_name}"
local_parent="${local_base}/${session}"
local_target="${local_parent}/${config_name}"

echo "Selected session: ${session}"
echo "Sampling: every ${every_n} frames, JPEG quality ${jpeg_quality}"
ssh "${ssh_options[@]}" "${jetson_host}" \
    "test -s '${remote_svo}'" \
    || fail "remote SVO2 is missing or empty: ${remote_svo}"

ssh "${ssh_options[@]}" "${jetson_host}" \
    "/usr/bin/python3 /home/auv/extract_svo_frames.py '${remote_svo}' --output '${remote_frames}' --every-n '${every_n}' --jpeg-quality '${jpeg_quality}'"

if [[ -s "${local_target}/.complete" ]]; then
    image_count="$(find "${local_target}" -maxdepth 1 -type f -name 'frame_*.jpg' | wc -l)"
    echo "Already downloaded: ${local_target}"
    echo "Images: ${image_count}"
    exit 0
fi

mkdir -p "${local_parent}"
if [[ -e "${local_target}" ]]; then
    backup_path="${local_target}.incomplete-$(date +%Y%m%d-%H%M%S)"
    mv -- "${local_target}" "${backup_path}"
    echo "Moved incomplete local output to: ${backup_path}" >&2
fi

local_partial="$(mktemp -d "${local_target}.partial.XXXXXX")"
cleanup_partial() {
    if [[ -n "${local_partial:-}" && -d "${local_partial}" ]]; then
        rm -rf -- "${local_partial}"
    fi
}
trap cleanup_partial EXIT INT TERM

echo 'Streaming extracted JPEG files to this computer...'
ssh "${ssh_options[@]}" "${jetson_host}" \
    "test -s '${remote_frames}/.complete' && tar -C '${remote_frames}' -cf - ." \
    | tar -C "${local_partial}" -xf -

[[ -s "${local_partial}/.complete" ]] \
    || fail 'downloaded frame set has no completion marker'
image_count="$(find "${local_partial}" -maxdepth 1 -type f -name 'frame_*.jpg' | wc -l)"
(( image_count > 0 )) || fail 'downloaded frame set contains no JPEG images'

mv -- "${local_partial}" "${local_target}"
local_partial=''
trap - EXIT INT TERM

echo "Downloaded: ${local_target}"
echo "Images: ${image_count}"
echo "Manifest: ${local_target}/frames.csv"
