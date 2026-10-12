#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./start_annotation_batch.sh test1 [--check-only|--include-reviewed]
  ./start_annotation_batch.sh test2 [--check-only|--include-reviewed]
  ./start_annotation_batch.sh test3 [--check-only|--include-reviewed]

Examples:
  ./start_annotation_batch.sh test1
  ./start_annotation_batch.sh test2 --check-only
  ./start_annotation_batch.sh test3 --include-reviewed
EOF
}

if [[ $# -lt 1 ]]; then
    usage >&2
    exit 2
fi

batch="$1"
shift
batch_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${batch_root}/../.." && pwd)"
annotator="${workspace_root}/AUV-Connection-Available/label_yolo_dataset.py"
classes_file="${batch_root}/merged_yolo_dataset/classes.json"

case "${batch}" in
    test1)
        images_dir="${batch_root}/yolo_frames_n10_q95"
        ;;
    test2)
        images_dir="${batch_root}/yolo_frames_n10_q95-2/yolo_frames_n10_q95"
        ;;
    test3)
        images_dir="${batch_root}/yolo_frames_n10_q95-3/yolo_frames_n10_q95"
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        echo "ERROR: unknown batch: ${batch}" >&2
        usage >&2
        exit 2
        ;;
esac

labels_dir="${images_dir}/labels"

if [[ ! -f "${annotator}" ]]; then
    echo "ERROR: annotator not found: ${annotator}" >&2
    exit 1
fi
if [[ ! -d "${images_dir}" ]]; then
    echo "ERROR: images directory not found: ${images_dir}" >&2
    exit 1
fi
if [[ ! -f "${classes_file}" ]]; then
    echo "ERROR: classes file not found: ${classes_file}" >&2
    exit 1
fi
if ! /usr/bin/python3 -c 'import cv2' >/dev/null 2>&1; then
    echo 'ERROR: system Python cannot import cv2.' >&2
    exit 1
fi

echo "BATCH=${batch}"
exec /usr/bin/python3 "${annotator}" \
    --root "${batch_root}" \
    --images-dir "${images_dir}" \
    --labels-dir "${labels_dir}" \
    --classes-file "${classes_file}" \
    "$@"
