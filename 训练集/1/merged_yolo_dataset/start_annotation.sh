#!/usr/bin/env bash
set -euo pipefail

dataset_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${dataset_root}/../../.." && pwd)"
annotator="${workspace_root}/AUV-Connection-Available/label_yolo_dataset.py"

if [[ ! -f "${annotator}" ]]; then
    echo "ERROR: annotator not found: ${annotator}" >&2
    exit 1
fi
if [[ ! -x /usr/bin/python3 ]]; then
    echo 'ERROR: /usr/bin/python3 is unavailable.' >&2
    exit 1
fi
if ! /usr/bin/python3 -c 'import cv2' >/dev/null 2>&1; then
    echo 'ERROR: system Python cannot import cv2.' >&2
    echo 'Install the Ubuntu python3-opencv package before labeling.' >&2
    exit 1
fi

exec /usr/bin/python3 "${annotator}" --root "${dataset_root}" "$@"
