#!/usr/bin/env bash
# Recreate ~/yolo_env for Python 3.12 inside the Jazzy container.
# Ultralytics + PyTorch CPU; numpy<2 for OpenCV compatibility.

set -euo pipefail

YOLO_ENV="${YOLO_ENV:-/root/yolo_env}"

python3 --version

rm -rf "${YOLO_ENV}"
python3 -m venv "${YOLO_ENV}"
# shellcheck disable=SC1091
source "${YOLO_ENV}/bin/activate"

pip install --upgrade pip wheel
pip install "numpy<2" "opencv-python-headless>=4.8,<4.11"
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
# Keep OpenCV 4.x + numpy<2 (OpenCV 5 breaks cv_bridge conversions).
pip install "numpy<2" "opencv-python-headless>=4.8,<4.11" --force-reinstall

echo ""
echo "YOLO venv ready at ${YOLO_ENV}"
echo "Activate: source ${YOLO_ENV}/bin/activate"
