#!/usr/bin/env bash
#
# Download the YOLO-FastestV2 NCNN model for human_detector_yolo.
#
# Produces, under <package>/models/:
#   yolo-fastestv2-opt.param
#   yolo-fastestv2-opt.bin   (~0.5 MB)
#
# Run once on the source checkout, then rebuild:
#   bash src/human_detector_yolo/scripts/download_model.sh
#   pip install --no-deps ncnn
#   colcon build --packages-select human_detector_yolo
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$PKG_DIR/models"
mkdir -p "$MODELS_DIR"

BASE="https://raw.githubusercontent.com/dog-qiuqiu/Yolo-FastestV2/main/sample/ncnn/model"

echo ">> downloading YOLO-FastestV2 (~0.5 MB)"
curl -L --fail -o "$MODELS_DIR/yolo-fastestv2-opt.param" "$BASE/yolo-fastestv2-opt.param"
curl -L --fail -o "$MODELS_DIR/yolo-fastestv2-opt.bin"   "$BASE/yolo-fastestv2-opt.bin"

echo ">> done:"
echo "   $MODELS_DIR/yolo-fastestv2-opt.param"
echo "   $MODELS_DIR/yolo-fastestv2-opt.bin"
echo ""
echo "Now:  pip install --no-deps ncnn"
echo "      colcon build --packages-select human_detector_yolo"
