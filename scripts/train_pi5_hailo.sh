#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
#
# Complete training pipeline for Pi 5 + Hailo-8
# Downloads dataset, trains model, exports ONNX, compiles to HEF

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

PROJECT_NAME="fcc4"
IMAGE_SIZE=512
MODEL_SIZE="n"
EPOCHS=100
DOCKER_IMAGE="hailo8_ai_sw_suite_2025-10:1"
TRY_640=0

# Parse arguments
VERSION=""
RUN_NAME=""

usage() {
  echo "Usage: $0 -v VERSION [-n RUN_NAME] [-p PROJECT] [-s IMG_SIZE] [-e EPOCHS] [-m MODEL_SIZE] [-d DOCKER_IMAGE] [--try-640]"
    echo ""
    echo "Options:"
  echo "  -v VERSION       RoboFlow dataset version number (required)"
  echo "  -n RUN_NAME      Custom run name (default: pi5_hailo_v<VERSION>)"
  echo "  -p PROJECT       RoboFlow project ID (default: fcc4)"
  echo "  -s IMG_SIZE      Training/export image size (default: 512)"
  echo "  -e EPOCHS        Training epochs (default: 100)"
  echo "  -m MODEL_SIZE    YOLO model size: n,s,m,l,x (default: n)"
  echo "  -d DOCKER_IMAGE  Hailo Docker image (default: hailo8_ai_sw_suite_2025-10:1)"
  echo "  --try-640        Try 640 compile first; fallback to 512 if compile fails"
    echo ""
    echo "Example:"
  echo "  $0 -v 4 -n pi_can_v4a"
  echo "  $0 -v 4 -n pi_can_v4a --try-640"
    exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -v) VERSION="$2"; shift 2 ;;
    -n) RUN_NAME="$2"; shift 2 ;;
    -p) PROJECT_NAME="$2"; shift 2 ;;
    -s) IMAGE_SIZE="$2"; shift 2 ;;
    -e) EPOCHS="$2"; shift 2 ;;
    -m) MODEL_SIZE="$2"; shift 2 ;;
    -d) DOCKER_IMAGE="$2"; shift 2 ;;
    --try-640) TRY_640=1; IMAGE_SIZE=640; shift ;;
    -h|--help) usage ;;
    *)
      echo -e "${RED}❌ Unknown option: $1${NC}"
      usage
      ;;
    esac
done

if [ -z "$VERSION" ]; then
    echo -e "${RED}❌ Error: Version number required${NC}"
    usage
fi

if [ -z "$RUN_NAME" ]; then
    RUN_NAME="pi5_hailo_v${VERSION}"
fi

if [[ "$MODEL_SIZE" != "n" && "$MODEL_SIZE" != "s" && "$MODEL_SIZE" != "m" && "$MODEL_SIZE" != "l" && "$MODEL_SIZE" != "x" ]]; then
  echo -e "${RED}❌ Error: MODEL_SIZE must be one of: n, s, m, l, x${NC}"
  exit 1
fi

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Pi 5 + Hailo-8 Training Pipeline${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "  Project: ${PROJECT_NAME}"
echo -e "  Version: ${VERSION}"
echo -e "  Run Name: ${RUN_NAME}"
echo -e "  Image Size: ${IMAGE_SIZE}x${IMAGE_SIZE}"
echo -e "  Model: YOLOv8${MODEL_SIZE}"
echo -e "  Docker Image: ${DOCKER_IMAGE}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""

# Step 1: Download dataset
echo -e "${YELLOW}📥 Step 1/4: Downloading dataset from RoboFlow...${NC}"
python src/utils/roboflow_download.py \
    --project "$PROJECT_NAME" \
    --version "$VERSION" \
    --format yolov8

DATASET_PATH="datasets/roboflow_exports/${PROJECT_NAME}.v${VERSION}-yolov8"

if [ ! -f "${DATASET_PATH}/data.yaml" ]; then
  DATASET_PATH=$(find datasets/roboflow_exports -maxdepth 1 -mindepth 1 -type d -iname "${PROJECT_NAME}.v${VERSION}-yolov8" | head -n 1)
fi

if [ -z "$DATASET_PATH" ] || [ ! -f "${DATASET_PATH}/data.yaml" ]; then
  echo -e "${RED}❌ Dataset download failed: data.yaml not found${NC}"
  echo "Found dataset directories:"
  ls -1 datasets/roboflow_exports 2>/dev/null || true
  exit 1
fi

echo -e "${GREEN}✅ Dataset downloaded to: ${DATASET_PATH}${NC}"
echo ""

# Step 2: Create training configuration
echo -e "${YELLOW}📝 Step 2/4: Creating training configuration...${NC}"

CONFIG_PATH="configs/training/${RUN_NAME}.yaml"

cat > "$CONFIG_PATH" << EOF
# Auto-generated configuration for Pi 5 + Hailo-8
# Generated: $(date)
# Dataset version: ${VERSION}

model:
  size: "${MODEL_SIZE}"

dataset:
  path: "${DATASET_PATH}"

compute:
  device: "auto"
  batch_size: "auto"
  workers: "auto"
  enable_amp: true

training:
  epochs: ${EPOCHS}
  imgsz: ${IMAGE_SIZE}
  optimizer: "auto"
  patience: 50

output:
  project_dir: "models/checkpoints"
  run_name: "${RUN_NAME}"
EOF

echo -e "${GREEN}✅ Configuration created: ${CONFIG_PATH}${NC}"
echo ""

# Step 3: Train model
echo -e "${YELLOW}🚀 Step 3/4: Training model...${NC}"
python src/training/train.py --config "$CONFIG_PATH"

# Find trained model
TRAINED_MODEL=""
for CANDIDATE in \
  "runs/detect/models/checkpoints/${RUN_NAME}/weights/best.pt" \
  "models/checkpoints/${RUN_NAME}/weights/best.pt"; do
  if [ -f "$CANDIDATE" ]; then
    TRAINED_MODEL="$CANDIDATE"
    break
  fi
done

if [ -z "$TRAINED_MODEL" ]; then
  TRAINED_MODEL=$(find . -path "*/${RUN_NAME}/weights/best.pt" -type f | head -n 1)
fi

if [ -z "$TRAINED_MODEL" ] || [ ! -f "$TRAINED_MODEL" ]; then
    echo -e "${RED}❌ Training failed: best.pt not found${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Model trained successfully: ${TRAINED_MODEL}${NC}"
echo ""

# Step 4: Export + compile
echo -e "${YELLOW}📦 Step 4/4: Exporting and compiling for Hailo...${NC}"

export_and_compile() {
  local SIZE="$1"
  local MODEL_DIR_NAME="$RUN_NAME"

  if [ "$SIZE" != "$IMAGE_SIZE" ]; then
    MODEL_DIR_NAME="${RUN_NAME}_${SIZE}"
  fi

  local EXPORT_DIR="models/exported/${MODEL_DIR_NAME}/pi5_hailo8"

  echo -e "${BLUE}→ Exporting ONNX at ${SIZE}x${SIZE} ...${NC}"
  python src/export/export.py \
    --model "$TRAINED_MODEL" \
    --device pi5_hailo8 \
    --imgsz "$SIZE" \
    --output "$EXPORT_DIR"

  local ONNX_PATH="${EXPORT_DIR}/best.onnx"
  if [ ! -f "$ONNX_PATH" ]; then
    echo -e "${RED}❌ ONNX export failed: ${ONNX_PATH} not found${NC}"
    return 1
  fi

  echo -e "${BLUE}→ Compiling HEF from ${ONNX_PATH} ...${NC}"
  if python docker/run_hailo_compile.py --onnx "$ONNX_PATH" --docker-image "$DOCKER_IMAGE"; then
    echo -e "${GREEN}✅ HEF compile succeeded at ${SIZE}x${SIZE}${NC}"
    echo -e "${GREEN}   Export dir: ${EXPORT_DIR}${NC}"
    return 0
  else
    echo -e "${YELLOW}⚠️  HEF compile failed at ${SIZE}x${SIZE}${NC}"
    return 1
  fi
}

if [ "$TRY_640" -eq 1 ]; then
  echo -e "${YELLOW}Trying 640 first, then auto-fallback to 512 on compile failure...${NC}"
  if ! export_and_compile 640; then
    echo -e "${YELLOW}Falling back to 512...${NC}"
    export_and_compile 512 || {
      echo -e "${RED}❌ Compile failed for both 640 and 512${NC}"
      exit 1
    }
  fi
else
  export_and_compile "$IMAGE_SIZE" || exit 1
fi

# Summary
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Pi 5 + Hailo-8 Pipeline Complete!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "Trained model: ${TRAINED_MODEL}"
echo -e ""
echo -e "${YELLOW}Next step:${NC}"
echo -e "  ./scripts/deploy_pi5_hailo.sh -m ${RUN_NAME}"
echo ""
