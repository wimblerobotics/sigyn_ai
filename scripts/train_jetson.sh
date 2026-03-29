#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
#
# ============================================================
# NOT YET TESTED — Jetson Orin Nano is not currently deployed
# in the robot. This script has not been run on real hardware.
# Use train_pi5_hailo.sh or train_oakd.sh for tested pipelines.
# ============================================================
#
# Complete training pipeline for Jetson Orin Nano
# Downloads dataset, trains model, exports to ONNX (TensorRT compilation on device)

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_NAME="fcc4"
IMAGE_SIZE=640
MODEL_SIZE="s"  # small model - Jetson can handle it
EPOCHS=100

# Parse arguments
VERSION=""
RUN_NAME=""

usage() {
    echo "Usage: $0 -v VERSION [-n RUN_NAME]"
    echo ""
    echo "Options:"
    echo "  -v VERSION    RoboFlow dataset version number (required)"
    echo "  -n RUN_NAME   Custom run name (optional, default: jetson_v{VERSION})"
    echo ""
    echo "Example:"
    echo "  $0 -v 4"
    echo "  $0 -v 4 -n my_jetson_detector"
    exit 1
}

while getopts "v:n:h" opt; do
    case $opt in
        v) VERSION="$OPTARG" ;;
        n) RUN_NAME="$OPTARG" ;;
        h) usage ;;
        *) usage ;;
    esac
done

if [ -z "$VERSION" ]; then
    echo -e "${RED}❌ Error: Version number required${NC}"
    usage
fi

if [ -z "$RUN_NAME" ]; then
    RUN_NAME="jetson_v${VERSION}"
fi

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Jetson Orin Nano Training Pipeline${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "  Project: ${PROJECT_NAME}"
echo -e "  Version: ${VERSION}"
echo -e "  Run Name: ${RUN_NAME}"
echo -e "  Image Size: ${IMAGE_SIZE}x${IMAGE_SIZE}"
echo -e "  Model: YOLOv8${MODEL_SIZE}"
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
    echo -e "${RED}❌ Dataset download failed: data.yaml not found${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Dataset downloaded to: ${DATASET_PATH}${NC}"
echo ""

# Step 2: Create training configuration
echo -e "${YELLOW}📝 Step 2/4: Creating training configuration...${NC}"

CONFIG_PATH="configs/training/${RUN_NAME}.yaml"

cat > "$CONFIG_PATH" << EOF
# Auto-generated configuration for Jetson Orin Nano
# Generated: $(date)
# Dataset version: ${VERSION}

model:
  size: "${MODEL_SIZE}"  # small model - Jetson can handle more complexity

dataset:
  path: "${DATASET_PATH}"

compute:
  device: "auto"  # Auto-detect GPU/CPU
  batch_size: "auto"  # Auto-detect based on GPU
  workers: "auto"

training:
  epochs: ${EPOCHS}
  imgsz: ${IMAGE_SIZE}  # Jetson works well with 640x640

output:
  project_dir: "models/checkpoints"
  run_name: "${RUN_NAME}"
EOF

echo -e "${GREEN}✅ Configuration created: ${CONFIG_PATH}${NC}"
echo ""

# Step 3: Train model
echo -e "${YELLOW}🚀 Step 3/4: Training model...${NC}"
echo -e "   This may take 30-90 minutes depending on your GPU"
echo ""

python src/training/train.py --config "$CONFIG_PATH"

# Find the trained model
TRAINED_MODEL=$(find runs/detect -path "*/models/checkpoints/${RUN_NAME}/weights/best.pt" | head -1)

if [ ! -f "$TRAINED_MODEL" ]; then
    echo -e "${RED}❌ Training failed: best.pt not found${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Model trained successfully: ${TRAINED_MODEL}${NC}"
echo ""

# Step 4: Export for Jetson
echo -e "${YELLOW}📦 Step 4/4: Exporting model for Jetson Orin Nano...${NC}"

EXPORT_DIR="models/exported/${RUN_NAME}/jetson_orin_nano"

python src/export/export.py \
    --model "$TRAINED_MODEL" \
    --device jetson_orin_nano \
    --imgsz 640 \
    --output "$EXPORT_DIR"

if [ ! -f "${EXPORT_DIR}/best.onnx" ]; then
    echo -e "${RED}❌ Export failed: ONNX file not found${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Model exported to: ${EXPORT_DIR}${NC}"
echo ""

# Summary
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Jetson Orin Nano Pipeline Complete!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "📁 Artifacts:"
echo -e "   Trained Model:  ${TRAINED_MODEL}"
echo -e "   ONNX Export:    ${EXPORT_DIR}/best.onnx"
echo -e "   Labels:         ${EXPORT_DIR}/labels.txt"
echo ""
echo -e "📊 Training Results:"
echo -e "   Results CSV:    runs/detect/models/checkpoints/${RUN_NAME}/results.csv"
echo -e "   Training Plots: runs/detect/models/checkpoints/${RUN_NAME}/"
echo ""
echo -e "${YELLOW}⚠️  Next Steps:${NC}"
echo -e "   1. Copy files to Jetson:"
echo -e "      scp ${EXPORT_DIR}/* ros@jetson.local:~/"
echo -e "   2. Compile to TensorRT on Jetson device"
echo -e "   3. Deploy (deployment script coming soon)"
echo ""
