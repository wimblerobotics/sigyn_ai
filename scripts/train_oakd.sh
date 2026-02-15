#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
#
# Complete training pipeline for OAK-D Lite
# Downloads dataset, trains model, exports to .blob

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_NAME="fcc4"
IMAGE_SIZE=416  # OAK-D works best with 416x416
MODEL_SIZE="5n"  # YOLOv5 nano for OAK-D blob compatibility
EPOCHS=100

# Parse arguments
VERSION=""
RUN_NAME=""

usage() {
    echo "Usage: $0 -v VERSION [-n RUN_NAME]"
    echo ""
    echo "Options:"
    echo "  -v VERSION    RoboFlow dataset version number (required)"
    echo "  -n RUN_NAME   Custom run name (optional, default: oakd_v{VERSION})"
    echo ""
    echo "Example:"
    echo "  $0 -v 5"
    echo "  $0 -v 5 -n my_oakd_detector"
    echo ""
    echo "Note: Make sure your RoboFlow dataset version uses 416x416 resize!"
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
    RUN_NAME="oakd_v${VERSION}"
fi

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  OAK-D Lite Training Pipeline${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "  Project: ${PROJECT_NAME}"
echo -e "  Version: ${VERSION}"
echo -e "  Run Name: ${RUN_NAME}"
echo -e "  Image Size: ${IMAGE_SIZE}x${IMAGE_SIZE}"
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
# Auto-generated configuration for OAK-D Lite
# Generated: $(date)
# Dataset version: ${VERSION}
# Using YOLOv5 for OAK-D blob compatibility

model:
  size: "${MODEL_SIZE}"  # YOLOv5 nano for OAK-D

dataset:
  path: "${DATASET_PATH}"

compute:
  device: "auto"  # Auto-detect GPU/CPU
  batch_size: "auto"  # Auto-detect based on GPU
  workers: "auto"

training:
  epochs: ${EPOCHS}
  imgsz: ${IMAGE_SIZE}  # OAK-D requires 416x416

output:
  project_dir: "models/checkpoints"
  run_name: "${RUN_NAME}"
EOF

echo -e "${GREEN}✅ Configuration created: ${CONFIG_PATH}${NC}"
echo ""

# Step 3: Train model
echo -e "${YELLOW}🚀 Step 3/4: Training model...${NC}"
echo -e "   This may take 20-60 minutes depending on your GPU"
echo ""

python src/training/train.py --config "$CONFIG_PATH"

# Find the trained model (support multiple Ultralytics output layouts)
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
    TRAINED_MODEL=$(find . -path "*/${RUN_NAME}/weights/best.pt" -type f | head -1)
fi

if [ ! -f "$TRAINED_MODEL" ]; then
    echo -e "${RED}❌ Training failed: best.pt not found${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Model trained successfully: ${TRAINED_MODEL}${NC}"
echo ""

# Step 4: Export for OAK-D with blob compilation
echo -e "${YELLOW}📦 Step 4/4: Exporting and compiling model for OAK-D...${NC}"
echo -e "   Note: Blob compilation requires internet connection"
echo ""

EXPORT_DIR="models/exported/${RUN_NAME}/oakd_lite"

python src/export/export.py \
    --model "$TRAINED_MODEL" \
    --device oakd_lite \
    --imgsz 416 \
    --output "$EXPORT_DIR" \
    --compile

if [ ! -f "${EXPORT_DIR}/best.blob" ]; then
    echo -e "${YELLOW}⚠️  Blob compilation may have failed, but ONNX export should be available${NC}"
fi

echo -e "${GREEN}✅ Model exported to: ${EXPORT_DIR}${NC}"
echo ""

# Summary
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ OAK-D Lite Pipeline Complete!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "📁 Artifacts:"
echo -e "   Trained Model:  ${TRAINED_MODEL}"
echo -e "   ONNX Export:    ${EXPORT_DIR}/best.onnx"

if [ -f "${EXPORT_DIR}/best.blob" ]; then
    echo -e "   Blob File:      ${EXPORT_DIR}/best.blob"
fi

echo -e "   Labels:         ${EXPORT_DIR}/labels.txt"
echo ""
echo -e "📊 Training Results:"
echo -e "   Results CSV:    runs/detect/models/checkpoints/${RUN_NAME}/results.csv"
echo -e "   Training Plots: runs/detect/models/checkpoints/${RUN_NAME}/"
echo ""
echo -e "${YELLOW}⚠️  Next Steps:${NC}"

if [ ! -f "${EXPORT_DIR}/best.blob" ]; then
    echo -e "   1. If blob compilation failed, re-run export with --compile"
    echo -e "   2. Deploy with: ./scripts/deploy_oakd.sh -m ${RUN_NAME}"
else
    echo -e "   1. Deploy with: ./scripts/deploy_oakd.sh -m ${RUN_NAME}"
fi
echo ""
