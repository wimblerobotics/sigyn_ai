#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0
# Pi 5 + Hailo-8 Model Deployment Script for Sigyn Vision
# Usage: ./scripts/deploy_pi5_hailo.sh -m <model_name>
# Example: ./scripts/deploy_pi5_hailo.sh -m fcc4_v4_640
#
# PREREQUISITE: The pi_can_detector repo must already be cloned at:
#   ~/sigyn_vision_ws/src/pi_can_detector/
# Run vcstool first, then run this script to deploy the model.
# After this script succeeds, start the pi-can-detector service:
#   sudo systemctl start pi-can-detector

set -e  # Exit on error

# Color output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
MODEL_NAME=""
REMOTE_HOST="sigynVision"
REMOTE_USER="ros"
REMOTE_PATH="~/sigyn_vision_ws/src/pi_can_detector/models"
DEVICE="pi5_hailo8"

# Parse command line arguments
while getopts "m:h:u:p:d:" opt; do
  case $opt in
    m) MODEL_NAME="$OPTARG" ;;
    h) REMOTE_HOST="$OPTARG" ;;
    u) REMOTE_USER="$OPTARG" ;;
    p) REMOTE_PATH="$OPTARG" ;;
    d) DEVICE="$OPTARG" ;;
    \?) echo "Invalid option -$OPTARG" >&2; exit 1 ;;
  esac
done

# Validate required arguments
if [ -z "$MODEL_NAME" ]; then
    echo -e "${RED}Error: Model name is required${NC}"
    echo "Usage: $0 -m <model_name> [-h remote_host] [-u remote_user] [-p remote_path] [-d device]"
    echo "Example: $0 -m fcc4_v4_640"
    exit 1
fi

echo -e "${GREEN}=== Pi 5 + Hailo-8 Model Deployment ===${NC}"
echo "Model: $MODEL_NAME"
echo "Device: $DEVICE"
echo "Remote: $REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH"
echo ""

# Check if model directory exists
EXPORT_DIR="models/exported/${MODEL_NAME}/${DEVICE}"
if [ ! -d "$EXPORT_DIR" ]; then
    echo -e "${RED}Error: Export directory not found: $EXPORT_DIR${NC}"
    echo "Available exports:"
    ls -la models/exported/ 2>/dev/null || echo "No exports found"
    exit 1
fi

# Check for required files
# Preferred: pre-compiled HEF from local machine
HEF_FILE=$(find "${EXPORT_DIR}" -name "*.hef" -type f | head -n 1)
ONNX_FILE=$(find "${EXPORT_DIR}" -name "*.onnx" -type f | head -n 1)
LABELS_FILE="${EXPORT_DIR}/labels.txt"

if [ -z "$HEF_FILE" ] && [ -z "$ONNX_FILE" ]; then
    echo -e "${RED}Error: No .hef or .onnx file found in: $EXPORT_DIR${NC}"
    echo "Files in directory:"
    ls -la "$EXPORT_DIR"
    exit 1
fi

if [ -n "$HEF_FILE" ] && [ -f "$HEF_FILE" ]; then
    echo "Found HEF file: $(basename "$HEF_FILE")"
else
    echo -e "${YELLOW}Warning: No .hef found, will deploy ONNX only${NC}"
fi

if [ -n "$ONNX_FILE" ] && [ -f "$ONNX_FILE" ]; then
    echo "Found ONNX file: $(basename "$ONNX_FILE")"
fi

if [ ! -f "$LABELS_FILE" ]; then
    echo -e "${YELLOW}Warning: Labels file not found: $LABELS_FILE${NC}"
    echo "Continuing without labels file..."
fi

# Test SSH connection
echo -e "${GREEN}Testing SSH connection...${NC}"
if ! ssh "${REMOTE_USER}@${REMOTE_HOST}" "echo 'Connection successful'"; then
    echo -e "${RED}Error: Cannot connect to ${REMOTE_USER}@${REMOTE_HOST}${NC}"
    exit 1
fi

# Create remote directory if it doesn't exist
echo -e "${GREEN}Ensuring remote directory exists...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p ${REMOTE_PATH}"

# Backup existing model links/files if they exist
echo -e "${GREEN}Backing up existing model (if any)...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "
    cd ${REMOTE_PATH}
    TS=\$(date +%Y%m%d_%H%M%S)
    for LINK in can_detector.hef can_detector.onnx; do
        if [ -L \"\$LINK\" ]; then
            CURRENT_TARGET=\$(readlink \"\$LINK\")
            echo \"Current symlink \$LINK points to: \$CURRENT_TARGET\"
            if [ -f \"\$CURRENT_TARGET\" ]; then
                BACKUP_NAME=\"\${CURRENT_TARGET}.backup.\${TS}\"
                cp \"\$CURRENT_TARGET\" \"\$BACKUP_NAME\"
                echo \"Backed up to: \$BACKUP_NAME\"
            fi
        elif [ -f \"\$LINK\" ]; then
            BACKUP_NAME=\"\${LINK}.backup.\${TS}\"
            mv \"\$LINK\" \"\$BACKUP_NAME\"
            echo \"Backed up to: \$BACKUP_NAME\"
        fi
    done
"

# Copy HEF file (preferred)
if [ -n "$HEF_FILE" ] && [ -f "$HEF_FILE" ]; then
    echo -e "${GREEN}Copying HEF file...${NC}"
    REMOTE_HEF_NAME="${MODEL_NAME}.hef"
    scp "$HEF_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${REMOTE_HEF_NAME}"
fi

# Copy ONNX file (optional fallback/debug)
if [ -n "$ONNX_FILE" ] && [ -f "$ONNX_FILE" ]; then
    echo -e "${GREEN}Copying ONNX file...${NC}"
    REMOTE_ONNX_NAME="${MODEL_NAME}.onnx"
    scp "$ONNX_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${REMOTE_ONNX_NAME}"
fi

# Copy labels file if it exists
if [ -f "$LABELS_FILE" ]; then
    echo -e "${GREEN}Copying labels file...${NC}"
    REMOTE_LABELS_NAME="${MODEL_NAME}_labels.txt"
    scp "$LABELS_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${REMOTE_LABELS_NAME}"
fi

# Create symlinks
echo -e "${GREEN}Creating symlink...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "
    cd ${REMOTE_PATH}
    if [ -n \"${HEF_FILE}\" ] && [ -f \"${REMOTE_HEF_NAME}\" ]; then
        rm -f can_detector.hef
        ln -s ${REMOTE_HEF_NAME} can_detector.hef
        echo 'Symlink created: can_detector.hef -> ${REMOTE_HEF_NAME}'
    fi
    if [ -n \"${ONNX_FILE}\" ] && [ -f \"${REMOTE_ONNX_NAME}\" ]; then
        rm -f can_detector.onnx
        ln -s ${REMOTE_ONNX_NAME} can_detector.onnx
        echo 'Symlink created: can_detector.onnx -> ${REMOTE_ONNX_NAME}'
    fi
"

# Verify deployment
echo -e "${GREEN}Verifying deployment...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "
    cd ${REMOTE_PATH}
    echo 'Files in models directory:'
    ls -lh
    echo ''
    echo 'Symlink details (if present):'
    ls -l can_detector.hef 2>/dev/null || true
    ls -l can_detector.onnx 2>/dev/null || true
"

echo ""
echo -e "${GREEN}=== Deployment Complete ===${NC}"
if [ -n "$HEF_FILE" ] && [ -f "$HEF_FILE" ]; then
    echo "Model deployed: ${REMOTE_HEF_NAME}"
    echo "Active model: can_detector.hef -> ${REMOTE_HEF_NAME}"
else
    echo -e "${YELLOW}No HEF deployed; ONNX-only deployment${NC}"
fi
if [ -n "$ONNX_FILE" ] && [ -f "$ONNX_FILE" ]; then
    echo "ONNX deployed: ${REMOTE_ONNX_NAME}"
fi
echo ""
echo "Next steps:"
echo "1. SSH to robot: ssh ${REMOTE_USER}@${REMOTE_HOST}"
if [ -n "$HEF_FILE" ] && [ -f "$HEF_FILE" ]; then
    echo "2. Launch/test your ROS 2 detector using can_detector.hef"
    echo "3. If needed, rollback with: cd ${REMOTE_PATH} && ln -sf <backup_file> can_detector.hef"
else
    echo "2. Compile ONNX to HEF on the target machine"
    echo "3. Test the model in your ROS 2 workspace"
    echo "4. If needed, rollback with: cd ${REMOTE_PATH} && ln -sf <backup_file> can_detector.onnx"
fi
