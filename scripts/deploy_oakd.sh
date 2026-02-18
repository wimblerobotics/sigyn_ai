#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0
# OAK-D Model + Node Deployment Script for Sigyn Robot
# Usage: ./scripts/deploy_oakd.sh -m <model_name>
# Example: ./scripts/deploy_oakd.sh -m oakd_v4

set -e  # Exit on error

# Color output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
MODEL_NAME=""
REMOTE_HOST="sigyn7900a"
REMOTE_USER="ros"
REMOTE_PATH="~/sigyn_ws/src/Sigyn/yolo_oakd_test/models"
DEVICE="oakd_lite"
DEPLOY_NODE=1
LOCAL_NODE_SOURCE="scripts/oakd_can_detector_ultralytics.py"
REMOTE_NODE_PATH="~/sigyn_ws/src/Sigyn/yolo_oakd_test/yolo_oakd_test/oakd_can_detector.py"

# Parse command line arguments
while getopts "m:h:u:p:d:s" opt; do
  case $opt in
    m) MODEL_NAME="$OPTARG" ;;
    h) REMOTE_HOST="$OPTARG" ;;
    u) REMOTE_USER="$OPTARG" ;;
    p) REMOTE_PATH="$OPTARG" ;;
    d) DEVICE="$OPTARG" ;;
        s) DEPLOY_NODE=0 ;;
    \?) echo "Invalid option -$OPTARG" >&2; exit 1 ;;
  esac
done

# Validate required arguments
if [ -z "$MODEL_NAME" ]; then
    echo -e "${RED}Error: Model name is required${NC}"
    echo "Usage: $0 -m <model_name> [-h remote_host] [-u remote_user] [-p remote_path] [-d device] [-s]"
    echo "Example: $0 -m oakd_v4"
    echo "  -s: Skip deploying node script (deploy blob/labels only)"
    exit 1
fi

echo -e "${GREEN}=== OAK-D Model Deployment ===${NC}"
echo "Model: $MODEL_NAME"
echo "Device: $DEVICE"
echo "Remote: $REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH"
if [ "$DEPLOY_NODE" -eq 1 ]; then
    echo "Node source: $LOCAL_NODE_SOURCE"
fi
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
# OAK-D export creates files with OpenVINO version suffix like: best_openvino_2021.4_6shave.blob
BLOB_FILE=$(find "${EXPORT_DIR}" -name "*.blob" -type f | head -n 1)
LABELS_FILE="${EXPORT_DIR}/labels.txt"

if [ -z "$BLOB_FILE" ] || [ ! -f "$BLOB_FILE" ]; then
    echo -e "${RED}Error: No .blob file found in: $EXPORT_DIR${NC}"
    echo "Files in directory:"
    ls -la "$EXPORT_DIR"
    exit 1
fi

echo "Found blob file: $(basename "$BLOB_FILE")"

if [ "$DEPLOY_NODE" -eq 1 ] && [ ! -f "$LOCAL_NODE_SOURCE" ]; then
    echo -e "${RED}Error: Node source file not found: $LOCAL_NODE_SOURCE${NC}"
    exit 1
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

# Backup existing can_detector.blob if it exists
echo -e "${GREEN}Backing up existing model (if any)...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "
    cd ${REMOTE_PATH}
    if [ -L can_detector.blob ]; then
        CURRENT_TARGET=\$(readlink can_detector.blob)
        echo 'Current symlink points to: '\$CURRENT_TARGET
        if [ -f \"\$CURRENT_TARGET\" ]; then
            BACKUP_NAME=\"\${CURRENT_TARGET}.backup.\$(date +%Y%m%d_%H%M%S)\"
            cp \"\$CURRENT_TARGET\" \"\$BACKUP_NAME\"
            echo \"Backed up to: \$BACKUP_NAME\"
        fi
    elif [ -f can_detector.blob ]; then
        BACKUP_NAME=\"can_detector.blob.backup.\$(date +%Y%m%d_%H%M%S)\"
        mv can_detector.blob \"\$BACKUP_NAME\"
        echo \"Backed up to: \$BACKUP_NAME\"
    fi
"

# Copy blob file
echo -e "${GREEN}Copying blob file...${NC}"
REMOTE_BLOB_NAME="${MODEL_NAME}.blob"
scp "$BLOB_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${REMOTE_BLOB_NAME}"

# Copy labels file if it exists
if [ -f "$LABELS_FILE" ]; then
    echo -e "${GREEN}Copying labels file...${NC}"
    REMOTE_LABELS_NAME="${MODEL_NAME}_labels.txt"
    scp "$LABELS_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${REMOTE_LABELS_NAME}"
fi

# Deploy node script (recommended for Ultralytics host-decoding pipeline)
if [ "$DEPLOY_NODE" -eq 1 ]; then
    echo -e "${GREEN}Deploying OAK-D node script...${NC}"
    ssh "${REMOTE_USER}@${REMOTE_HOST}" "cp ${REMOTE_NODE_PATH} ${REMOTE_NODE_PATH}.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true"
    scp "$LOCAL_NODE_SOURCE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_NODE_PATH}"
fi

# Create symlink
echo -e "${GREEN}Creating symlink...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "
    cd ${REMOTE_PATH}
    rm -f can_detector.blob
    ln -s ${REMOTE_BLOB_NAME} can_detector.blob
    echo 'Symlink created: can_detector.blob -> ${REMOTE_BLOB_NAME}'
"

# Verify deployment
echo -e "${GREEN}Verifying deployment...${NC}"
ssh "${REMOTE_USER}@${REMOTE_HOST}" "
    cd ${REMOTE_PATH}
    echo 'Files in models directory:'
    ls -lh
    echo ''
    echo 'Symlink details:'
    ls -l can_detector.blob
"

echo ""
echo -e "${GREEN}=== Deployment Complete ===${NC}"
echo "Model deployed: ${REMOTE_BLOB_NAME}"
echo "Active model: can_detector.blob -> ${REMOTE_BLOB_NAME}"
if [ "$DEPLOY_NODE" -eq 1 ]; then
    echo "Node deployed: ${REMOTE_NODE_PATH}"
fi
echo ""
echo "Next steps:"
echo "1. SSH to robot: ssh ${REMOTE_USER}@${REMOTE_HOST}"
echo "2. Launch detector: ros2 launch base oakd_yolo26_detector.launch.py"
echo "3. Verify stream: ros2 topic hz /oakd/annotated_image"
echo "4. If needed, rollback blob with: cd ${REMOTE_PATH} && ln -sf <backup_file> can_detector.blob"
