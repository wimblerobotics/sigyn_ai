#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0
#
# deploy_local_hef.sh - Copy Hailo .hef model from sigyn_ai to pi_can_detector
#
# Called as a Sigyn2 deployment hook (post_build stage on sigynVision).
# Both sigyn_ai and pi_can_detector are already cloned in the same workspace
# by the time this runs, so no SCP is needed.
#
# Usage: run automatically by setup_robot.py, or manually:
#   cd ~/sigyn_vision_ws
#   src/sigyn_ai/scripts/deploy_local_hef.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIGYN_AI_DIR="$(dirname "$SCRIPT_DIR")"                    # .../src/sigyn_ai
WORKSPACE_DIR="$(dirname "$(dirname "$SIGYN_AI_DIR")")"    # workspace root (up from src/)

MODELS_SRC="$SIGYN_AI_DIR/models/exported"
DEST_DIR="$WORKSPACE_DIR/src/pi_can_detector/models"

# Verify destination repo is present
if [ ! -d "$DEST_DIR" ]; then
    echo "  ✗ pi_can_detector not found at: $DEST_DIR"
    echo "    Has vcstool import been run?"
    exit 1
fi

mkdir -p "$DEST_DIR"

# Find all .hef files under models/exported (any model, any subdirectory)
HEF_FILES=$(find "$MODELS_SRC" -name "*.hef" -type f 2>/dev/null | sort)

if [ -z "$HEF_FILES" ]; then
    echo "  ✗ No .hef files found in $MODELS_SRC"
    echo "    Run 'git lfs pull' in sigyn_ai if this is a fresh clone."
    exit 1
fi

# Use the first .hef found (alphabetically — deterministic)
# For multiple models, extend this to accept MODEL_NAME env var
HEF=$(echo "$HEF_FILES" | head -1)
MODEL_DIR=$(dirname "$HEF")                  # .../exported/<ModelName>/pi5_hailo8
MODEL_VARIANT=$(basename "$(dirname "$MODEL_DIR")")  # e.g. PiHat512b

DEST_HEF="$DEST_DIR/${MODEL_VARIANT}.hef"

echo "  Source : $HEF"
echo "  Dest   : $DEST_HEF"

cp "$HEF" "$DEST_HEF"
ln -sf "${MODEL_VARIANT}.hef" "$DEST_DIR/can_detector.hef"
echo "  Symlink: can_detector.hef -> ${MODEL_VARIANT}.hef"

# Copy labels.txt if present
LABELS="$MODEL_DIR/labels.txt"
if [ -f "$LABELS" ]; then
    cp "$LABELS" "$DEST_DIR/${MODEL_VARIANT}_labels.txt"
    echo "  Labels : ${MODEL_VARIANT}_labels.txt"
fi

echo ""
echo "  ✓ Model deployed. Start service with:"
echo "    sudo systemctl start pi-can-detector"
