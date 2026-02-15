# Sigyn AI

**Vision system training, export, and deployment toolkit for household service robots**

Sigyn AI is a complete workflow for training YOLO models and deploying them across multiple edge AI devices. Designed for the robotics community with an emphasis on reproducibility, documentation, and budget-friendly hardware options.

## 🎯 Features

- **Multi-Device Support**: Pi 5 + Hailo-8, OAK-D Lite, Jetson Orin Nano
- **Flexible Training**: Works with GPU, CPU, or Google Colab
- **RoboFlow Integration**: Automated dataset download and management
- **One-Command Deployment**: SSH-based deployment with automatic rollback
- **Docker Workflows**: Isolated environments for training and compilation
- **Community-Friendly**: Apache 2.0 license, extensive documentation

## 🚀 Quick Start

### 1. Installation

```bash
# Clone repository
git clone https://github.com/yourusername/sigyn_ai.git
cd sigyn_ai

# Install Python dependencies
pip install ultralytics roboflow onnx onnx-simplifier pyyaml

# Set RoboFlow API key
export ROBOFLOW_API_KEY='your_key_here'
```

### 2. Download Dataset

```bash
# Download from RoboFlow
python src/utils/roboflow_download.py --project FCC4 --version 4 --format yolov8
```

### 3. Train Model

```bash
# Train with config file
python src/training/train.py --config configs/training/can_detector_pihat.yaml
```

### 4. Export for Device

```bash
# Export for Pi 5 + Hailo-8
python src/export/export.py --model models/checkpoints/can_detector_pihat_v1/weights/best.pt --device pi5_hailo8

# Export for OAK-D
python src/export/export.py --model models/checkpoints/can_detector_pihat_v1/weights/best.pt --device oakd_lite
```

### 5. Deploy to Robot

```bash
# Deploy to specific camera
python src/deployment/deploy.py --model can_detector_pihat_v1 --target sigyn --camera gripper_cam
```
# OAK-D end-to-end (recommended)
./scripts/train_oakd.sh -v 5 -n oakd_v5
./scripts/deploy_oakd.sh -m oakd_v5

# Or generic deployment flow
python src/deployment/deploy.py --model can_detector_pihat_v1 --target sigyn --camera gripper_cam

## 📁 Repository Structure

```
sigyn_ai/
├── src/
│   ├── training/          # Training scripts
│   ├── export/            # Model export for devices
│   ├── deployment/        # Deployment automation
# Sigyn AI

Vision model training, export, and deployment toolkit for household robotics.

## Features

- Multi-device support: Pi 5 + Hailo-8, OAK-D Lite, Jetson Orin Nano
- RoboFlow dataset download integration
- Scripted train/export/deploy flow for OAK-D
- Python-first workflows with optional Docker steps

## Quick Start

```bash
# 1) Install dependencies
pip install -r requirements.txt

# 2) Set RoboFlow key
export ROBOFLOW_API_KEY='your_key_here'

# 3) Train (config-driven)
python src/training/train.py --config configs/training/can_detector_pihat.yaml

# 4) Export (example: OAK-D)
python src/export/export.py \
  --model models/checkpoints/<run_name>/weights/best.pt \
  --device oakd_lite \
  --imgsz 416 \
  --compile
```

## OAK-D End-to-End (Scripted)

```bash
# Train + export
./scripts/train_oakd.sh -v <roboflow_version> -n <run_name>

# Deploy blob + OAK-D node script
./scripts/deploy_oakd.sh -m <run_name>
```

Then on robot:

```bash
ros2 launch base oakd_yolo26_detector.launch.py
```

## Repository Layout

- src/training: training entrypoint
- src/export: model export and device compilation hooks
- src/deployment: generic deployment helper
- scripts: high-level train/deploy shell workflows
- configs/devices: per-device export/runtime settings
- configs/training: training configs
- docs: detailed guides and OAK-D notes

## Documentation

- [Getting Started](docs/GETTING_STARTED.md)
- [Getting Started (No GPU)](docs/GETTING_STARTED_NO_GPU.md)
- [Building on a Budget](docs/BUILDING_ON_BUDGET.md)
- [OAK-D Ultralytics Complete](docs/OAKD_ULTRALYTICS_COMPLETE.md)

## License

Apache-2.0. See [LICENSE](LICENSE).

