# Sigyn AI

**Vision system training, export, and deployment toolkit for household service robots**

Sigyn AI is a complete workflow for training YOLO models and deploying them across
multiple edge AI devices. Designed for the robotics community with an emphasis on
reproducibility, documentation, and budget-friendly hardware options.

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
git clone https://github.com/wimblerobotics/sigyn_ai.git
cd sigyn_ai

# Install Python dependencies
pip install -r requirements.txt

# Set RoboFlow API key
export ROBOFLOW_API_KEY='your_key_here'
```

### 2. OAK-D End-to-End (Scripted, Recommended)

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

> **Getting slow FPS with OAK-D?** The key insight is to replace
> `YoloSpatialDetectionNetwork` with a generic `NeuralNetwork` node plus custom
> post-processing. This achieves **25–30 FPS** with Ultralytics-exported models.
> See [docs/OAKD_ULTRALYTICS_COMPLETE.md](docs/OAKD_ULTRALYTICS_COMPLETE.md) for
> the full explanation and the reference implementation in
> `scripts/oakd_can_detector_ultralytics.py`.

### 3. Other Devices

```bash
# Train (config-driven)
python src/training/train.py --config configs/training/can_detector_pihat.yaml

# Export for Pi 5 + Hailo-8
python src/export/export.py \
  --model models/checkpoints/<run_name>/weights/best.pt \
  --device pi5_hailo8 \
  --imgsz 512 \
  --compile

# Export for OAK-D
python src/export/export.py \
  --model models/checkpoints/<run_name>/weights/best.pt \
  --device oakd_lite \
  --imgsz 416 \
  --compile

# Deploy to robot
python src/deployment/deploy.py --model <run_name> --target sigyn --camera gripper_cam
```

## 📁 Repository Layout

```
sigyn_ai/
├── src/
│   ├── training/          # Training scripts
│   ├── export/            # Model export for devices
│   ├── deployment/        # Deployment automation
│   └── utils/             # Dataset download helpers
├── scripts/               # High-level train/deploy shell workflows
│   └── oakd_can_detector_ultralytics.py  # Reference ROS node for OAK-D
├── configs/
│   ├── devices/           # Per-device export/runtime settings
│   ├── training/          # Training configs
│   └── robots/            # Robot deployment configs
├── docker/                # Dockerfile + compilation wrappers
├── docs/                  # Detailed guides
└── models/                # Exported model artifacts; deployment-ready outputs may be tracked via Git LFS
```

## 📖 Documentation

- [Getting Started](docs/GETTING_STARTED.md)
- [Getting Started (No GPU)](docs/GETTING_STARTED_NO_GPU.md)
- [Building on a Budget](docs/BUILDING_ON_BUDGET.md)
- [OAK-D Complete Guide](docs/OAKD_ULTRALYTICS_COMPLETE.md) — **start here if using OAK-D**

## License

Apache-2.0. See [LICENSE](LICENSE).

