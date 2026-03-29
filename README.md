# Sigyn AI

**YOLO training, export, and deployment toolkit for robots**

Tested devices: **Raspberry Pi 5 + Hailo-8 AI HAT** and **Luxonis OAK-D / OAK-D Lite** running ROS 2 Jazzy on Ubuntu 24.04.

> Built for the [Homebrew Robotics Club](https://www.hbrobotics.org/) community and
> other open-source robotics projects. Apache 2.0 licensed.

---

## What This Does

A complete, scriptable workflow for getting YOLO object detection running on
your robot:

```
Capture images → RoboFlow dataset → Train (GPU/Colab) → Export → Deploy
```

Everything is driven by shell scripts and YAML configs — no notebook required
for the normal path.

---

## Device Quick-Reference

| Device | YOLO | Input | Notes |
|--------|------|-------|-------|
| Pi 5 + Hailo-8 AI HAT | YOLOv8n | 512 × 512 | INT8 on Hailo-8. 640 sometimes fails compile — see [Pi Hat notes](#pi-5--hailo-8-ai-hat-notes) |
| OAK-D / OAK-D Lite | YOLOv5n | 416 × 416 | FP16 on Myriad X. Same blob works for both full OAK-D and OAK-D Lite |

> **Why YOLOv5n for OAK-D?**
> The OAK-D uses an Intel Myriad X VPU and runs Luxonis DepthAI `.blob` models
> compiled with OpenVINO. Ultralytics-exported YOLOv5n compiles reliably and
> achieves 18–25 FPS. YOLOv8n *can* be used but requires the same
> `NeuralNetwork`-node workaround and has been less consistently tested on the
> Myriad X. 

---

## Quick Start

### 1. Install

```bash
git clone https://github.com/wimblerobotics/sigyn_ai.git
cd sigyn_ai

# Python dependencies (virtual environment recommended)
pip install -r requirements.txt

# Enable repo-managed git hooks (includes pre-push secret scan)
git config core.hooksPath .githooks

# RoboFlow API key — get yours at https://app.roboflow.com/settings/api
export ROBOFLOW_API_KEY='your_key_here'
```

### 2. Pi 5 + Hailo-8 End-to-End

```bash
# Download RoboFlow dataset, train, export ONNX, compile to .hef — one command
./scripts/train_pi5_hailo.sh -v <roboflow_version> -n <run_name>

# Deploy .hef to robot over SSH
./scripts/deploy_pi5_hailo.sh -m <run_name> \
  -h <robot_host> \
  -u <robot_user> \
  -p <remote_models_dir>
```

### 3. OAK-D End-to-End

```bash
# Download RoboFlow dataset, train on YOLOv5n, export to .blob — one command
./scripts/train_oakd.sh -v <roboflow_version> -n <run_name>

# Deploy blob + detector node to robot over SSH
./scripts/deploy_oakd.sh -m <run_name> \
  -h <robot_host> \
  -u <robot_user> \
  -p <remote_models_dir> \
  -r <remote_detector_node.py>
```

After deploying, launch the detector on the robot. In the Sigyn robot the
launch file is named `oakd_yolo26_detector.launch.py` — this is a historical
name from when the architecture was under development; "26" refers to the
YOLOv5 26 × 26 feature-map layer, not a YOLO version number:

```bash
# On the robot (Sigyn-specific launch file name)
ros2 launch sigyn_bringup oakd_yolo26_detector.launch.py
```

For your own robot, adapt the launch file name accordingly.

> **Slow FPS on OAK-D?** Replace `YoloSpatialDetectionNetwork` with a generic
> `NeuralNetwork` node. This achieves 18–25 FPS at 416 × 416.
> See [docs/OAKD_ULTRALYTICS_COMPLETE.md](docs/OAKD_ULTRALYTICS_COMPLETE.md).

---

## Pi 5 + Hailo-8 AI HAT Notes

- The Hailo-8 (not 8L) runs INT8 quantized inference at up to 26 TOPS.
- **Default input size: 512 × 512.** This compiles reliably.
- **640 × 640 may fail** the Hailo compiler with `Agent infeasible` depending
  on model graph topology. Use `--try-640` if you want to attempt it with
  automatic fallback to 512.
- Use YOLOv8n (nano) — larger sizes (YOLOv8s, m) hit memory limits more often.
- Reference: [Hailo-8 AI HAT hardware page](https://www.raspberrypi.com/products/ai-hat/)

---

## Building a Training Dataset with RoboFlow

See **[docs/ROBOFLOW_WORKFLOW.md](docs/ROBOFLOW_WORKFLOW.md)** for the complete guide covering:

- Creating a free account and project
- Capturing images with the robot's bluetooth joystick (Y button → OAK-D, B button → Pi camera)
- Uploading, annotating, and auto-labeling
- Correct preprocessing settings for each device (512 × 512 or 416 × 416)
- Adding new images to an existing version and retraining

---

## Repository Layout

```
sigyn_ai/
├── src/
│   ├── training/          # train.py — YOLOv8/v5 training with hardware auto-detection
│   ├── export/            # export.py — .pt → ONNX → device-specific format
│   ├── deployment/        # deploy.py — SSH-based deployment with rollback
│   └── utils/             # roboflow_download.py — dataset management
├── scripts/               # One-command train/deploy shell workflows
│   └── oakd_can_detector_ultralytics.py  # Reference ROS 2 node for OAK-D
├── configs/
│   ├── devices/           # Per-device specs and export settings
│   ├── training/          # Training configs (model size, dataset path, etc.)
│   └── robots/            # Robot deployment targets
├── docker/                # Hailo DFC and OAK-D blob compilation containers
├── docs/                  # Detailed guides
└── datasets/ / models/    # Local only — excluded from git (large files)
```

---

## Documentation

| Guide | When to read it |
|-------|----------------|
| [Getting Started](docs/GETTING_STARTED.md) | Full step-by-step from scratch |
| [RoboFlow Workflow](docs/ROBOFLOW_WORKFLOW.md) | Dataset creation and management |
| [OAK-D Complete Guide](docs/OAKD_ULTRALYTICS_COMPLETE.md) | OAK-D architecture issue and fix |
| [Getting Started (No GPU)](docs/GETTING_STARTED_NO_GPU.md) | Using Google Colab |
| [Building on a Budget](docs/BUILDING_ON_BUDGET.md) | Hardware recommendations |
| [GPU Recommendations](docs/GPU_RECOMMENDATIONS.md) | Training GPU selection |

---

## Related Repositories

These repos must all be **public** before the documentation links and image-capture
workflow are fully usable:

| Repo | Purpose | Required for |
|------|---------|--------------|
| [`wimblerobotics/sigyn_bluetooth_joystick`](https://github.com/wimblerobotics/sigyn_bluetooth_joystick) | Joystick node — publishes button topics | Image capture |
| [`wimblerobotics/pi_can_detector`](https://github.com/wimblerobotics/pi_can_detector) | Pi 5 + Hailo-8 detector; subscribes `/sigyn/take_gripper_picture`, saves to `~/training_images/` | **Pi camera training images** |
| [`wimblerobotics/sigyn_oakd_detection`](https://github.com/wimblerobotics/sigyn_oakd_detection) | OAK-D detector node (YOLOv5n blob) | **OAK-D training images** *(capture not yet wired up)* |
| [`wimblerobotics/Sigyn`](https://github.com/wimblerobotics/Sigyn) | Full robot bringup, launch files | Running the robot |
| [`wimblerobotics/can_do_challenge`](https://github.com/wimblerobotics/can_do_challenge) | Can-Do Challenge task controller | Context for the fcc4 project |

---

## License

Apache-2.0. See [LICENSE](LICENSE).

