# Sigyn AI - Architecture and Context for AI Assistants

This document provides quick context for AI assistants (like GitHub Copilot, Claude, GPT-4) to understand the project structure and help effectively.

## 🎯 Project Purpose

**Sigyn AI** is a training, export, and deployment toolkit for YOLO-based object detection on household service robots. It manages the complete workflow from RoboFlow datasets to deployed models on edge devices.

**Key Goal**: Enable fast iteration on vision models across multiple robot platforms with minimal manual steps.

## 🤖 Target Use Case

**Primary Robot**: Sigyn - a differential drive household service robot
- **Tasks**: Object manipulation (grasping cans), navigation, floor obstacle avoidance
- **Constraints**: Battery-powered (1KW, ~3hr runtime), power budget ~10W for vision
- **Control**: ROS 2 Jazzy, Nav2, BehaviorTree.CPP for reactive control
- **Vision Systems**:
  - Gripper-mounted camera (Pi 5 + Hailo-8) for manipulation
  - Top-mounted OAK-D Lite for navigation

## 📐 Architecture Overview

```
Workflow: Capture → RoboFlow → Train → Export → Deploy
          ↓          ↓          ↓       ↓        ↓
        Robot    Annotate    GPU    ONNX→    SSH to
        Camera   Dataset    Local   Device   Robot
                            /Colab  Format
```

### Component Responsibilities

1. **Training** (`src/training/`): Train YOLOv8 models with hardware auto-detection
2. **Export** (`src/export/`): Convert .pt → ONNX → device-specific formats
3. **Deployment** (`src/deployment/`): SSH-based deployment with rollback
4. **Utils** (`src/utils/`): RoboFlow API integration, dataset management

### Configuration Philosophy

- **Everything in YAML**: No command-line args to memorize
- **Device-specific configs**: Each device (Pi5/Hailo, OAK-D, Jetson) has optimization params
- **Robot configs**: Per-robot camera mappings and deployment targets
- **Training configs**: Dataset path, model size, augmentation, compute settings

## 🔧 Technical Stack

### Training
- **Framework**: Ultralytics YOLOv8 (8.4.14)
- **Backend**: PyTorch with CUDA
- **Hardware**: NVIDIA GPUs (RTX 2060/3060/4060 Ti) or Google Colab
- **Format**: .pt (PyTorch checkpoint)

### Export Pipeline
```
.pt → ONNX (common) → Device-specific:
                      ├─ Hailo-8: .hef (INT8, Hailo DFC)
                      ├─ OAK-D: .blob (FP16, Myriad X)
                      └─ Jetson: .engine (FP16, TensorRT)
```

### Deployment
- **Method**: SSH + SCP
- **Features**: Automatic backup, validation, rollback on failure
- **Security**: SSH key-based authentication

## 📊 Performance Requirements

| Device | Min FPS | Max Latency | Reason |
|--------|---------|-------------|--------|
| Pi 5 + Hailo-8 | 20 | 50ms | Gripper reactivity (behavior tree tick rate) |
| OAK-D Lite | 15 | 70ms | Navigation (slower movement, longer horizon) |
| Jetson Orin Nano | 30 | 35ms | Future upgrade, more headroom |

**Critical**: Latency matters more than FPS because behavior trees make decisions at each tick. A 100ms latency means robot is reacting to 100ms-old world state.

## 🗂️ Directory Structure Logic

```
sigyn_ai/
├── src/               # Executable Python scripts (chmod +x)
│   ├── training/      # train.py - YOLOv8 training with auto-config
│   ├── export/        # export.py - Multi-device ONNX export
│   ├── deployment/    # deploy.py - SSH deployment with rollback
│   └── utils/         # roboflow_download.py - Dataset management
├── configs/           # YAML configurations (version controlled)
│   ├── devices/       # Device constraints (TOPS, input sizes, formats)
│   ├── training/      # Training recipes (epochs, augmentation, etc.)
│   └── robots/        # Robot hardware and deployment targets
├── datasets/          # NOT in git (large files)
│   ├── raw_captures/  # Images from robot cameras
│   ├── roboflow_exports/  # Downloaded annotated datasets
│   └── synthetic/     # Future: generated training data
├── models/            # NOT in git (large files)
│   ├── checkpoints/   # .pt models from training
│   └── exported/      # Device-specific formats (.hef, .blob, .engine)
├── docker/            # Containerized workflows
│   ├── training.Dockerfile      # PyTorch + Ultralytics
│   ├── hailo_export.Dockerfile  # Hailo SDK
│   └── oakd_export.Dockerfile   # DepthAI SDK
└── docs/              # User documentation
```

## 🎛️ Key Design Decisions

### Why Separate Repo (not in Sigyn monorepo)?
- **Reusability**: Multiple robots can use same training infrastructure
- **Build isolation**: Hailo/OAK-D libraries don't need to be on all machines
- **Community**: Easier for others to adopt without Sigyn-specific dependencies

### Why YAML Configs (not CLI args)?
- **Reproducibility**: Config file = complete experiment record
- **Documentation**: Self-documenting (comments in YAML)
- **Avoid errors**: No typos in long command lines

### Why Hybrid Public/Private Data?
- **Code is public**: Anyone can use training/export/deployment scripts
- **Data is private**: .gitignore keeps large datasets and models local
- **Flexibility**: Each user downloads their own RoboFlow datasets

### Why Multiple Device Exports?
- **Different architectures**: Myriad X (OAK-D) ≠ Hailo-8 ≠ Ampere GPU (Jetson)
- **Input size optimization**: OAK-D prefers 416x416, Hailo-8 prefers 640x640
- **Quantization**: Hailo INT8, OAK-D FP16, Jetson FP16/INT8

## 🔍 Common Tasks (For AI Assistants)

### User wants to train a new model
**Check**: 
1. Dataset exists in `datasets/roboflow_exports/`
2. Training config in `configs/training/` references correct dataset
3. GPU available or suggest Colab

**Suggest**: Start with 20 epochs for testing, 100 for production

### User wants to export for a device
**Check**:
1. .pt checkpoint exists in `models/checkpoints/`
2. Device config exists in `configs/devices/`
3. Export script will create ONNX, but device-specific compilation may need Docker

**Note**: Hailo compilation requires Docker, OAK-D can use blobconverter, Jetson must compile on-device

### User wants to deploy
**Check**:
1. Exported model exists in `models/exported/<name>/<device>/`
2. Robot config exists in `configs/robots/`
3. SSH keys are set up (`ssh-copy-id`)
4. Target device is online

**Suggest**: Use `--dry-run` first to validate

### User asks about performance issues
**Common causes**:
- **Low FPS**: Model too large (try YOLOv8n instead of s/m), input size too large
- **High latency**: Batch processing (should be batch=1 on edge), slow post-processing
- **Accuracy drop**: Quantization effects (INT8 loses 1-3% typically), need more training data

## 📝 Conventions

### Naming
- **Models**: `<task>_<device>_v<number>` (e.g., `can_detector_pihat_v1`)
- **Configs**: Match model names or be descriptive
- **Datasets**: RoboFlow project names preserved

### Versioning
- **v1, v2, v3**: Increment when retraining with new data or different hyperparameters
- **Dataset versions**: Track via `dataset_metadata.json` in dataset directories

### Git Strategy
- **Commit**: All code, configs, docs, Dockerfiles
- **Ignore**: datasets/, models/ (use .gitignore)
- **Document**: Model training configs should be committed alongside code changes

## 🚨 Important Constraints

### Hardware
- **Mini ITX form factor**: Limits GPU to dual-slot, ~270mm length
- **Power budget on robot**: ~10W for vision (battery life concern)
- **No high-power GPUs on robot**: Training happens on desktop, inference on edge

### Software
- **ROS 2 Jazzy**: Robot runs latest ROS 2
- **Python 3.10+**: Ultralytics requirement
- **Docker for Hailo**: Hailo SDK only in container, manual compilation required

### Workflow
- **Train on desktop/Colab**: Local GPU or free cloud
- **Compile device-specific**: May require Docker or device-side compilation
- **Deploy via SSH**: No cloud deployment, direct to robot

## 🎓 Learning Resources

When users ask "how do I...":
- **Train**: Point to `docs/GETTING_STARTED.md`
- **No GPU**: Point to `docs/GETTING_STARTED_NO_GPU.md`
- **Budget hardware**: Point to `docs/BUILDING_ON_BUDGET.md`
- **Quick commands**: Point to `QUICK_REFERENCE.md`

## 🤝 Philosophy

**Built for the community**:
- Accessible (works on budget hardware)
- Documented (assume long gaps between usage)
- Modular (use parts without whole system)
- Practical (real constraints, real robots)

**Apache 2.0 License**: Use commercially or non-commercially, just credit the author.

---

**Last Updated**: 2026-02-13  
**Maintainer**: ros@amdc (Sigyn project)  
**Related Projects**: Sigyn (main robot repo), linorobot2 (inspiration)
