# Quick Reference Guide

> **Status (2026-02-14):** Use the OAK-D scripted flow below for current deployments.
>
> ```bash
> ./scripts/train_oakd.sh -v <roboflow_version> -n <run_name>
> ./scripts/deploy_oakd.sh -m <run_name>
> ```
>
> Pi 5 + Hailo reliable path:
>
> ```bash
> ./scripts/train_pi5_hailo.sh -v <roboflow_version> -n <run_name>
> ```
>
> On robot:
>
> ```bash
> ros2 launch base oakd_yolo26_detector.launch.py
> ```

## 🚀 Common Commands

### Setup

```bash
# Install dependencies
pip install -r requirements.txt

# Set RoboFlow API key
export ROBOFLOW_API_KEY='your_key_here'
```

### Download Dataset

```bash
# List your projects
python src/utils/roboflow_download.py --list-projects

# Download latest version
python src/utils/roboflow_download.py --project FCC4 --format yolov8

# Download specific version
python src/utils/roboflow_download.py --project FCC4 --version 4 --format yolov8
```

### Train Model

```bash
# Train with config
python src/training/train.py --config configs/training/can_detector_pihat.yaml

# Train with device override
python src/training/train.py --config configs/training/can_detector_pihat.yaml --device cuda:0

# Train on CPU (slow!)
python src/training/train.py --config configs/training/can_detector_pihat.yaml --device cpu
```

### Export Model

```bash
# Export for Pi 5 + Hailo-8
python src/export/export.py \
    --model runs/detect/models/checkpoints/can_detector_pihat_v1/weights/best.pt \
    --device pi5_hailo8 \
    --imgsz 512 \
    --output models/exported/can_detector_pihat_v1/pi5_hailo8

# Export for OAK-D and auto-compile
python src/export/export.py \
    --model runs/detect/models/checkpoints/can_detector_pihat_v1/weights/best.pt \
    --device oakd_lite \
    --compile

# Export for Jetson
python src/export/export.py \
    --model runs/detect/models/checkpoints/can_detector_pihat_v1/weights/best.pt \
    --device jetson_orin_nano
```

### Compile for Hailo (Docker)

```bash
# If your local image tag is this (common with Hailo package installs):
#   hailo8_ai_sw_suite_2025-10:1

# Using wrapper script (auto-detects Hailo CLI mode)
python docker/run_hailo_compile.py \
    --onnx models/exported/can_detector_pihat_v1/pi5_hailo8/best.onnx \
    --docker-image hailo8_ai_sw_suite_2025-10:1

# Tip: if your local image is `sigyn_ai_hailo:latest`, omit --docker-image
```

### Deploy to Robot

```bash
# Deploy to specific camera
python src/deployment/deploy.py \
    --model can_detector_pihat_v1 \
    --target sigyn \
    --camera gripper_cam

# Deploy to all cameras
python src/deployment/deploy.py \
    --model can_detector_oakd_v1 \
    --target sigyn \
    --all-cameras

# Dry run (test without deploying)
python src/deployment/deploy.py \
    --model can_detector_pihat_v1 \
    --target sigyn \
    --camera gripper_cam \
    --dry-run
```

## 📁 File Locations

### Configs
- Training: `configs/training/<name>.yaml`
- Devices: `configs/devices/{pi5_hailo8,oakd_lite,jetson_orin_nano}.yaml`
- Robots: `configs/robots/<robot_name>.yaml`

### Data
- Datasets: `datasets/roboflow_exports/<project>.<version>-<format>/`
- Raw captures: `datasets/raw_captures/<date>/`

### Models
- Checkpoints: `runs/detect/models/checkpoints/<name>/weights/best.pt`
- Exported: `models/exported/<name>/<device>/`

## 🔧 Troubleshooting

### Training Issues

**CUDA out of memory:**
```bash
# Reduce batch size in config
compute:
  batch_size: 2  # Instead of 4 or 8
```

**Shared memory error:**
```bash
# Set workers to 0 in config
compute:
  workers: 0
```

**No GPU:**
```bash
# Use Google Colab (see docs/GETTING_STARTED_NO_GPU.md)
# Or train on CPU (very slow):
python src/training/train.py --config configs/training/can_detector_pihat.yaml --device cpu
```

### Export Issues

**Hailo compilation fails:**
```bash
# Check Docker is running
docker ps

# Try optimization level 1 instead of 2
# Edit: models/exported/<name>/pi5_hailo8/compile_hailo.sh
```

**OAK-D blobconverter fails:**
```bash
# Install blobconverter
pip install blobconverter

# Or upload ONNX manually:
# https://blobconverter.luxonis.com/
```

### Deployment Issues

**SSH connection refused:**
```bash
# Check target is online
ping sigyn.local

# Setup SSH keys
ssh-copy-id ros@sigyn.local

# Test SSH
ssh ros@sigyn.local 'echo success'
```

**Deployment test fails:**
```bash
# Check files on target
ssh ros@sigyn.local 'ls -la ~/pi_can_detector_ws/src/pi_can_detector/'

# Check target has required libraries
ssh ros@sigyn.local 'python3 -c "import hailo_platform"'
```

## 📊 Performance Expectations

| Device | Model | Input Size | FPS | Latency |
|--------|-------|------------|-----|---------|
| Pi 5 + Hailo-8 | YOLOv8n | 640x640 | 25-30 | ~35ms |
| OAK-D Lite | YOLOv8n | 416x416 | 15-20 | ~60ms |
| Jetson Orin Nano | YOLOv8n | 640x640 | 60-80 | ~15ms |

## 🌐 Useful Links

- **RoboFlow**: https://app.roboflow.com/
- **Ultralytics Docs**: https://docs.ultralytics.com/
- **Hailo SDK**: https://hailo.ai/developer-zone/
- **OAK-D Docs**: https://docs.luxonis.com/
- **TensorRT**: https://developer.nvidia.com/tensorrt

## 💡 Tips

1. **Always backup**: Deployment script backs up automatically
2. **Use configs**: Don't rely on command-line args you'll forget
3. **Version everything**: Model names should include version (v1, v2, etc.)
4. **Test locally first**: Use `--dry-run` before actual deployment
5. **Monitor metrics**: Check training results.csv for overfitting
6. **Start small**: 20 epochs for testing, 100 for production

## 🔄 Typical Workflow

```bash
# 1. Capture images on robot
# (Use your robot's camera capture method)

# 2. Upload to RoboFlow, annotate, generate version

# 3. Download dataset
python src/utils/roboflow_download.py --project MyProject --format yolov8

# 4. Create training config (copy from examples)
cp configs/training/can_detector_pihat.yaml configs/training/my_model.yaml
# Edit: dataset path, model size, epochs, etc.

# 5. Train
python src/training/train.py --config configs/training/my_model.yaml

# 6. Export for your device
python src/export/export.py \
    --model models/checkpoints/my_model_v1/weights/best.pt \
    --device pi5_hailo8

# 7. Compile (if needed)
python docker/run_hailo_compile.py \
    --onnx models/exported/best/pi5_hailo8/best.onnx

# 8. Deploy
python src/deployment/deploy.py \
    --model my_model_v1 \
    --target myrobot \
    --camera main_cam

# 9. Test on robot
# (Use your robot's test scripts)

# 10. Iterate: More data, retrain, redeploy
```

---

## 🔧 OAK-D Ultralytics Solution

### Deploy Ultralytics-Compatible Node
```bash
# Train + export
./scripts/train_oakd.sh -v <roboflow_version> -n <run_name>

# Deploy blob + node
./scripts/deploy_oakd.sh -m <run_name>
```

### Key Features
- ✅ Works with Ultralytics YOLOv5 single-output exports
- ✅ Uses `dai.node.NeuralNetwork` (generic)
- ✅ Custom NMS and spatial computation
- ✅ No anchor mask errors!

### Documentation
- **Complete Guide**: `docs/OAKD_ULTRALYTICS_COMPLETE.md`

---

**Need more help?** See full documentation in `docs/` directory.
