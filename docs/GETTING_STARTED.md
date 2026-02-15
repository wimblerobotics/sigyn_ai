# Getting Started with Sigyn AI

This guide walks through the complete workflow from capturing images to deploying models on your robot.

## Prerequisites

- Python 3.10 or newer
- (Optional) NVIDIA GPU with CUDA support
- RoboFlow account (free tier is fine)
- Target device (Pi 5 + Hailo, OAK-D, or Jetson)

## Installation

### 1. Clone Repository

```bash
cd ~
git clone https://github.com/yourusername/sigyn_ai.git
cd sigyn_ai
```

### 2. Install Python Dependencies

```bash
# Core dependencies
pip install ultralytics==8.4.14 roboflow onnx onnxsim pyyaml

# Optional: For OAK-D compilation
pip install depthai blobconverter

# Optional: For development
pip install pytest black flake8
```

### 3. Configure RoboFlow

Get your API key from https://app.roboflow.com/settings/api

```bash
# Add to ~/.bashrc for persistence
echo 'export ROBOFLOW_API_KEY="your_key_here"' >> ~/.bashrc
source ~/.bashrc

# Or set for current session
export ROBOFLOW_API_KEY="your_key_here"
```

### 4. Verify Installation

```bash
# Check GPU availability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"

# Test RoboFlow connection
python src/utils/roboflow_download.py --list-projects
```

## Complete Workflow Example

### Step 1: Capture and Annotate Images

**Capture images on your robot:**

```bash
# Use ROS 2 image_view or custom capture node
ros2 run image_tools cam2image --ros-args -p frequency:=1.0

# Or use v4l2-ctl for direct camera capture
v4l2-ctl --device /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=YUYV \
    --stream-mmap --stream-to=capture_%03d.jpg --stream-count=50
```

**Upload and annotate in RoboFlow:**

1. Create project: https://app.roboflow.com/
2. Upload images (drag and drop)
3. Annotate bounding boxes
4. Apply preprocessing:
   - Auto-contrast: ✓
   - Resize: 640x640 (for Pi 5/Hailo) or 416x416 (for OAK-D)
5. Generate version with augmentation:
   - Rotation: ±10°
   - Brightness: ±20%
   - Blur: slight

### Step 2: Download Dataset

```bash
# Download latest version
python src/utils/roboflow_download.py --project YourProject --format yolov8

# Or specific version
python src/utils/roboflow_download.py --project YourProject --version 4 --format yolov8
```

Dataset will be downloaded to: `datasets/roboflow_exports/YourProject.v4-yolov8/`

### Step 3: Create Training Configuration

Copy and edit the example config:

```bash
cp configs/training/can_detector_pihat.yaml configs/training/my_detector.yaml
```

Edit `configs/training/my_detector.yaml`:

```yaml
model:
  size: "n"  # n=nano, s=small, m=medium

dataset:
  path: "datasets/roboflow_exports/YourProject.v4-yolov8"

compute:
  batch_size: "auto"  # Auto-detect based on GPU
  workers: "auto"

training:
  epochs: 100
  imgsz: 640  # Must match device target

output:
  project_dir: "models/checkpoints"
  run_name: "my_detector_v1"
```

### Step 4: Train Model

```bash
# Train with auto-detected hardware
python src/training/train.py --config configs/training/my_detector.yaml
```

**Expected output:**
```
🔍 Detecting hardware...
   GPU: NVIDIA GeForce RTX 2060
   VRAM: 5.9 GB
📊 Auto-detected batch size: 4
👷 Auto-detected workers: 0

🚀 Starting training with YOLOv8n
   Epochs: 100
   Image size: 640
   Batch: 4

Epoch 1/100: 100%|████████| 25/25 [00:03<00:00, 7.23it/s]
...
Epoch 100/100: Precision 0.982, Recall 0.818, mAP@50 0.922

✅ Training complete!
   Best model: models/checkpoints/my_detector_v1/weights/best.pt
```

### Step 5: Validate Results

```bash
# Check training metrics
cat models/checkpoints/my_detector_v1/results.csv

# View training curves (requires matplotlib)
python -c "
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('models/checkpoints/my_detector_v1/results.csv')
df[['metrics/precision(B)', 'metrics/recall(B)', 'metrics/mAP50(B)']].plot()
plt.savefig('training_metrics.png')
print('Saved: training_metrics.png')
"
```

### Step 6: Export for Target Device

**For Pi 5 + Hailo-8:**

```bash
python src/export/export.py \
    --model models/checkpoints/my_detector_v1/weights/best.pt \
    --device pi5_hailo8
```

Then compile to .hef (requires Hailo Docker):

```bash
python docker/run_hailo_compile.py \
    --onnx models/exported/best/pi5_hailo8/best.onnx
```

**For OAK-D Lite:**

```bash
python src/export/export.py \
    --model models/checkpoints/my_detector_v1/weights/best.pt \
    --device oakd_lite \
    --compile  # Automatically compiles to .blob
```

**For Jetson Orin Nano:**

```bash
python src/export/export.py \
    --model models/checkpoints/my_detector_v1/weights/best.pt \
    --device jetson_orin_nano

# Then SCP to Jetson and compile there
scp models/exported/best/jetson_orin_nano/* ros@jetson.local:~/
ssh ros@jetson.local "python3 compile_tensorrt.py"
```

### Step 7: Configure Robot Deployment

Edit `configs/robots/sigyn.yaml` (or create your own):

```yaml
robot:
  name: "YourRobot"
  identifier: "yourrobot"

hardware:
  cameras:
    main_cam:
      device: "pi5_hailo8"  # or oakd_lite, jetson_orin_nano
      deployment:
        host: "ros@yourrobot.local"
        model_dir: "~/detector_ws/models/"
```

### Step 8: Deploy to Robot

```bash
# Dry run (validation only)
python src/deployment/deploy.py \
    --model my_detector_v1 \
    --target yourrobot \
    --camera main_cam \
    --dry-run

# Actual deployment
python src/deployment/deploy.py \
    --model my_detector_v1 \
    --target yourrobot \
    --camera main_cam
```

**Expected output:**
```
🔍 Loading robot config: yourrobot
🎯 Deploying to camera: main_cam
   Device: pi5_hailo8
💾 Backed up existing models to: ~/detector_ws/models/backups/20260213_143022
📤 Deploying model files...
   Copying: best.hef
   Copying: labels.txt
✅ Files deployed
🧪 Testing deployment...
✅ Deployment test passed
🎉 Successfully deployed my_detector_v1 to main_cam
```

### OAK-D Scripted Workflow (Recommended)

For the current OAK-D path, use the dedicated scripts:

```bash
# Train + export
./scripts/train_oakd.sh -v <roboflow_version> -n <run_name>

# Deploy blob + node script
./scripts/deploy_oakd.sh -m <run_name>

# On robot
ros2 launch base oakd_yolo26_detector.launch.py
```

## Next Steps

- **Improve accuracy**: Add more training images, especially edge cases
- **Multi-class detection**: Annotate multiple object types
- **Benchmark performance**: Measure FPS and latency on device
- **Integration**: Connect detector to ROS 2 nodes, behavior trees

## Troubleshooting

### Training Issues

**"CUDA out of memory"**
- Reduce batch size in config: `batch_size: 2`
- Use smaller model: `size: "n"` instead of `"s"`
- Close other GPU-using applications

**"Shared memory error"**
- Set workers to 0: `workers: 0`
- Or increase shared memory: `docker run --shm-size=8g`

**Training is very slow (CPU)**
- Use Google Colab (see [GETTING_STARTED_NO_GPU.md](GETTING_STARTED_NO_GPU.md))
- Reduce epochs for testing: `epochs: 20`
- Use smaller dataset for proof-of-concept

### Export Issues

**"Hailo compilation failed"**
- Check ONNX is simplified: use `--compile` flag
- Try optimization level 1: edit compile script
- Verify Docker image: `docker images | grep hailo`

**"blobconverter upload failed"**
- Check internet connection
- Try again (service can be slow)
- Use local OpenVINO compilation (advanced)

### Deployment Issues

**"SSH connection refused"**
- Verify target is online: `ping yourrobot.local`
- Check SSH keys: `ssh-copy-id ros@yourrobot.local`
- Verify hostname in robot config

**"Test failed after deployment"**
- Check target has required libraries
- Verify model files copied: `ssh ros@yourrobot.local 'ls -la ~/detector_ws/models/'`
- Check logs on target device

See the OAK-D docs for current details:
- [OAKD_ULTRALYTICS_COMPLETE.md](OAKD_ULTRALYTICS_COMPLETE.md)

## Getting Help

- **Issues**: use the repository issue tracker
- **Discussions**: use repository discussions

---

For budget/no-GPU setup, continue to [GETTING_STARTED_NO_GPU.md](GETTING_STARTED_NO_GPU.md).
