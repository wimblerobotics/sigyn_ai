# Getting Started Without a GPU

Don't have a GPU? No problem! Google Colab provides free GPU access for training YOLO models. This guide shows you how to use Sigyn AI with Colab.

## Why Google Colab?

- ✅ **Free GPU access** (Tesla T4 with 15GB VRAM)
- ✅ **Pre-installed PyTorch and CUDA**
- ✅ **No local installation required**
- ✅ **12-hour session limit** (enough for most training)
- ⚠️ **Sessions disconnect** (save checkpoints frequently)

## Setup

### 1. Create Colab Notebook

Go to https://colab.research.google.com/ and create a new notebook.

### 2. Enable GPU

- Click "Runtime" → "Change runtime type"
- Hardware accelerator: **GPU**
- GPU type: **T4** (free tier) or **V100/A100** (Colab Pro)
- Save

### 3. Verify GPU

Run in first cell:

```python
!nvidia-smi
```

You should see Tesla T4 or similar GPU.

## Training Workflow

### Method 1: Clone and Train (Recommended)

**Cell 1: Setup**

```python
# Clone repository
!git clone https://github.com/yourusername/sigyn_ai.git
%cd sigyn_ai

# Install dependencies (most already installed in Colab)
!pip install ultralytics==8.4.14 roboflow onnx-simplifier -q

# Verify GPU
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0)}")
```

**Cell 2: Download Dataset**

```python
# Set RoboFlow API key
import os
os.environ['ROBOFLOW_API_KEY'] = 'YOUR_API_KEY_HERE'

# Download dataset
!python src/utils/roboflow_download.py --project FCC4 --version 4 --format yolov8
```

**Cell 3: Train Model**

```python
# Train with config
!python src/training/train.py --config configs/training/can_detector_pihat.yaml
```

**Cell 4: Download Results**

```python
# Zip results for download
!zip -r model_results.zip models/checkpoints/can_detector_pihat_v1/

# Download via Colab UI
from google.colab import files
files.download('model_results.zip')
```

### Method 2: Inline Training (Quick Tests)

For quick experiments without full repo:

```python
# Install Ultralytics
!pip install ultralytics roboflow -q

# Download dataset
from roboflow import Roboflow
rf = Roboflow(api_key="YOUR_KEY")
project = rf.workspace().project("FCC4")
dataset = project.version(4).download("yolov8")

# Train
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
results = model.train(
    data=f'{dataset.location}/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,  # Colab T4 can handle larger batches
    device=0,
    project='runs',
    name='colab_exp1'
)

# Download best model
!zip -r model.zip runs/colab_exp1/
from google.colab import files
files.download('model.zip')
```

## Best Practices

### Batch Size

Colab GPUs have more VRAM than typical consumer GPUs:

| GPU | VRAM | Recommended Batch (640x640) |
|-----|------|-----------------------------|
| Tesla K80 | 12GB | 8-12 |
| Tesla T4 | 15GB | 12-16 |
| Tesla V100 | 16GB | 16-24 |
| A100 | 40GB | 32-48 |

### Save Frequently

Colab sessions disconnect after inactivity. Save checkpoints:

```python
# In training config
training:
  save_period: 10  # Save every 10 epochs
```

Or mount Google Drive:

```python
from google.colab import drive
drive.mount('/content/drive')

# Save models to Drive
output:
  project_dir: "/content/drive/MyDrive/sigyn_ai/models"
```

### Monitor Training

Use TensorBoard in Colab:

```python
%load_ext tensorboard
%tensorboard --logdir models/checkpoints/
```

## Exporting Models

After training in Colab, export for your device:

```python
# Export to ONNX
!python src/export/export.py \
    --model models/checkpoints/can_detector_pihat_v1/weights/best.pt \
    --device pi5_hailo8

# Download exported model
!zip -r exported_model.zip models/exported/
from google.colab import files
files.download('exported_model.zip')
```

**Then on your local machine:**

```bash
# Unzip exported model
unzip exported_model.zip

# Compile for Hailo (requires Docker locally)
python docker/run_hailo_compile.py --onnx models/exported/best/pi5_hailo8/best.onnx

# Deploy to robot
python src/deployment/deploy.py --model best --target sigyn --camera gripper_cam
```

## Colab Pro vs Free

| Feature | Free | Pro ($10/mo) |
|---------|------|--------------|
| GPU Access | Yes | Priority access |
| GPU Options | T4 | T4, V100, A100 |
| Session Length | 12 hours | 24 hours |
| Background Execution | No | Yes |
| RAM | 12GB | High-RAM option |

**Recommendation**: Start with free tier. Upgrade to Pro if:
- You train frequently (>10 hours/week)
- You need longer sessions (>12 hours)
- Free tier GPUs are often unavailable

## Full Example: Train Can Detector in Colab

Create new notebook and run these cells:

**Cell 1:**
```python
# Setup
!git clone https://github.com/yourusername/sigyn_ai.git
%cd sigyn_ai
!pip install ultralytics==8.4.14 roboflow onnx-simplifier -q
```

**Cell 2:**
```python
# Download dataset
import os
os.environ['ROBOFLOW_API_KEY'] = 'YOUR_KEY'
!python src/utils/roboflow_download.py --project FCC4 --version 4 --format yolov8
```

**Cell 3:**
```python
# Train (this will take ~3-5 minutes on T4)
!python src/training/train.py --config configs/training/can_detector_pihat.yaml
```

**Cell 4:**
```python
# View results
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('models/checkpoints/can_detector_pihat_v1/results.csv')
df[['metrics/precision(B)', 'metrics/recall(B)', 'metrics/mAP50(B)']].plot()
plt.title('Training Metrics')
plt.xlabel('Epoch')
plt.ylabel('Score')
plt.legend()
plt.grid(True)
plt.show()
```

**Cell 5:**
```python
# Test inference
from ultralytics import YOLO
from PIL import Image
import requests
from io import BytesIO

model = YOLO('models/checkpoints/can_detector_pihat_v1/weights/best.pt')

# Test on sample image (or upload your own)
# For uploaded image:
from google.colab import files
uploaded = files.upload()
img_path = list(uploaded.keys())[0]

results = model(img_path)
results[0].plot()  # Display detection
```

**Cell 6:**
```python
# Export and download
!python src/export/export.py \
    --model models/checkpoints/can_detector_pihat_v1/weights/best.pt \
    --device pi5_hailo8

!zip -r final_model.zip models/exported/ models/checkpoints/
from google.colab import files
files.download('final_model.zip')
```

## Troubleshooting

**"No GPU available"**
- Runtime → Change runtime type → Hardware accelerator: GPU
- Restart runtime
- If still no GPU, Colab free tier may be at capacity (try later)

**"Session disconnected"**
- Save checkpoints frequently (`save_period: 10`)
- Mount Google Drive for persistent storage
- Upgrade to Colab Pro for background execution

**"Disk space full"**
- Clear outputs: Edit → Clear all outputs
- Delete intermediate files: `!rm -rf runs/*/weights/last.pt`
- Use smaller datasets for testing

## Next Steps

Once you've trained in Colab:
1. Download model files
2. Export to device format locally (or in Colab)
3. Deploy to robot using deployment script
4. Integrate into ROS 2 nodes

See [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) for deployment instructions.

---

**Pro tip**: Create a template notebook with setup cells, save it in Google Drive, and reuse for each training run!
