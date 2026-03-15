# Models Directory Structure

This directory contains trained models and exported formats. Deployment-ready model binaries may be committed with Git LFS when they are needed for reproducible deployment. Intermediate build artifacts and local cache files should stay out of git.

## Expected Structure

```
models/
├── checkpoints/            # Training outputs (.pt models)
│   ├── can_detector_pihat_v1/
│   │   ├── weights/
│   │   │   ├── best.pt          # Best model during training
│   │   │   └── last.pt          # Last epoch
│   │   ├── results.csv          # Training metrics
│   │   ├── confusion_matrix.png
│   │   └── ...
│   └── multi_class_oakd_v1/
│
└── exported/               # Device-specific exports
    ├── can_detector_pihat_v1/
    │   ├── pi5_hailo8/
    │   │   ├── best.hef         # Compiled Hailo model
    │   │   ├── best.onnx        # Intermediate ONNX
    │   │   └── labels.txt       # Class names
    │   ├── oakd_lite/
    │   │   ├── best.blob        # Compiled OAK-D model
    │   │   ├── best.onnx
    │   │   └── labels.txt
    │   └── jetson_orin_nano/
    │       ├── best.engine      # Compiled TensorRT model
    │       ├── best.onnx
    │       └── labels.txt
    └── multi_class_oakd_v1/
```

## Model Lifecycle

1. **Training** → `checkpoints/<name>/weights/best.pt`
2. **Export** → `exported/<name>/<device>/best.onnx`
3. **Compilation** → `exported/<name>/<device>/best.{hef,blob,engine}`
4. **Deployment** → Copy to robot target device

## Storage Requirements

- **Checkpoint** (.pt): 5-100 MB depending on model size
- **ONNX** (.onnx): 10-200 MB
- **Hailo** (.hef): 5-50 MB (compressed)
- **OAK-D** (.blob): 5-30 MB
- **Jetson** (.engine): 10-100 MB

## Version Control Policy

- Commit reproducible training configs in `configs/training/`
- Commit deployment-ready exports such as `.hef`, `.blob`, `.onnx`, and `.engine` via Git LFS when they are needed by scripts or docs
- Do not commit Hailo `.har` files, compiler logs, or scratch outputs
- Pretrained vendor weights downloaded at the repo root (for example `yolov8n.pt`) are local cache files and are ignored

## Model Versioning

Use descriptive names with version numbers:
- `can_detector_pihat_v1` - First version, Pi 5 + Hailo target
- `can_detector_pihat_v2` - Improved with more data
- `multi_class_oakd_v1` - Multi-class model for OAK-D

Keep training configs alongside models for reproducibility.

## Cleaning Up

To save space, delete old checkpoints but keep exported models:

```bash
# Remove intermediate checkpoints (keep best.pt)
find models/checkpoints -name "last.pt" -delete
find models/checkpoints -name "epoch*.pt" -delete

# Remove old training outputs
rm -rf models/checkpoints/*/runs/
```
