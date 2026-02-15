# Contributing to Sigyn AI

Thank you for your interest in contributing to Sigyn AI! This project aims to make vision AI accessible to the robotics community.

## 🎯 Project Goals

1. **Accessibility**: Works on budget hardware (used GPUs, Colab, edge devices <$250)
2. **Reproducibility**: YAML configs + versioning = repeatable experiments
3. **Documentation**: Assume long gaps between usage, make rediscovery easy
4. **Community**: Help others build vision systems for their robots

## 🤝 How to Contribute

### Good First Issues

**Documentation**:
- Add troubleshooting entries for your specific hardware
- Improve error messages in scripts
- Add examples for different use cases
- Translate docs to other languages

**Features**:
- Add support for YOLOv9/v10/v11
- Improve RoboFlow integration (auto-upload metrics)
- Add visualization tools for bounding boxes
- Create Jupyter notebooks for tutorials

**Testing**:
- Test on different GPUs (AMD, Intel)
- Test on Mac (MPS backend)
- Test with different RoboFlow projects
- Verify deployment on different robots

### Larger Contributions

**Training**:
- Quantization-Aware Training (QAT) for Hailo
- Multi-GPU training support
- Distributed training across machines
- Hyperparameter optimization (Optuna, Ray Tune)

**Export**:
- NVIDIA Jetson Nano support (older Jetson)
- Qualcomm Hexagon DSP support
- Apple Neural Engine export
- OpenVINO export for Intel NUC

**Deployment**:
- Web UI for deployment management
- ROS 2 integration packages
- Docker Compose orchestration
- Cloud deployment (AWS, Azure IoT)

**Advanced Features**:
- Semantic segmentation pipeline
- Model distillation (teacher-student)
- Synthetic data generation (Blender, Isaac Sim)
- Active learning workflow

## 📋 Contribution Workflow

### 1. Setup Development Environment

```bash
# Fork and clone
git clone https://github.com/yourusername/sigyn_ai.git
cd sigyn_ai

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies + dev tools
pip install -r requirements.txt
pip install pytest black flake8 pre-commit
```

### 2. Create Feature Branch

```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

### 3. Make Changes

**Code Style**:
- Run `black .` for formatting
- Run `flake8 .` for linting
- Add type hints where reasonable
- Write docstrings for public functions

**Testing**:
```bash
# Run tests
pytest tests/

# Add new tests for your feature
# tests/test_your_feature.py
```

**Documentation**:
- Update relevant docs in `docs/`
- Add examples to QUICK_REFERENCE.md
- Update ROADMAP.md if adding planned features

### 4. Commit Changes

**Commit Message Format**:
```
<type>: <subject>

<body>

<footer>
```

**Types**:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style (formatting, no logic change)
- `refactor`: Code refactoring
- `test`: Adding/updating tests
- `chore`: Maintenance (dependencies, build)

**Example**:
```
feat: Add support for YOLOv9 models

- Update export.py to handle YOLOv9 architecture
- Add yolov9n.yaml training config example
- Document YOLOv9-specific settings in devices configs

Closes #42
```

### 5. Push and Create Pull Request

```bash
git push origin feature/your-feature-name
```

Then create PR on GitHub with:
- **Title**: Clear, descriptive (same as commit subject)
- **Description**: What, why, how
- **Testing**: How did you test this?
- **Screenshots**: If UI/visual changes

## 🧪 Testing Guidelines

### Unit Tests
```python
# tests/test_training.py
def test_hardware_detection():
    info = detect_hardware()
    assert 'cuda_available' in info
    assert 'device_name' in info
```

### Integration Tests
```python
# tests/test_export_integration.py
def test_export_to_onnx(tmp_path):
    # Test complete export workflow
    model_path = "path/to/test_model.pt"
    output = export_to_onnx(model_path, device_config, tmp_path)
    assert output.exists()
    assert output.suffix == '.onnx'
```

### Manual Testing
1. Test on at least one device (Pi5/Hailo, OAK-D, or Jetson)
2. Verify deployment (dry-run at minimum)
3. Check training metrics make sense

## 📝 Code Review Checklist

Before submitting PR, ensure:
- [ ] Code follows style guide (black, flake8)
- [ ] Tests pass (`pytest tests/`)
- [ ] Documentation updated
- [ ] CHANGELOG.md updated (if applicable)
- [ ] No secrets/API keys committed
- [ ] Large files not committed (datasets, models)
- [ ] Works on at least Python 3.10

## 🐛 Bug Reports

**Use GitHub Issues** with:
1. **Environment**: OS, Python version, GPU
2. **Steps to reproduce**: Exact commands run
3. **Expected behavior**: What should happen
4. **Actual behavior**: What actually happened
5. **Logs**: Full error messages, stack traces
6. **Config files**: Relevant YAML configs

**Good Bug Report Example**:
```markdown
### Environment
- OS: Ubuntu 22.04
- Python: 3.10.12
- GPU: NVIDIA RTX 3060 12GB
- sigyn_ai: main branch (commit abc123)

### Steps to Reproduce
1. `python src/training/train.py --config configs/training/can_detector_pihat.yaml`
2. Training starts, but crashes after 5 epochs

### Expected Behavior
Training should complete 100 epochs

### Actual Behavior
```
Epoch 5/100: CUDA out of memory
```

### Config
configs/training/can_detector_pihat.yaml:
```yaml
compute:
  batch_size: 16  # This was too large
```
```

## 🌟 Feature Requests

**Use GitHub Discussions** for:
- New feature ideas
- Architecture discussions
- Hardware compatibility questions
- Best practices sharing

**Good Feature Request**:
```markdown
**Feature**: Add Apple Silicon (M1/M2/M3) support

**Use Case**: Many Mac users want to train locally without Colab

**Proposal**:
- Detect MPS backend in train.py
- Add mps device option in configs
- Document performance vs CUDA GPUs

**Challenges**:
- MPS backend has some missing ops for YOLO
- Need Mac hardware for testing
```

## 📄 License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.

## 🙏 Recognition

Contributors will be:
- Listed in CONTRIBUTORS.md
- Mentioned in release notes
- Credited in relevant documentation

Significant contributions may be highlighted in README.md.

## 📬 Contact

- **Issues**: GitHub Issues
- **Discussions**: GitHub Discussions
- **Security**: Email maintainer directly (see README.md)

---

**Thank you for helping make robotics AI more accessible!** 🤖
