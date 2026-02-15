# Pushing Sigyn AI to GitHub

## Step-by-Step Guide

### 1. Initialize Git Repository (if not done)

```bash
cd ~/sigyn_ai

# Initialize git
git init

# Add all files
git add .

# Check what will be committed
git status
```

### 2. Create Initial Commit

```bash
git commit -m "Initial commit: Sigyn AI training and deployment toolkit

Complete vision AI workflow for household robotics:
- Training pipeline with GPU auto-detection
- Multi-device export (Pi5/Hailo, OAK-D, Jetson)  
- Automated SSH deployment with rollback
- RoboFlow integration for dataset management
- Docker workflows for compilation
- Comprehensive documentation (budget guide, no-GPU guide)
- Apache 2.0 license for community use

Supports:
- YOLOv8 object detection
- Hardware: RTX 2060/3060/4060 Ti, Google Colab
- Edge devices: Raspberry Pi 5 + Hailo-8, OAK-D Lite, Jetson Orin Nano
- Target FPS: 15-30 depending on device"
```

### 3. Create GitHub Repository

**Via GitHub Web UI**:
1. Go to https://github.com/new
2. Repository name: `sigyn_ai`
3. Description: "Vision AI training and deployment toolkit for household service robots"
4. Public or Private: **Public** (for community)
5. **DO NOT** initialize with README, .gitignore, or license (you already have these)
6. Click "Create repository"

### 4. Add Remote and Push

GitHub will show you commands. Use HTTPS or SSH:

**Option A: HTTPS** (easier for first time)
```bash
git remote add origin https://github.com/YOUR_USERNAME/sigyn_ai.git
git branch -M main
git push -u origin main
```

**Option B: SSH** (recommended for frequent use)
```bash
# First time: Set up SSH key if not done
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub
# Copy output and add to GitHub: Settings → SSH and GPG keys → New SSH key

# Then push
git remote add origin git@github.com:YOUR_USERNAME/sigyn_ai.git
git branch -M main
git push -u origin main
```

### 5. Verify Upload

```bash
# Check remote
git remote -v

# Should show:
# origin  https://github.com/YOUR_USERNAME/sigyn_ai.git (fetch)
# origin  https://github.com/YOUR_USERNAME/sigyn_ai.git (push)
```

Visit: `https://github.com/YOUR_USERNAME/sigyn_ai`

---

## Troubleshooting

### "Remote origin already exists"
```bash
# Remove old remote
git remote remove origin

# Add new one
git remote add origin https://github.com/YOUR_USERNAME/sigyn_ai.git
```

### "Large files detected"
```bash
# Check .gitignore is working
git status

# Should NOT see files in:
# - datasets/
# - models/
# - *.hef, *.blob, *.engine, *.pt files

# If they're staged, unstage:
git reset HEAD datasets/ models/
git checkout -- .gitignore
```

### "Permission denied (publickey)"
```bash
# Test SSH connection
ssh -T git@github.com

# If fails, use HTTPS instead or set up SSH key
```

---

## Recommended GitHub Settings

### 1. Add Topics

Go to: Repository → About → Settings (⚙️)

Add topics:
- `robotics`
- `computer-vision`
- `yolo`
- `object-detection`
- `ros2`
- `edge-ai`
- `raspberry-pi`
- `jetson`
- `household-robotics`

### 2. Enable Discussions

Settings → General → Features → ✅ Discussions

Create categories:
- **Q&A**: Help and questions
- **Show and Tell**: Share your robots
- **Ideas**: Feature requests
- **General**: Everything else

### 3. Set Up Issue Templates

Create `.github/ISSUE_TEMPLATE/`:

**bug_report.md**:
```markdown
---
name: Bug Report
about: Report a problem
---

## Environment
- OS:
- Python version:
- GPU:

## Steps to Reproduce
1. 
2. 
3. 

## Expected Behavior

## Actual Behavior

## Config Files
```yaml
# Paste relevant config
```

**feature_request.md**:
```markdown
---
name: Feature Request
about: Suggest an enhancement
---

## Feature Description

## Use Case

## Proposed Implementation

## Alternatives Considered
```

### 4. Add GitHub Actions (Optional)

`.github/workflows/test.yml`:
```yaml
name: Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      - run: pip install -r requirements.txt
      - run: pytest tests/
```

---

## After Pushing

### 1. Update README with Your GitHub URL

```bash
# Replace "yourusername" in README.md with your actual username
sed -i 's/yourusername/YOUR_ACTUAL_USERNAME/g' README.md docs/*.md

git add README.md docs/
git commit -m "docs: Update GitHub URLs with actual username"
git push
```

### 2. Create Release (v1.0.0)

GitHub → Releases → "Create a new release"

**Tag**: `v1.0.0`  
**Title**: `v1.0.0 - Initial Release`  
**Description**:
```markdown
# Sigyn AI v1.0.0 - Initial Release

Complete vision AI training and deployment toolkit for robotics!

## Features

✅ Training pipeline with GPU auto-detection  
✅ Multi-device export (Pi5/Hailo, OAK-D, Jetson)  
✅ Automated deployment with SSH and rollback  
✅ RoboFlow integration  
✅ Docker workflows  
✅ Comprehensive documentation  

## Getting Started

See [GETTING_STARTED.md](docs/GETTING_STARTED.md)

## Supported Hardware

**Training**: NVIDIA GPUs (RTX 2060+), Google Colab  
**Inference**: Raspberry Pi 5 + Hailo-8, OAK-D Lite, Jetson Orin Nano

## Installation

```bash
git clone https://github.com/YOUR_USERNAME/sigyn_ai.git
cd sigyn_ai
pip install -r requirements.txt
```

See [README.md](README.md) for full documentation.
```

### 3. Share Your Work!

**Reddit**:
- r/robotics: "Released an open-source AI training toolkit for robots"
- r/ROS: "Sigyn AI: YOLOv8 training to edge deployment for ROS 2 robots"

**ROS Discourse**:
- Category: Projects and Demos
- Title: "[Project] Sigyn AI - Vision training and deployment toolkit"

**Twitter/X**:
```
🤖 Just released Sigyn AI - an open-source toolkit for training and deploying YOLO models on robots!

✅ Works with budget hardware (RTX 2060, Colab)
✅ Multi-device support (Pi5/Hailo, OAK-D, Jetson)
✅ Complete workflow: RoboFlow → Train → Deploy

Apache 2.0 license

https://github.com/YOUR_USERNAME/sigyn_ai

#ROS2 #Robotics #AI #OpenSource
```

---

## Maintenance Workflow

### Making Updates

```bash
# Make changes
# ... edit files ...

# Stage and commit
git add .
git commit -m "feat: Add YOLOv9 support"

# Push
git push
```

### Branching for Features

```bash
# Create feature branch
git checkout -b feature/yolov9-support

# Make changes and commit
git add .
git commit -m "feat: Add YOLOv9 model support"

# Push feature branch
git push -u origin feature/yolov9-support

# Create PR on GitHub
# After merge, delete branch
git checkout main
git pull
git branch -d feature/yolov9-support
```

---

## Quick Reference

**Clone** (others using your repo):
```bash
git clone https://github.com/YOUR_USERNAME/sigyn_ai.git
cd sigyn_ai
pip install -r requirements.txt
```

**Update** (you making changes):
```bash
git add .
git commit -m "type: description"
git push
```

**Pull latest** (sync from GitHub):
```bash
git pull
```

---

**Your repo URL will be**: `https://github.com/YOUR_USERNAME/sigyn_ai`

Replace `YOUR_USERNAME` with your actual GitHub username!
