# sigyn_ai Deployment Guide

## Overview

`sigyn_ai` is **not** a ROS package. It is a training and model deployment toolchain
for the Sigyn robot's vision subsystem. It lives alongside ROS packages in the workspace
`src/` directory, but `colcon build` ignores it because there is no `package.xml`.

## Repository Layout on sigynVision (SV)

```
~/sigyn_vision_ws/
  src/
    sigyn_ai/          ← this repo (toolchain, no package.xml — colcon ignores it)
    pi_can_detector/   ← ROS 2 package (colcon builds this)
    pi_gripper/        ← ROS 2 package (colcon builds this)
  sigyn_vision.repos   ← vcstool manifest listing all three repos above
```

`colcon` will only build `pi_can_detector` and `pi_gripper` because those are the only
directories that contain a `package.xml`. `sigyn_ai` is silently skipped.

## Full Deployment Sequence for sigynVision

```bash
# Step 1: Clone all repos (done by setup_robot.py in Sigyn2)
cd ~/sigyn_vision_ws/src
vcs import < ~/sigyn_vision_ws/sigyn_vision.repos

# Step 2: Build ROS packages
cd ~/sigyn_vision_ws
colcon build

# Step 3: Deploy the AI model (required before starting pi-can-detector service)
cd ~/sigyn_vision_ws/src/sigyn_ai
./scripts/deploy_pi5_hailo.sh -m <model_name>

# Step 4: Install and start services (done by setup_robot.py in Sigyn2)
~/sigyn_vision_ws/src/pi_can_detector/scripts/setup_service.sh
~/sigyn_vision_ws/src/pi_gripper/scripts/setup_service.sh
```

### Step 3 in Detail

`deploy_pi5_hailo.sh` copies the compiled Hailo `.hef` (and optionally `.onnx` and
`labels.txt`) to `~/sigyn_vision_ws/src/pi_can_detector/models/` on the target machine
and creates a stable symlink `can_detector.hef` pointing to the versioned file.

**Prerequisites for Step 3:**

- `pi_can_detector` must already be cloned at `~/sigyn_vision_ws/src/pi_can_detector/`
  (Step 1 handles this via `vcs import`).
- SSH key-based access to `ros@sigynVision` must be configured on the machine running
  this script.
- A compiled model must exist locally under `models/exported/<model_name>/pi5_hailo8/`.

## Notes on Hailo Runtime

Hailo inference on sigynVision uses the **system-wide** `python3-hailort` package.
There is no virtual environment to activate. The `pi-can-detector` systemd service
runs under the system Python directly.

## Service Management

After deployment, the `pi-can-detector` service can be managed with:

```bash
sudo systemctl start   pi-can-detector
sudo systemctl stop    pi-can-detector
sudo systemctl status  pi-can-detector
sudo systemctl enable  pi-can-detector   # start on boot
sudo journalctl -u pi-can-detector -f    # follow logs
```

## Model Rollback

If a newly deployed model causes problems, roll back using the backup created
automatically by `deploy_pi5_hailo.sh`:

```bash
ssh ros@sigynVision
cd ~/sigyn_vision_ws/src/pi_can_detector/models
ls -lh                                     # find the .backup.<timestamp> file
ln -sf <backup_filename>.hef can_detector.hef
sudo systemctl restart pi-can-detector
```
