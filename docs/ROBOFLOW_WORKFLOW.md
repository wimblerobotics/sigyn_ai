# RoboFlow Workflow Guide

This guide covers the complete lifecycle of a training dataset using
[Roboflow](https://roboflow.com): account setup, dataset creation, image
capture, annotation, versioning, and handing off to the training scripts.

The Sigyn workflows treat RoboFlow as the **single source of truth** for every
training image — both original captures and augmented versions are tracked
there, so any collaborator can reproduce the exact dataset used for each trained
model.

---

## 1. Prerequisites

| What | Where |
|------|-------|
| RoboFlow free account | https://app.roboflow.com/sign-up |
| RoboFlow API key | https://app.roboflow.com/settings/api |

There is **no cost** for the free tier as long as your dataset is public, which
is fine for a household-robotics project. (Private datasets require a paid
plan.)

### Store your API key locally

```bash
# Add to ~/.bashrc so it persists across sessions
echo 'export ROBOFLOW_API_KEY="your_key_here"' >> ~/.bashrc
source ~/.bashrc

# Verify
echo $ROBOFLOW_API_KEY
```

The file `roboflow_api_key.txt` is listed in `.gitignore` if you prefer to
store it there instead, but the environment variable approach is safer and is
what all the training scripts expect.

---

## 2. Create a Project

1. Log in at https://app.roboflow.com  
2. Click **New Project** (top right)  
3. Fill in the form:
   - **Project Name**: a slug you'll use everywhere (e.g. `can-detector`)
   - **Annotation Group**: leave as default or name it after your object class
   - **Project Type**: `Object Detection`  
4. Click **Create Public Project** (free tier) or **Create Private Project**

> **Sigyn Note**: The active project is **fcc4** (`can-detector` style name),
> which detects aluminium soda/food cans for the
> [Can-Do Challenge](https://github.com/wimblerobotics/can_do_challenge).

---

## 3. Capture Training Images

### Option A — On-Robot Capture (Recommended)

Sigyn's bluetooth joystick triggers on-demand captures while you manually
position cans in the robot's environment.

#### How it works — the full chain

The joystick node ([`wimblerobotics/sigyn_bluetooth_joystick`](https://github.com/wimblerobotics/sigyn_bluetooth_joystick),
`src/joystick_node.cpp`) publishes a `std_msgs/Bool` message on button press:

| Physical Button | ROS 2 topic published | Who subscribes |
|-----------------|----------------------|--------------------|
| **B** | `/sigyn/take_gripper_picture` | `pi_can_detector` node |
| **X** | `/sigyn/take_oakd_picture` | sigyn_oakd_detection


#### Pi camera / gripper camera (B button)

The subscriber is in
[`wimblerobotics/pi_can_detector`](https://github.com/wimblerobotics/pi_can_detector)
— `pi_can_detector/pi_can_detector_node.py`, method `capture_image_callback()`:

```python
def capture_image_callback(self, msg: Bool):
    """Saves the most-recent camera frame to training_images_dir when
    msg.data is True (published by the B button on the bluetooth joystick)."""
    if not msg.data:
        return
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filename = f'gripper_image_{timestamp}.jpg'
    filepath = os.path.join(self.training_images_dir, filename)
    cv2.imwrite(filepath, self.current_frame)
```

- Saves to: `~/training_images/gripper_image_<YYYYMMDD_HHMMSS_ffffff>.jpg`  
  (override the default with the `training_images_dir` ROS parameter)
- The node must be running (`pi_can_detector.launch.py`) for captures to work.

#### OAK-D camera (X button)

- Saves to: ~/sigyn_ws/training_images/oakd/oakd_capture_<YYYYMMDD_HHMMSS_ffffff>.jpg
- The node must be running (`oakd_detector.launch.py`) for captures to work.


**Workflow for Pi camera captures**:
```bash
# On the robot's Pi (pi_can_detector must be running as a service or in a terminal)
# Check the service is active:
systemctl --user status pi-can-detector

# On the dev PC — collect captures by driving the robot near objects and pressing B
# Images accumulate on the Pi at ~/training_images/

# Pull them to sigyn_ai for upload
rsync -av ros@sigyn-pi.local:~/training_images/ datasets/raw_captures/pi_gripper/
```

### Option B — Manual / Video Extracti
For bootstrapping or when the robot is not available:

```bash
# Extract frames from a video at 1 fps
ffmpeg -i my_video.mp4 -vf fps=1 datasets/raw_captures/bootstrap/%04d.jpg

# Or just point a webcam at your objects and capture at a fixed interval
v4l2-ctl --device /dev/video0 \
    --set-fmt-video=width=640,height=480,pixelformat=MJPG \
    --stream-mmap --stream-to=datasets/raw_captures/bootstrap/%04d.jpg \
    --stream-count=100
```

### Tips for Good Training Images

- Vary lighting: good light, backlighting, shadows, fluorescent vs natural
- Vary distance: 0.3 m to 2 m for gripper camera, 0.5 m to 4 m for top camera
- Vary background: different floor types, furniture, walls
- Vary object orientation: upright, tilted, partially occluded
- 100–200 unique images is often enough for single-class detection

---

## 4. Upload Images to RoboFlow

1. Inside your project page, click **Upload** (top nav bar)  
2. Choose **Select Files** or drag-and-drop your image folder  
3. Click **Save and Continue** once the upload finishes  
4. You land on the **Annotate** tab — images are not yet annotated

---

## 5. Annotate Images

### Auto-Annotate (Fastest)

Roboflow provides a built-in auto-annotator powered by a foundation model:

1. From the **Annotate** tab, click **Auto Label**  
2. Select **Segment Anything** or **Grounding DINO** (free tier has usage limits)
3. For each batch, type your class name (e.g. `can`) and let it run  
4. **Review every annotation!** Auto-label is fast but never perfect — a
   wrong annotation hurts more than a missing one  
5. Correct bounding boxes by clicking and dragging corners  
6. Click **Save** after each image  

### Manual Annotate

1. From the **Annotate** tab, open an image  
2. Press `B` to select the bounding box tool (or click the box icon)
3. Draw a tight rectangle around the object  
4. Type the class label and press Enter  
5. Use arrow keys to advance to the next image

> **Shortcut**: After the first image, Roboflow's keyboard shortcut `←→` lets
> you step through images quickly.

---

## 6. Generate a Dataset Version

Each version freezes a snapshot of images + annotations with specific
pre-processing and augmentation applied. You create **two separate dataset
versions** — one per target device — because they use different input sizes.

### Version for Pi 5 + Hailo-8 AI HAT (512 × 512)

From your project page:

1. Click **Generate** (left sidebar) → **Create New Version**  
2. **Preprocessing** tab (apply in order):
   - **Auto-Orient**: ✅ (corrects phones/cameras that embed rotation)
   - **Resize**: `512 × 512`, method `Stretch to`  
   - **Auto-Adjust Contrast**: `Adaptive Equalization`  
3. **Augmentation** tab — add the following:
   - Rotation: `-10° to +10°`
   - Bounding Box **Brightness**: `-20% to +20%`
   - Bounding Box **Blur**: up to `2.5 px`
   - Bounding Box **Motion Blur**: length `100 px`, angle `0°`, frames `1`
4. **Generate** tab:
   - Train / Valid / Test split: `80% / 10% / 10%`
   - Click **Generate** and wait (a few minutes for 200 images)

The version name in this repo follows `fcc4.v<N>-yolov8` (e.g. `fcc4.v7-yolov8`).

### Version for OAK-D / OAK-D Lite (416 × 416)

Repeat the same steps but:

- **Resize**: `416 × 416`, method `Stretch to`  
- Keep all other preprocessing and augmentation settings identical  

> **OAK-D vs OAK-D Lite sizing note**: The full OAK-D and OAK-D Lite both
> use the Intel Myriad X VPU and both accept 416 × 416 blobs. No change is
> needed between the two hardware variants.

---

## 7. Download the Dataset

After the version generates, download it to this repo's `datasets/` directory.

### Using the Download Script (Recommended)

```bash
# Pi 5 / Hailo-8 version (example: version 7)
python src/utils/roboflow_download.py --project fcc4 --version 7 --format yolov8

# OAK-D version (example: version 6)
python src/utils/roboflow_download.py --project fcc4 --version 6 --format yolov8
```

The script reads `ROBOFLOW_API_KEY` from your environment and saves to:

```
datasets/roboflow_exports/fcc4.v7-yolov8/
    data.yaml
    train/
    valid/
    test/
```

### Manually (if you prefer)

1. From your project, click the version in the left sidebar  
2. Click **Export Dataset** → format **YOLOv8** → **Download zip to computer**  
3. Unzip into `datasets/roboflow_exports/<project>.v<N>-yolov8/`

---

## 8. Train

Once the dataset is downloaded, run the appropriate training pipeline:

```bash
# Pi 5 + Hailo-8 (one command: download → train → export → compile)
./scripts/train_pi5_hailo.sh -v <version_number> -n <run_name>

# OAK-D (one command: download → train → export → compile)
./scripts/train_oakd.sh -v <version_number> -n <run_name>
```

Or, if you already have the dataset downloaded and just want to train:

```bash
python src/training/train.py --config configs/training/PiHat512c.yaml
python src/training/train.py --config configs/training/oakd_v6_rtx3060.yaml
```

See [Getting Started](GETTING_STARTED.md) for the full step-by-step.

---

## 9. Adding New Images to an Existing Version

When you capture more images (e.g. with the joystick), you add them to the same
RoboFlow project and create a new version rather than modifying the old one.
This preserves the provenance of every trained model.

### Steps

1. **Upload** new images: go to your project → **Upload** → drag new images  
2. **Annotate**: use Auto Label or manual annotation as in §5  
3. **Generate a new version**: click **Generate** → **Create New Version**  
   - Keep the same preprocessing and augmentation settings  
   - Roboflow will **combine** the new images with all previous approved images  
4. **Note the new version number** (e.g. v8)  
5. **Download and retrain**:

```bash
./scripts/train_pi5_hailo.sh -v 8 -n PiHat512_v8
```

> **AI-assisted retraining**: You can say to the AI: *"RoboFlow version 8 is
> ready. Download and run the full Pi Hat training pipeline."* The AI can
> execute all steps in order: download → train → export → compile.

---

## 10. Viewing Your API Usage / Limits

The free RoboFlow tier limits auto-label calls and private datasets but has no
quota on uploads, annotations, or exports for public projects.  
Check your usage at https://app.roboflow.com/settings/billing.

---

## Reference — Important RoboFlow Links

| Resource | URL |
|----------|-----|
| Dashboard | https://app.roboflow.com |
| API Key | https://app.roboflow.com/settings/api |
| Docs: Upload | https://docs.roboflow.com/datasets/adding-data |
| Docs: Annotate | https://docs.roboflow.com/annotate |
| Docs: Augment | https://docs.roboflow.com/datasets/create-a-dataset-version |
| Docs: Python API | https://docs.roboflow.com/api-reference/python |
| Roboflow Universe (pre-built datasets) | https://universe.roboflow.com |
