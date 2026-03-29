# Building Robotics AI on a Budget

This guide helps you build capable AI vision systems for under $500 total hardware cost. Designed for students, hobbyists, and anyone building their first robot.

## 🎯 Budget Tiers

### Tier 1: Bare Minimum ($150-200)
**For: Learning, proof-of-concept, single robot**

- **Training**: Google Colab Free (T4 GPU)
- **Camera**: Raspberry Pi Camera Module 3 ($25)
- **Compute**: Raspberry Pi 5 8GB ($80)
- **Accelerator**: None (CPU inference only)
- **Storage**: 128GB microSD ($15)

**Capabilities:**
- YOLOv8n at 320x320: ~5-8 FPS
- Single class detection
- Good enough for: Slow-moving robots, stationary manipulation

**Limitations:**
- Slow inference, limited reactivity
- Cannot run larger models
- No real-time object tracking

---

### Tier 2: Practical Budget ($350-450)
**For: Serious hobby robots, small projects**

- **Training**: Google Colab Free or used RTX 2060 ($200 used)
- **Camera**: OAK-D Lite ($149)
- **Compute**: Robot's main computer (reuse existing)
- **Storage**: 256GB SSD ($30)

**Capabilities:**
- YOLOv8n at 416x416: 15-20 FPS
- Multi-class detection (5-10 classes)
- 3D spatial detection with depth
- Good enough for: Mobile robots, household tasks, navigation

**Limitations:**
- OAK-D struggles with very small objects
- Limited to ~10 classes before slowdown

**Why This Tier:**
- ✅ OAK-D includes camera + accelerator + depth sensor
- ✅ Best value per dollar for edge inference
- ✅ USB connection (no additional compute board needed)
- ✅ Widely used in community (good support)

---

### Tier 3: Serious Development ($500-700)
**For: Multiple robots, research, content creation**

- **Training**: RTX 3060 12GB ($300-350 used)
- **Camera**: OAK-D Lite ($149) OR Pi Camera ($25)
- **Compute**: Raspberry Pi 5 ($80)
- **Accelerator**: Hailo-8 AI Hat ($70)
- **Storage**: 512GB NVMe SSD ($50)

**Capabilities:**
- Training: YOLOv8m in minutes, segmentation models
- Inference: YOLOv8n at 640x640: 25-30 FPS
- Multi-class detection (20+ classes)
- Good enough for: Professional demos, research, multiple robots

**Why This Tier:**
- ✅ Local training (no Colab dependency)
- ✅ Train segmentation models and larger YOLOs
- ✅ Fast iteration (train-test-deploy cycle)
- ✅ Hailo-8 outperforms OAK-D at detection

---

## 💰 Component Breakdown

### Training Options

| Option | Cost | Speed | Pros | Cons |
|--------|------|-------|------|------|
| **Colab Free** | $0 | Medium | Free, no setup | 12hr limit, unreliable access |
| **Colab Pro** | $10/mo | Fast | V100/A100 access | Ongoing cost |
| **Used RTX 2060** | $200 | Medium | Local, unlimited | Only 6GB VRAM |
| **Used RTX 3060** | $300-350 | Fast | 12GB VRAM | Higher cost |
| **New RTX 4060 Ti 16GB** | $500 | Fast | 16GB, efficient | New price |

**Recommendation**: 
- Start with Colab Free
- Buy used RTX 3060 if training >10 hours/month

### Edge Inference Options

| Device | Cost | FPS (YOLOv8n 640) | Pros | Cons |
|--------|------|-------------------|------|------|
| **Pi 5 CPU** | $80 | 5-8 | Cheap, simple | Too slow |
| **Pi 5 + Hailo-8** | $150 | 25-30 | Fast, low power | Compilation hassles |
| **OAK-D Lite** | $149 | 15-20 | Includes camera, depth | Smaller models only |

**Recommendation**:
- **Best value**: OAK-D Lite ($149 for camera + compute + depth)
- **Best performance**: Pi 5 + Hailo-8 ($150 compute + accelerator only)

### Camera Options

| Camera | Cost | Resolution | FPS | Notes |
|--------|------|------------|-----|-------|
| **Pi Camera Module 3** | $25 | 11.9MP | 30 | Great value, wide compatibility |
| **Pi Camera Module 3 Wide** | $35 | 11.9MP | 30 | 120° FOV for navigation |
| **USB Webcam** | $20-50 | 1080p | 30 | Easy, generic |
| **OAK-D Lite** | $149 | 4MP + depth | 30 | Includes accelerator |

**Recommendation**: Start with Pi Camera Module 3 ($25)

---

## 🛒 Budget Shopping Lists

### $200 Starter Kit

```
Google Colab Free            $0
Raspberry Pi 5 8GB          $80
Pi Camera Module 3          $25
Power supply (Pi 5)         $12
128GB microSD + case        $20
USB keyboard/mouse          $15
HDMI cable (have one?)       $8
-----------------------------------
TOTAL:                     $160
```

**What you can do:**
- Train YOLOv8n models on Colab
- Run inference at 5-8 FPS on Pi CPU
- Learn the complete workflow
- Prototype and test algorithms

**Upgrade path:**
- Add Hailo-8 AI Hat ($70) → 25 FPS
- OR Buy OAK-D Lite ($149) → 15 FPS + depth

---

### $450 Practical Kit

```
Google Colab Free            $0
Raspberry Pi 5 8GB          $80
OAK-D Lite                 $149
256GB SSD + adapter         $35
Power supply + case         $30
SD card (for Pi OS)         $15
-----------------------------------
TOTAL:                     $309

OR with local training:
Used RTX 2060 6GB          $200
(same as above)            $309
-----------------------------------
TOTAL:                     $509
```

**What you can do:**
- Train locally or on Colab
- 15-20 FPS real-time detection
- 3D spatial localization with depth
- Multi-class detection (10+ classes)
- Build complete mobile robot

**Upgrade path:**
- Add second OAK-D ($149) for dual cameras
- Upgrade GPU to RTX 3060 ($350) for segmentation

---

### $700 Serious Kit

```
RTX 3060 12GB (used)       $350
Raspberry Pi 5 8GB          $80
Hailo-8 AI Hat              $70
Pi Camera Module 3          $25
512GB NVMe SSD              $50
Power supplies + cables     $30
32GB RAM for PC (if needed) $60
-----------------------------------
TOTAL:                     $665
```

**What you can do:**
- Train any YOLO model locally (n/s/m/l/x)
- Train segmentation models
- 25-30 FPS real-time detection
- Multiple robots (reuse training PC)
- Fast iteration (5 min training → deploy)

---

## 🧮 Cost Over Time Analysis

### Year 1 Costs

**Colab-Only Approach:**
- Hardware: $160 (Pi + camera)
- Colab Pro: $120/year ($10/mo)
- **Total Year 1**: $280

**Local GPU Approach:**
- Hardware: $380 (Pi + camera + RTX 2060)
- Electricity: ~$50/year (training 10hr/week)
- **Total Year 1**: $430

**Break-even: ~15 months** (Colab Pro vs local GPU)

**But consider:**
- Local GPU: Unlimited training, no session limits
- Colab Pro: No upfront cost, reliable access to latest GPUs
- Local GPU: Resale value (~$150 after 2 years)

---

## 💡 Money-Saving Tips

### 1. Buy Used GPUs
- eBay, Craigslist, Facebook Marketplace
- Check seller reputation
- Test before buying (run benchmarks)
- Aim for RTX 2000/3000 series (good VRAM/price)

### 2. Reuse Existing Hardware
- Old gaming PC? Add a used GPU
- Have a laptop? Use Colab for training
- Existing Raspberry Pi 4? Start there (upgrade to Pi 5 later)

### 3. Start Small
- Train on Colab (free)
- Deploy on Pi 5 CPU (slow but functional)
- Upgrade accelerator when budget allows

### 4. Share Resources
- Training PC can serve multiple robots
- One OAK-D can be swapped between test robots
- Co-working space GPU (if available)

### 5. Consider Alternatives
- **RoboFlow hosted training**: $0-50/mo (no local GPU needed)
- **Google Cloud / AWS**: Pay per use (~$0.50-2.00/hr for GPU)
- **University resources**: Free compute if you're a student

---

## 📊 Performance vs Cost

```
                      FPS (YOLOv8n 640x640)
50 |                            ● Pi+Hailo ($150)
40 |
30 |
20 |                    ● OAK-D ($149)
10 |            ● Pi5 CPU ($80)
 0 |___________________________________________________
   $0    $50   $100   $150   $200   $250   $300   $350
                        Cost
```

**Sweet Spots:**
- **$149**: OAK-D Lite (best value for beginners)
- **$150**: Pi 5 + Hailo-8 (if you already have camera)
- **$350**: RTX 3060 for training (if serious about multiple robots)

---

## 🎓 Student / Educator Discounts

### Hardware
- **Raspberry Pi**: Bulk pricing for classrooms
- **GitHub Student Pack**: Free Colab Pro credits (sometimes)

### Software
- **RoboFlow**: Free Pro tier for students (.edu email)
- **Weights & Biases**: Free for academics
- **Google Cloud**: $300 free credits for education

### Communities
- **First Robotics**: Sponsorships for hardware
- **Hackster.io**: Hardware grants for projects
- **Local makerspaces**: Shared tools and compute

---

## 🤔 Decision Tree

```
Do you have any existing hardware?
├─ Yes, gaming PC with PCIe slot
│  └─ Buy used RTX 3060 ($350) → Train locally
├─ Yes, any computer
│  └─ Buy OAK-D Lite ($149) → Train on Colab
└─ No, starting from scratch
   └─ Buy Pi 5 + camera ($105) → Train on Colab
      └─ Later: Add Hailo-8 ($70) OR OAK-D ($149)
```

---

## ✅ My Recommendation

**For most people starting out:**

**Phase 1: Learn ($160)**
- Pi 5 + Pi Camera
- Google Colab Free
- Deploy at 5-8 FPS (acceptable for learning)

**Phase 2: Make it Real ($+150)**
- Add Hailo-8 AI Hat → 25 FPS
- OR Add OAK-D Lite → 15 FPS + depth + better camera

**Phase 3: Scale ($+350)**
- Buy used RTX 3060 12GB
- Now you can: Train large models, multiple robots, fast iteration

**Total over 6-12 months: $660**

This gets you:
- Complete training pipeline
- Real-time inference (25-30 FPS)
- Depth sensing (if OAK-D path)
- Multi-class detection
- Segmentation capable
- Professional results

---

**Remember**: The best hardware is the hardware you can afford NOW to start learning. You can always upgrade later!
