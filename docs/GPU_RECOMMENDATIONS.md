# GPU Recommendations for Mini ITX Systems

## Recommended GPUs for Sigyn AI Training

### 🥇 Best Value: RTX 3060 12GB
**Price**: $300-350 used, $350-400 new  
**VRAM**: 12GB GDDR6  
**Power**: 170W TDP  
**Mini ITX Compatible**: ✅ Yes (dual-slot, 242mm)

**Why This One**:
- 12GB VRAM (more than RTX 3070/3080 8GB)
- Handles YOLOv8m and segmentation models
- Excellent used market availability
- Lower power than 3070/3080

**Training Performance**:
- YOLOv8n (100 epochs, 640x640, 100 images): ~2 minutes
- YOLOv8s: ~3-4 minutes
- YOLOv8m: ~6-8 minutes
- Segmentation models: Feasible

---

### 🥈 Best New Option: RTX 4060 Ti 16GB
**Price**: $500-550 new  
**VRAM**: 16GB GDDR6  
**Power**: 165W TDP  
**Mini ITX Compatible**: ✅ Yes (dual-slot, 244mm)

**Why This One**:
- 16GB handles ALL YOLO variants
- More efficient (lower power than 30-series)
- Latest architecture (Ada Lovelace)
- AV1 encoding for video augmentation

**Training Performance**:
- YOLOv8n: ~1.5 minutes
- YOLOv8s: ~2-3 minutes
- YOLOv8m: ~4-5 minutes
- YOLOv8l: Possible (~8-10 min)

---

### 💰 Budget Option: RTX 2060 6GB (Your Current GPU)
**Price**: $180-220 used  
**VRAM**: 6GB GDDR6  
**Power**: 160W TDP  
**Mini ITX Compatible**: ✅ Yes (dual-slot, 229mm)

**Limitations**:
- Only handles YOLOv8n/s comfortably
- YOLOv8m at reduced batch sizes
- Segmentation models struggle
- Small datasets only (<500 images)

**When to Upgrade**:
- Training segmentation models
- Multi-class with >10 classes
- Datasets >500 images
- Want faster iteration

---

### 🚫 Avoid These (Mini ITX Issues)

**RTX 3080/3090/4080/4090**:
- Too large (3-slot, >300mm)
- Too much power (320-450W)
- Won't fit in most Mini ITX cases
- Overkill for robotics training

**AMD GPUs**:
- ROCm support improving but not as mature
- Ultralytics YOLO optimized for CUDA
- Harder to troubleshoot issues

---

## Mini ITX Compatibility Checklist

### GPU Physical Constraints
- **Slots**: 2-2.5 slots max (most Mini ITX cases)
- **Length**: <270mm (check your case spec)
- **Height**: <140mm for SFF cases

**Your Case**: Check manufacturer specs for:
1. Maximum GPU length
2. Clearance with side panel
3. Clearance with CPU cooler

### Power Supply Requirements

#### RTX 2060 (160W)
- **Minimum PSU**: 450W
- **Recommended PSU**: 550W
- **Connector**: 1x 8-pin PCIe

#### RTX 3060 (170W)
- **Minimum PSU**: 550W
- **Recommended PSU**: 650W
- **Connector**: 1x 8-pin PCIe

#### RTX 4060 Ti (165W)
- **Minimum PSU**: 550W
- **Recommended PSU**: 650W
- **Connector**: 1x 8-pin PCIe (some models) or 12VHPWR (new connector)

### Complete System Power Budget

**Example with RTX 3060**:
```
CPU (AMD 7900X):     ~120W (typical load)
GPU (RTX 3060):      ~170W (training load)
Motherboard:         ~50W
RAM (32GB):          ~20W
Storage (NVMe SSD):  ~10W
Fans/Peripherals:    ~20W
-----------------------------------
Total:               ~390W
Recommended PSU:     650W (60% load = efficient)
```

**Why Overhead Matters**:
- PSUs most efficient at 50-70% load
- Transient power spikes can exceed TDP
- Room for future upgrades
- Longer PSU lifespan

---

## Recommended Power Supplies for Mini ITX

### SFX Form Factor (Most Common)

**Budget**: Corsair SF550 (550W, Gold)
- Price: ~$100
- Sufficient for RTX 3060
- 7-year warranty

**Recommended**: Corsair SF750 (750W, Platinum)
- Price: ~$150
- Headroom for RTX 4060 Ti or upgrades
- Ultra-quiet fan
- 7-year warranty

**Premium**: Cooler Master V850 SFX (850W, Gold)
- Price: ~$160
- Maximum headroom
- Fully modular

### Checking Your Current PSU

```bash
# On Linux, check 12V rail capacity (critical for GPU)
sudo dmidecode -t 39
```

Look for:
- **Total wattage**: Should be ≥550W for RTX 3060
- **12V rail amperage**: Should be ≥40A for RTX 3060
- **Efficiency rating**: 80+ Bronze minimum, Gold preferred

---

## My Specific Recommendation for You

Based on your setup (AMD 7900X, Mini ITX):

### Option 1: Upgrade GPU Now (Best ROI)
**Buy**: Used RTX 3060 12GB (~$320)  
**PSU**: Keep current if ≥550W, otherwise upgrade to SF650  
**Total**: $320 (GPU only) or $420 (GPU + PSU)

**Benefits**:
- 2x VRAM (12GB vs 6GB)
- Train segmentation models
- Faster iteration
- Handle larger datasets

### Option 2: Wait and Save for Top Tier
**Buy**: RTX 4060 Ti 16GB new (~$520)  
**PSU**: Upgrade to SF650 or SF750 (~$120)  
**Total**: ~$640

**Benefits**:
- 2.5x VRAM (16GB vs 6GB)
- Latest architecture
- Future-proof for 3+ years
- Best efficiency

### Option 3: Keep 2060, Use Colab More
**Cost**: $0 now, $10/month for Colab Pro  
**Break-even**: ~15 months vs buying RTX 3060

**When This Makes Sense**:
- Training <10 hours/month
- Budget extremely tight right now
- Building multiple robots (training PC shared)

---

## Verification Before Purchase

### Check Your Case

```bash
# Measure your case
# 1. GPU length clearance (use ruler)
# 2. Width (slots) - should be 2-2.5 slots
# 3. Check case manual for GPU compatibility list
```

**Common Mini ITX Cases**:
- **NZXT H1**: Max 324mm (any GPU works)
- **Fractal Design Node 202**: Max 310mm, 2-slot (most work)
- **Cooler Master NR200**: Max 330mm, 3-slot (all work)
- **Silverstone SG13**: Max 267mm, 2-slot (tight fit)

### Check Your PSU

```bash
# Look at PSU label for:
# 1. Total wattage
# 2. 12V rail amperage
# 3. Available PCIe power connectors
```

**Rule of Thumb**: 
- RTX 2060/3060/4060 Ti: Need 1x 8-pin PCIe power
- Your PSU must have this connector available

---

## Shopping Tips

### Buying Used GPUs

**Where**: eBay, r/hardwareswap, Facebook Marketplace, Craigslist  
**Price Check**: Use eBay "Sold listings" for market rate

**Red Flags**:
- "Untested" or "For parts"
- No photos of actual card
- Mining history (check seller)
- Price too good to be true

**What to Ask**:
1. "Was this used for mining?" (Prefer gaming use)
2. "Does it have warranty remaining?"
3. "Can I test before buying?" (Local only)

**Test When You Get It**:
```bash
# Stress test
pip install gpustat
watch -n 1 gpustat

# In another terminal, run training for 30 min
python src/training/train.py --config configs/training/can_detector_pihat.yaml

# Monitor temps (should be <80°C)
# Check for artifacts, crashes
```

---

## Decision Matrix

| GPU | VRAM | Price | Mini ITX | PSU | Best For |
|-----|------|-------|----------|-----|----------|
| **RTX 2060 6GB** | 6GB | $200 | ✅ | 550W | Current, ok for nano models |
| **RTX 3060 12GB** | 12GB | $330 | ✅ | 650W | **Best value upgrade** |
| **RTX 4060 Ti 16GB** | 16GB | $520 | ✅ | 650W | **Best new, future-proof** |

**My Recommendation**: RTX 3060 12GB used (~$320) + verify your PSU is ≥550W

---

**Questions?** Check GPU physically fits in your case before buying!
