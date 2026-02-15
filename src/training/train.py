#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
"""
Training script for YOLO models across multiple device targets.

This script supports flexible hardware configurations and automatically
adjusts training parameters based on available resources.

Usage:
    python train.py --config configs/training/can_detector_pihat.yaml
    python train.py --config configs/training/multi_class_oakd.yaml
"""

import argparse
import sys
import yaml
from pathlib import Path
import torch
from ultralytics import YOLO


def detect_hardware():
    """Detect available hardware and return recommended settings."""
    device_info = {
        'cuda_available': torch.cuda.is_available(),
        'device_name': None,
        'vram_gb': 0,
        'recommended_batch': 4,
        'recommended_workers': 0
    }
    
    if torch.cuda.is_available():
        device_info['device_name'] = torch.cuda.get_device_name(0)
        device_info['vram_gb'] = torch.cuda.get_device_properties(0).total_memory / 1e9
        
        # Adjust batch size based on VRAM
        if device_info['vram_gb'] >= 16:
            device_info['recommended_batch'] = 16
            device_info['recommended_workers'] = 8
        elif device_info['vram_gb'] >= 8:
            device_info['recommended_batch'] = 8
            device_info['recommended_workers'] = 4
        elif device_info['vram_gb'] >= 6:
            device_info['recommended_batch'] = 4
            device_info['recommended_workers'] = 2
        else:
            device_info['recommended_batch'] = 2
            device_info['recommended_workers'] = 0
    else:
        print("⚠️  WARNING: No CUDA GPU detected. Training will be slow on CPU.")
        print("    Consider using Google Colab (see docs/GETTING_STARTED_NO_GPU.md)")
        device_info['recommended_batch'] = 2
        device_info['recommended_workers'] = 0
    
    return device_info


def load_config(config_path):
    """Load training configuration from YAML file."""
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def merge_hardware_config(config, hardware_info):
    """Merge hardware detection with config, applying auto settings."""
    if config.get('compute', {}).get('batch_size') == 'auto':
        config['compute']['batch_size'] = hardware_info['recommended_batch']
        print(f"📊 Auto-detected batch size: {hardware_info['recommended_batch']}")
    
    if config.get('compute', {}).get('workers') == 'auto':
        config['compute']['workers'] = hardware_info['recommended_workers']
        print(f"👷 Auto-detected workers: {hardware_info['recommended_workers']}")
    
    if config.get('compute', {}).get('device') == 'auto':
        config['compute']['device'] = 'cuda:0' if hardware_info['cuda_available'] else 'cpu'
    
    return config


def validate_dataset(dataset_path):
    """Validate that dataset exists and has proper structure."""
    dataset_path = Path(dataset_path)
    
    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found: {dataset_path}")
    
    # Check for data.yaml
    data_yaml = dataset_path / 'data.yaml'
    if not data_yaml.exists():
        raise FileNotFoundError(f"data.yaml not found in dataset: {data_yaml}")
    
    return data_yaml


def train(config):
    """Execute training with the given configuration."""
    # Validate dataset
    data_yaml = validate_dataset(config['dataset']['path'])
    
    # Initialize model
    model_size = config['model']['size']
    
    # Support both YOLOv5 and YOLOv8
    # If model_size starts with "5", it's YOLOv5 (e.g., "5n", "5s")
    # Otherwise it's YOLOv8 (e.g., "n", "s", "m")
    if model_size.startswith('5'):
        model_name = f"yolov{model_size}.pt"  # yolov5n.pt, yolov5s.pt, etc.
        version = "v5"
    else:
        model_name = f"yolov8{model_size}.pt"  # yolov8n.pt, yolov8s.pt, etc.
        version = "v8"
    
    print(f"\n🚀 Starting training with YOLO{version} ({model_size})")
    print(f"   Dataset: {config['dataset']['path']}")
    print(f"   Device: {config['compute']['device']}")
    print(f"   Batch size: {config['compute']['batch_size']}")
    print(f"   Workers: {config['compute']['workers']}")
    print(f"   Epochs: {config['training']['epochs']}")
    print(f"   Image size: {config['training']['imgsz']}")
    
    # Load model
    model = YOLO(model_name)
    
    # Training arguments
    train_args = {
        'data': str(data_yaml),
        'epochs': config['training']['epochs'],
        'imgsz': config['training']['imgsz'],
        'batch': config['compute']['batch_size'],
        'workers': config['compute']['workers'],
        'device': config['compute']['device'],
        'project': config['output']['project_dir'],
        'name': config['output']['run_name'],
        'exist_ok': True,
        'pretrained': True,
        'optimizer': config['training'].get('optimizer', 'auto'),
        'verbose': True,
        'patience': config['training'].get('patience', 50),
        'save': True,
        'save_period': config['training'].get('save_period', -1),
        'cache': config['training'].get('cache', False),
        'amp': config['compute'].get('enable_amp', True),
    }
    
    # Add augmentation settings if present
    if 'augmentation' in config:
        aug = config['augmentation']
        if 'hsv_h' in aug:
            train_args['hsv_h'] = aug['hsv_h']
        if 'hsv_s' in aug:
            train_args['hsv_s'] = aug['hsv_s']
        if 'hsv_v' in aug:
            train_args['hsv_v'] = aug['hsv_v']
        if 'degrees' in aug:
            train_args['degrees'] = aug['degrees']
        if 'translate' in aug:
            train_args['translate'] = aug['translate']
        if 'scale' in aug:
            train_args['scale'] = aug['scale']
        if 'fliplr' in aug:
            train_args['fliplr'] = aug['fliplr']
        if 'mosaic' in aug:
            train_args['mosaic'] = aug['mosaic']
    
    # Train the model
    results = model.train(**train_args)
    
    print(f"\n✅ Training complete!")
    print(f"   Best model saved to: {config['output']['project_dir']}/{config['output']['run_name']}/weights/best.pt")
    
    # Validate the model
    print("\n📊 Running validation...")
    metrics = model.val()
    
    print(f"\n📈 Final Metrics:")
    print(f"   Precision: {metrics.box.mp:.3f}")
    print(f"   Recall: {metrics.box.mr:.3f}")
    print(f"   mAP@50: {metrics.box.map50:.3f}")
    print(f"   mAP@50-95: {metrics.box.map:.3f}")
    
    return results, metrics


def main():
    parser = argparse.ArgumentParser(
        description='Train YOLO models for Sigyn AI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Train with config file
  python train.py --config configs/training/can_detector_pihat.yaml
  
  # Train with config and override device
  python train.py --config configs/training/can_detector_pihat.yaml --device cuda:0
  
  # Train on CPU (slow)
  python train.py --config configs/training/can_detector_pihat.yaml --device cpu
        """
    )
    
    parser.add_argument(
        '--config',
        type=str,
        required=True,
        help='Path to training configuration YAML file'
    )
    
    parser.add_argument(
        '--device',
        type=str,
        default=None,
        help='Override device (cuda:0, cpu, etc.)'
    )
    
    args = parser.parse_args()
    
    # Detect hardware
    print("🔍 Detecting hardware...")
    hardware_info = detect_hardware()
    print(f"   GPU: {hardware_info['device_name'] or 'Not available'}")
    if hardware_info['cuda_available']:
        print(f"   VRAM: {hardware_info['vram_gb']:.1f} GB")
    
    # Load configuration
    print(f"\n📖 Loading config: {args.config}")
    config = load_config(args.config)
    
    # Merge hardware detection with config
    config = merge_hardware_config(config, hardware_info)
    
    # Apply command-line overrides
    if args.device:
        config['compute']['device'] = args.device
        print(f"⚙️  Device override: {args.device}")
    
    # Execute training
    try:
        train(config)
    except KeyboardInterrupt:
        print("\n\n⚠️  Training interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Training failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
