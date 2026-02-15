#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
"""
Helper script to run Hailo compilation inside Docker container.

This automates the process of compiling ONNX models to Hailo .hef format.

Usage:
    python run_hailo_compile.py --onnx models/exported/best/pi5_hailo8/best.onnx
"""

import argparse
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='Compile ONNX to Hailo .hef using Docker')
    
    parser.add_argument(
        '--onnx',
        type=str,
        required=True,
        help='Path to ONNX model'
    )
    
    parser.add_argument(
        '--docker-image',
        type=str,
        default='hailo8_ai_sw_suite_2025-10',
        help='Hailo Docker image name'
    )
    
    parser.add_argument(
        '--optimization',
        type=int,
        default=2,
        choices=[0, 1, 2],
        help='Optimization level (0=debug, 1=balanced, 2=performance)'
    )
    
    args = parser.parse_args()
    
    onnx_path = Path(args.onnx).resolve()
    if not onnx_path.exists():
        print(f"❌ ONNX file not found: {onnx_path}")
        sys.exit(1)
    
    output_dir = onnx_path.parent
    output_hef = output_dir / f"{onnx_path.stem}.hef"
    
    # Build docker command
    docker_cmd = [
        'docker', 'run',
        '--rm',
        '-v', f'{output_dir}:/workspace',
        args.docker_image,
        'hailo', 'compile',
        '--model', f'/workspace/{onnx_path.name}',
        '--hw-arch', 'hailo8',
        '--optimization-level', str(args.optimization),
        '--batch-size', '1',
        '--compression',
        '--output', f'/workspace/{onnx_path.stem}.hef'
    ]
    
    print(f"🐳 Running Hailo compilation in Docker...")
    print(f"   ONNX: {onnx_path.name}")
    print(f"   Output: {output_hef}")
    print(f"   Command: {' '.join(docker_cmd)}")
    
    try:
        subprocess.run(docker_cmd, check=True)
        print(f"\n✅ Compilation successful: {output_hef}")
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Compilation failed: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"\n❌ Docker not found or not running")
        print(f"   Install Docker: https://docs.docker.com/engine/install/")
        sys.exit(1)


if __name__ == '__main__':
    main()
