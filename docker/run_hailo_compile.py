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


def image_exists(image_name: str) -> bool:
    """Check if a Docker image exists locally."""
    try:
        result = subprocess.run(
            ['docker', 'image', 'inspect', image_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def supports_direct_compile(image_name: str, docker_user: str) -> bool:
    """Check if image supports `hailo compile` command."""
    try:
        result = subprocess.run(
            [
                'docker', 'run', '--rm',
                '-u', docker_user,
                image_name,
                'hailo', 'compile', '--help'
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


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
        default='sigyn_ai_hailo:latest',
        help='Hailo Docker image name (default: sigyn_ai_hailo:latest)'
    )
    
    parser.add_argument(
        '--optimization',
        type=int,
        default=2,
        choices=[0, 1, 2],
        help='Optimization level (0=debug, 1=balanced, 2=performance)'
    )

    parser.add_argument(
        '--docker-user',
        type=str,
        default='0:0',
        help='User to run inside Docker (default: 0:0 to avoid mounted-volume permission issues)'
    )
    
    args = parser.parse_args()
    
    onnx_path = Path(args.onnx).resolve()
    if not onnx_path.exists():
        print(f"❌ ONNX file not found: {onnx_path}")
        sys.exit(1)
    
    output_dir = onnx_path.parent
    output_hef = output_dir / f"{onnx_path.stem}.hef"

    if not image_exists(args.docker_image):
        print(f"❌ Docker image not found locally: {args.docker_image}")
        print("\nBuild the local image first:")
        print("   docker build -f docker/hailo_export.Dockerfile -t sigyn_ai_hailo:latest .")
        print("\nOr run with a different image:")
        print("   python docker/run_hailo_compile.py --onnx <path> --docker-image <image>")
        sys.exit(1)
    
    direct_compile = supports_direct_compile(args.docker_image, args.docker_user)

    if direct_compile:
        docker_cmd = [
            'docker', 'run',
            '--rm',
            '-u', args.docker_user,
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
    else:
        # DFC 3.x flow: parser -> optimize -> compiler
        docker_cmd = [
            'docker', 'run',
            '--rm',
            '-u', args.docker_user,
            '-v', f'{output_dir}:/workspace',
            args.docker_image,
            'bash', '-lc',
            (
                'set -e; '
                'cd /workspace; '
                f'hailo parser onnx {onnx_path.name} --hw-arch hailo8 --har-path {onnx_path.stem}.har -y; '
                f'hailo optimize {onnx_path.stem}.har --hw-arch hailo8 --use-random-calib-set --output-har-path {onnx_path.stem}_optimized.har; '
                f'hailo compiler {onnx_path.stem}_optimized.har --hw-arch hailo8 --output-dir /workspace; '
                f'if [ -f /workspace/{onnx_path.stem}.hef ]; then '
                f'  echo "Found /workspace/{onnx_path.stem}.hef"; '
                f'elif [ -f /workspace/{onnx_path.stem}_optimized.hef ]; then '
                f'  cp /workspace/{onnx_path.stem}_optimized.hef /workspace/{onnx_path.stem}.hef; '
                f'  echo "Renamed {onnx_path.stem}_optimized.hef -> {onnx_path.stem}.hef"; '
                f'else '
                f'  echo "❌ HEF output not found after compilation"; '
                f'  ls -lh /workspace/*.hef 2>/dev/null || true; '
                f'  exit 1; '
                f'fi'
            )
        ]
    
    print(f"🐳 Running Hailo compilation in Docker...")
    print(f"   ONNX: {onnx_path.name}")
    print(f"   Output: {output_hef}")
    print(f"   Mode: {'direct hailo compile' if direct_compile else 'parser/optimize/compiler'}")
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
