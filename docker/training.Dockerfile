# Docker configuration for YOLO training
#
# This container includes PyTorch, CUDA support, and Ultralytics YOLO.
# Use for training models on your local GPU.

FROM nvidia/cuda:12.1.0-cudnn8-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Install PyTorch with CUDA support
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install Ultralytics YOLO
RUN pip3 install ultralytics==8.4.14

# Install additional dependencies
RUN pip3 install \
    onnx \
    onnx-simplifier \
    onnxruntime-gpu \
    opencv-python \
    pyyaml \
    tqdm \
    matplotlib \
    seaborn \
    pandas

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]

# Build instructions:
#   docker build -f docker/training.Dockerfile -t sigyn_ai_training:latest .
#
# Run instructions:
#   docker run --rm --gpus all -v $(pwd):/workspace sigyn_ai_training:latest \
#       python3 src/training/train.py --config configs/training/can_detector_pihat.yaml
