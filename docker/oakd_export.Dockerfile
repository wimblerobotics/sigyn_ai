# Docker configuration for OAK-D model compilation
#
# This container includes DepthAI SDK and blobconverter for
# compiling ONNX models to .blob format for Myriad X.

FROM python:3.10-slim

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    libusb-1.0-0 \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install DepthAI and blobconverter
RUN pip3 install \
    depthai \
    blobconverter \
    onnx \
    onnx-simplifier \
    opencv-python \
    pyyaml

# Set working directory
WORKDIR /workspace

CMD ["/bin/bash"]

# Build instructions:
#   docker build -f docker/oakd_export.Dockerfile -t sigyn_ai_oakd:latest .
#
# Run instructions:
#   docker run --rm -v $(pwd):/workspace sigyn_ai_oakd:latest \
#       python3 -c "from blobconverter import from_onnx; \
#                   from_onnx('model.onnx', output_dir='.', shaves=6)"
