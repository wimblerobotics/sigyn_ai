# Docker configuration for Hailo model compilation
#
# This extends the official Hailo SDK container with additional tools.
# Use for compiling ONNX models to .hef format.

FROM hailo/hailo_sw_suite:2025-10

ENV DEBIAN_FRONTEND=noninteractive

# Install additional Python packages
RUN pip3 install \
    pyyaml \
    onnx \
    onnx-simplifier

# Set working directory
WORKDIR /workspace

CMD ["/bin/bash"]

# Build instructions:
#   docker build -f docker/hailo_export.Dockerfile -t sigyn_ai_hailo:latest .
#
# Run instructions:
#   docker run --rm -v $(pwd):/workspace sigyn_ai_hailo:latest \
#       hailo compile --model model.onnx --hw-arch hailo8 --output model.hef
#
# Or use the wrapper script:
#   python docker/run_hailo_compile.py --onnx models/exported/best/pi5_hailo8/best.onnx
