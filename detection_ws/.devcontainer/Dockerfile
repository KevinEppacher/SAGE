# === Stage 1: CUDA 12.1 base with runtime ===
FROM nvidia/cuda:12.1.1-devel-ubuntu22.04 AS cuda

# === Stage 2: ROS2 Humble with CUDA 12.1 copied in ===
FROM osrf/ros:humble-desktop-full

# Copy CUDA runtime libs from the previous stage
COPY --from=cuda /usr/local/cuda-12.1 /usr/local/cuda-12.1

# Set CUDA env explicitly to use 12.1 (even if host has 11.3)
ENV CUDA_HOME=/usr/local/cuda-12.1
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=12.1"

# Add ubuntu user with same UID/GID as host
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME

# Optional: Webcam support (remove if not needed)
RUN sudo usermod --append --groups video $USERNAME

# Update packages
RUN sudo apt update && sudo apt upgrade -y

########################################################
# Custom installations
########################################################

# Git
RUN sudo apt install -y git

RUN sudo apt update && sudo apt install -y \
    python3-pip \
    ffmpeg

WORKDIR /app
COPY requirements.txt /app/requirements.txt
COPY requirements_custom.txt /app/requirements_custom.txt

RUN pip install --upgrade pip==23.2.1
RUN pip install -r /app/requirements.txt
RUN pip install -r /app/requirements_custom.txt
RUN pip install openai-whisper

RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-tf-transformations

########################################################
# End Custom installations
########################################################

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
