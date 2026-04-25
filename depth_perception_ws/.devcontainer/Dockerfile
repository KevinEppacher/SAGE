ARG ROS_DISTRO=jazzy
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# === Stage 1: CUDA 12.8 base with runtime ===
FROM nvcr.io/nvidia/tensorrt:25.03-py3 AS cuda

# === Stage 2: ROS2 Humble with CUDA 12.8 copied in ===
FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Copy CUDA runtime libs from the previous stage
COPY --from=cuda /usr/local/cuda-12.8 /usr/local/cuda-12.8

# Set CUDA env explicitly to use 12.8 (even if host has 11.3)
ENV CUDA_HOME=/usr/local/cuda-12.8
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=12.8"

COPY --from=cuda /usr/include/x86_64-linux-gnu/Nv*.h /usr/include/x86_64-linux-gnu/
COPY --from=cuda /usr/lib/x86_64-linux-gnu/libnvinfer* /usr/lib/x86_64-linux-gnu/
COPY --from=cuda /usr/lib/x86_64-linux-gnu/libnvonnxparser* /usr/lib/x86_64-linux-gnu/
COPY --from=cuda /usr/lib/x86_64-linux-gnu/libnvparsers* /usr/lib/x86_64-linux-gnu/

# TensorRT env hints
ENV TENSORRT_INCLUDE_DIR=/usr/include/x86_64-linux-gnu
ENV TENSORRT_LIBRARY_INFER=/usr/lib/x86_64-linux-gnu/libnvinfer.so
ENV TENSORRT_LIBRARY_INFER_PLUGIN=/usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so
ENV TENSORRT_LIBRARY_ONNXPARSER=/usr/lib/x86_64-linux-gnu/libnvonnxparser.so
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# ########################################################
# # Middleware installations
# ########################################################

# Install both RMW middlewares
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp

RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /home/${USERNAME}/.bashrc \
 && echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/app/.devcontainer/fastrtps/fastdds.xml' >> /home/${USERNAME}/.bashrc

# ########################################################
# # End Middleware installations
# ########################################################

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

RUN sudo apt update && sudo apt install -y \
    python3-venv \
    python3-pip \
    ffmpeg

COPY requirements.txt /app/requirements.txt
RUN python3 -m venv /home/${USERNAME}/.venv
ENV PATH=/home/${USERNAME}/.venv/bin:$PATH
RUN python -m pip install --upgrade pip && pip install -r /app/requirements.txt

RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-tf-transformations 

########################################################
# End Custom installations
########################################################

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

# Set RMW to FastRTPS by default and point to custom FastDDS config
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /home/${USERNAME}/.bashrc \
 && echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/app/.devcontainer/fastrtps/fastdds.xml' >> /home/${USERNAME}/.bashrc
# Set up the entrypoint
COPY bash/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
# Set working directory to overlay workspace
WORKDIR /app
