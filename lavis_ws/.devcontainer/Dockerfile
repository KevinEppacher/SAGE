# ROS distribution to use
ARG ROS_DISTRO=humble
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ARG RMW_IMPLEMENTATION=rmw_zenoh_cpp

FROM nvidia/cuda:12.1.1-devel-ubuntu22.04 AS cuda

FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Copy CUDA runtime libs from the previous stage
COPY --from=cuda /usr/local/cuda-12.1 /usr/local/cuda-12.1

# Set CUDA env explicitly to use 12.1 (even if host has 11.3)
ENV CUDA_HOME=/usr/local/cuda-12.1
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=12.1"

# ########################################################
# # Middleware installations
# ########################################################

# Install both RMW middlewares
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp

# # Copy Cyclone DDS XML config (must be present in build context)
# COPY cyclonedds.xml /etc/cyclonedds.xml

# # Copy Zenoh config files (must be present in build context)
# COPY MY_ZENOH_ROUTER_CONFIG.json5 /root/MY_ZENOH_ROUTER_CONFIG.json5
# COPY MY_ZENOH_SESSION_CONFIG.json5 /root/MY_ZENOH_SESSION_CONFIG.json5

# # --- Default configuration (CycloneDDS for local performance) ---
# ENV RCUTILS_COLORIZED_OUTPUT=1
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# # ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
# ENV CYCLONEDDS_URI=/etc/cyclonedds.xml

# # --- Zenoh optional configuration placeholders (used when overridden) ---
# ENV ZENOH_ROUTER_CONFIG_URI=/root/MY_ZENOH_ROUTER_CONFIG.json5
# ENV ZENOH_SESSION_CONFIG=/root/MY_ZENOH_SESSION_CONFIG.json5

# # --- ROS setup for all shells ---
# RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
#     echo 'echo \"Active RMW: $RMW_IMPLEMENTATION\"' >> /root/.bashrc && \
#     echo 'if [ \"$RMW_IMPLEMENTATION\" = \"rmw_cyclonedds_cpp\" ]; then echo \"Using CycloneDDS → $CYCLONEDDS_URI\"; fi' >> /root/.bashrc && \
#     echo 'if [ \"$RMW_IMPLEMENTATION\" = \"rmw_zenoh_cpp\" ]; then echo \"Using Zenoh → $ZENOH_SESSION_CONFIG\"; fi' >> /root/.bashrc

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
    python3-pip

WORKDIR /app
COPY requirements.txt /app/requirements.txt

RUN pip install --upgrade pip==23.2.1
RUN pip install -r /app/requirements.txt

RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-costmap-2d

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
