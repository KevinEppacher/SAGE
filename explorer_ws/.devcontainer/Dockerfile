# ROS distribution to use
ARG ROS_DISTRO=humble
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

########################################################
# Base Image with External Dependencies
########################################################
FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install CUDA dependencies (without full CUDA toolkit)
RUN apt-get update && apt-get install -y \
    nvidia-cuda-toolkit

# Ensure NVIDIA libraries are correctly linked
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.3"

# Set up CUDA paths (if not automatically set)
ENV PATH /usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/cuda/lib64:${LD_LIBRARY_PATH}

########################################################
# Middleware installations
########################################################

# Install both RMW middlewares
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp

# Copy Cyclone DDS XML config (must be present in build context)
COPY cyclonedds.xml /etc/cyclonedds.xml

# Copy Zenoh config files (must be present in build context)
COPY MY_ZENOH_ROUTER_CONFIG.json5 /root/MY_ZENOH_ROUTER_CONFIG.json5
COPY MY_ZENOH_SESSION_CONFIG.json5 /root/MY_ZENOH_SESSION_CONFIG.json5

# --- Default configuration (CycloneDDS for local performance) ---
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
ENV CYCLONEDDS_URI=/etc/cyclonedds.xml

# --- Zenoh optional configuration placeholders (used when overridden) ---
ENV ZENOH_ROUTER_CONFIG_URI=/root/MY_ZENOH_ROUTER_CONFIG.json5
ENV ZENOH_SESSION_CONFIG=/root/MY_ZENOH_SESSION_CONFIG.json5

# --- ROS setup for all shells ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo 'echo \"Active RMW: $RMW_IMPLEMENTATION\"' >> /root/.bashrc && \
    echo 'if [ \"$RMW_IMPLEMENTATION\" = \"rmw_cyclonedds_cpp\" ]; then echo \"Using CycloneDDS → $CYCLONEDDS_URI\"; fi' >> /root/.bashrc && \
    echo 'if [ \"$RMW_IMPLEMENTATION\" = \"rmw_zenoh_cpp\" ]; then echo \"Using Zenoh → $ZENOH_SESSION_CONFIG\"; fi' >> /root/.bashrc

########################################################
# End Middleware installations
########################################################

# Install cppzmq (header-only) from source
RUN git clone --depth=1 https://github.com/zeromq/cppzmq /tmp/cppzmq \
 && cmake -S /tmp/cppzmq -B /tmp/cppzmq/build -DCPPZMQ_BUILD_TESTS=OFF -DCPPZMQ_BUILD_EXAMPLES=OFF \
 && cmake --install /tmp/cppzmq/build \
 && rm -rf /tmp/cppzmq

# Download and install Groot2
RUN curl -L -o /usr/local/bin/Groot2.AppImage \
  https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage \
  && chmod +x /usr/local/bin/Groot2.AppImage \
  && ln -s /usr/local/bin/Groot2.AppImage /usr/local/bin/groot

# Create base workspace with external dependencies
RUN mkdir -p /base_ws/src
WORKDIR /base_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base workspace
WORKDIR /base_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

########################################################
# Development Overlay Image #
########################################################
FROM base AS dev

# Dev container arguments
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID --create-home $USERNAME \
 && apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

# Add user to fuse group for AppImage support
RUN groupadd -f fuse && usermod -aG fuse $USERNAME

# Install development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb gdbserver nano

# Switch to user
USER $USERNAME

########################################################
# Custom installations
########################################################

# Install basic packages
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 \
    python3-pip python3-vcstool curl cmake \
    libzmq3-dev libceres-dev

# Install ROS packages
RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-nav2-mppi-controller

########################################################
# End Custom installations
########################################################

# Set bash as default shell for the user
RUN sudo chsh -s /bin/bash $USERNAME

# Add user to video group
RUN sudo usermod --append --groups video $USERNAME

# Set up user bashrc
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc \
 && echo 'alias groot="/usr/local/bin/groot"' >> /home/${USERNAME}/.bashrc
 
# Set working directory to overlay workspace
WORKDIR /app