FROM osrf/ros:humble-desktop-full

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

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

#####################################
## Custom install as non-root user ##
#####################################

USER root
RUN apt-get update && apt-get install -y curl
RUN curl -L -o /usr/local/bin/Groot2.AppImage \
  https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage \
  && chmod +x /usr/local/bin/Groot2.AppImage

RUN apt-get update && apt-get install -y fuse3 \
 && groupadd -f fuse && usermod -aG fuse ${USERNAME}

#########################################
## End Custom install as non-root user ##
#########################################

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    libceres-dev

RUN sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO}-pcl-ros
    
RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan

RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt-tf-tree

# --- BehaviorTree.CPP + ZMQ + GUI deps ---
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    libzmq3-dev \
    libfuse2 curl git cmake \
 && sudo rm -rf /var/lib/apt/lists/*

 # --- cppzmq (header-only) for Jammy: install from source ---
 RUN git clone --depth=1 https://github.com/zeromq/cppzmq /tmp/cppzmq \
 && cmake -S /tmp/cppzmq -B /tmp/cppzmq/build -DCPPZMQ_BUILD_TESTS=OFF -DCPPZMQ_BUILD_EXAMPLES=OFF \
 && sudo cmake --install /tmp/cppzmq/build \
 && rm -rf /tmp/cppzmq

########################################################
# End Custom installations
########################################################

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc
RUN echo 'alias groot="/usr/local/bin/Groot2.AppImage"' >> /home/${USERNAME}/.bashrc