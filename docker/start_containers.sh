#!/bin/bash

# Automated startup for IsaacLab ROS2 container and related docker-compose containers
# Make this script executable with: chmod +x start_isaaclab.sh

set -e  # Exit immediately if a command exits with a non-zero status

# Step 1: Navigate to the IsaacLab folder
cd ~/Documents/4_Semester/SAGE/IsaacLab || {
  echo "IsaacLab folder not found!"
  exit 1
}

# Step 2: Start the ROS2 container using the internal script
echo "Starting IsaacLab ROS2 container..."
./docker/container.py start ros2

# Step 3: Navigate to the docker directory
cd ../docker || {
  echo "docker folder not found!"
  exit 1
}

# Step 4: Start the detection, explorer, and exploitation containers via docker-compose
echo "Starting detection, explorer, and exploitation containers with docker-compose..."
docker compose up -d detection_container explorer_container exploitation_container lavis_container
