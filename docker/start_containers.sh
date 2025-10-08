#!/bin/bash

# Automated startup for IsaacLab ROS2 container and related docker-compose containers
# Make this script executable with: chmod +x start_containers.sh

set -e  # Exit immediately if a command exits with a non-zero status

# Get the script directory and navigate to the repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Repository root: $REPO_ROOT"

# Step 1: Navigate to the IsaacLab folder
cd "$REPO_ROOT/IsaacLab" || {
  echo "IsaacLab folder not found in $REPO_ROOT!"
  exit 1
}

# Step 2: Start the ROS2 container using the internal script
echo "Starting IsaacLab ROS2 container..."
./docker/container.py start ros2

# Step 3: Navigate to the docker directory
cd "$REPO_ROOT/docker" || {
  echo "docker folder not found in $REPO_ROOT!"
  exit 1
}

# Step 4: Start the detection, explorer, and exploitation containers via docker-compose
echo "Starting detection, explorer, and exploitation containers with docker-compose..."
docker compose up -d detection_container explorer_container exploitation_container lavis_container