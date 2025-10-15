#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the base workspace with external dependencies
if [ -f /base_ws/install/setup.bash ]
then
  source /base_ws/install/setup.bash
  echo "Sourced base workspace (external dependencies)"
fi

# Source the overlay workspace (your main packages)
if [ -f /app/install/setup.bash ]
then
  source /app/install/setup.bash
  echo "Sourced overlay workspace (main packages)"
fi

# Source bash aliases if they exist
if [ -f /app/.bash_aliases ]
then
  source /app/.bash_aliases
  echo "Sourced bash aliases"
fi

# Apply CycloneDDS configuration for this workspace
if [ -f /app/.devcontainer/set_cyclonedds_env.sh ]; then
  source /app/.devcontainer/set_cyclonedds_env.sh
fi

# Execute the command passed into this entrypoint
exec "$@"