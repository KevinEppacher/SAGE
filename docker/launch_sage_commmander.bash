#!/bin/bash

# Container-Name
CONTAINER_NAME=my_ros2_container

# Workspace-Pfad im Container
WORKSPACE_PATH=/app

# Launchfile
PACKAGE_NAME=sage_commander
LAUNCH_FILE=startup_detection_ws.launch.py

# Kommando im Container vorbereiten
CMD="cd ${WORKSPACE_PATH} && \
     source /opt/ros/humble/setup.bash && \
     source install/setup.bash && \
     ros2 launch ${PACKAGE_NAME} ${LAUNCH_FILE}"

# Im Container ausführen
docker exec -it ${CONTAINER_NAME} bash -c "${CMD}"