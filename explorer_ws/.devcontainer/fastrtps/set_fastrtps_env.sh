#!/bin/bash
# set_fastrtps_env.sh
# Always copy local Fast DDS (Fast-RTPS) profiles to /etc and configure environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_PATH="${SCRIPT_DIR}/fastdds.xml"
DEST_PATH="/etc/fastdds.xml"

# Copy file to /etc with root rights if needed
if [ -f "$SRC_PATH" ]; then
    # echo "Copying Fast DDS profiles from $SRC_PATH to $DEST_PATH"
    if [ "$(id -u)" -eq 0 ]; then
        cp "$SRC_PATH" "$DEST_PATH"
    else
        sudo cp "$SRC_PATH" "$DEST_PATH"
    fi
else
    echo "Error: Fast DDS profiles not found at $SRC_PATH"
    exit 1
fi

# Export runtime variables for Fast DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=app/.devcontainer/fastrtps/fastdds.xml


# echo "RMW_IMPLEMENTATION set to $RMW_IMPLEMENTATION"
# echo "FASTRTPS_DEFAULT_PROFILES_FILE set to $FASTRTPS_DEFAULT_PROFILES_FILE"