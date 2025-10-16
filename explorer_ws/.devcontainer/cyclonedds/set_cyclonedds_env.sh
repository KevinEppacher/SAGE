#!/bin/bash
# set_cyclonedds_env.sh
# Always copy local CycloneDDS XML to /etc and configure environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_PATH="${SCRIPT_DIR}/cyclonedds.xml"
DEST_PATH="/etc/cyclonedds.xml"

# Copy file to /etc with root rights if needed
if [ -f "$SRC_PATH" ]; then
    echo "Copying CycloneDDS config from $SRC_PATH to $DEST_PATH"
    if [ "$(id -u)" -eq 0 ]; then
        cp "$SRC_PATH" "$DEST_PATH"
    else
        sudo cp "$SRC_PATH" "$DEST_PATH"
    fi
else
    echo "Error: CycloneDDS config not found at $SRC_PATH"
    exit 1
fi

# Export runtime variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${DEST_PATH}"

echo "RMW_IMPLEMENTATION set to $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI set to $CYCLONEDDS_URI"
