#!/bin/bash
set -e

echo "[Set the ROS environment]"

name_ros_version=${name_ros_version:="humble"}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_BASHRC="$HOME/.bashrc"

# Determine ROS_DOMAIN_ID: try active interface IP octet, fallback to 0
detected_ip=$(ip route get 1.1.1.1 2>/dev/null | awk '{print $7}')
if [ -n "$detected_ip" ]; then
    last_octet=$(echo "$detected_ip" | awk -F. '{print $4}')
    # ROS_DOMAIN_ID should be between 0 and 101 for best compatibility
    if [ -n "$last_octet" ] && [ "$last_octet" -ge 0 ] 2>/dev/null; then
        ros_domain_id=$((last_octet % 102))
    else
        ros_domain_id=0
    fi
else
    ros_domain_id=0
fi

echo "Configuring environment for user $USER with ROS_DOMAIN_ID=$ros_domain_id..."

ROS_SOURCE_LINE="source /opt/ros/$name_ros_version/setup.bash"
WS_SOURCE_LINE="source $WORKSPACE_DIR/install/local_setup.bash"
DOMAIN_ID_LINE="export ROS_DOMAIN_ID=$ros_domain_id"

grep -qxF "$ROS_SOURCE_LINE" "$TARGET_BASHRC" 2>/dev/null || echo "$ROS_SOURCE_LINE" >> "$TARGET_BASHRC"
grep -qxF "$WS_SOURCE_LINE" "$TARGET_BASHRC" 2>/dev/null || echo "$WS_SOURCE_LINE" >> "$TARGET_BASHRC"

# Replace or append ROS_DOMAIN_ID
if grep -q "export ROS_DOMAIN_ID=" "$TARGET_BASHRC" 2>/dev/null; then
    sed -i "s/export ROS_DOMAIN_ID=.*/$DOMAIN_ID_LINE/" "$TARGET_BASHRC"
else
    echo "$DOMAIN_ID_LINE" >> "$TARGET_BASHRC"
fi

echo "[Environment setup successfully updated in $TARGET_BASHRC]"
