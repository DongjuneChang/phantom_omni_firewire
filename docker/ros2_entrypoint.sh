#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
