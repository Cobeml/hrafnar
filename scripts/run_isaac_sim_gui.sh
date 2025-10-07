#!/bin/bash
# Launch Isaac Sim with GUI and ROS2 support
# This script enables ROS2 camera feed publishing for VLM controller

set -e

echo "============================================"
echo "  Isaac Sim GUI Mode with ROS2 Enabled"
echo "============================================"

# Set ROS2 environment variables for Isaac Sim's internal ROS2
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
export ROS_DOMAIN_ID=0
export AMENT_PREFIX_PATH=$HOME/isaacsim/exts/isaacsim.ros2.bridge/humble

echo ""
echo "✅ ROS2 Environment:"
echo "   ROS_DISTRO=$ROS_DISTRO"
echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "   ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "   AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
echo "   LD_LIBRARY_PATH includes Isaac Sim ROS2 libs"
echo ""

# Check if Isaac Sim exists
ISAAC_SIM_PYTHON="$HOME/isaacsim/python.sh"
if [ ! -f "$ISAAC_SIM_PYTHON" ]; then
    echo "❌ Error: Isaac Sim not found at $ISAAC_SIM_PYTHON"
    echo "   Please update the path in this script."
    exit 1
fi

# Check if script argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <path_to_python_script.py>"
    echo ""
    echo "Example:"
    echo "  $0 /home/cobe-liu/Developing/hrafnar/9_people.py"
    exit 1
fi

SCRIPT_PATH="$1"

if [ ! -f "$SCRIPT_PATH" ]; then
    echo "❌ Error: Script not found: $SCRIPT_PATH"
    exit 1
fi

echo "🚀 Launching Isaac Sim with GUI..."
echo "   Script: $SCRIPT_PATH"
echo ""
echo "📸 Camera feed will be published to: /drone1/camera/color/image_raw"
echo "   VLM controller in Docker can subscribe via ROS_DOMAIN_ID=0"
echo ""
echo "---"
echo ""

# Launch Isaac Sim
exec "$ISAAC_SIM_PYTHON" "$SCRIPT_PATH"
