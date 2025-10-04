#!/bin/bash
#
# Run Pegasus Simulation with Docker ROS2 Bridge
# Automatically starts ROS2 container + Isaac Sim simulation
#

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "========================================================================"
echo "  Pegasus Simulator with Docker ROS2 Bridge"
echo "========================================================================"
echo ""

# Check if a script was provided
if [ $# -eq 0 ]; then
    echo -e "${RED}❌ ERROR: No simulation script provided${NC}"
    echo ""
    echo "Usage:"
    echo "  $0 examples/navy_ship_patrol_NO_PEOPLE.py"
    echo ""
    echo "Available scripts:"
    ls -1 examples/*.py | grep -E "(navy_ship|camera|ros2)" | sed 's/^/  /'
    exit 1
fi

SCRIPT="$1"

# Check if script exists
if [ ! -f "$SCRIPT" ]; then
    echo -e "${RED}❌ ERROR: Script not found: $SCRIPT${NC}"
    exit 1
fi

# Check if Isaac Sim is installed
if [ -z "$ISAACSIM_PYTHON_EXE" ]; then
    echo -e "${RED}❌ ERROR: ISAACSIM_PYTHON_EXE not set${NC}"
    echo ""
    echo "Please set environment variable:"
    echo "  export ISAACSIM_PYTHON_EXE=/home/cobe-liu/isaacsim/python.sh"
    exit 1
fi

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: Docker is not running${NC}"
    echo ""
    echo "Start Docker:"
    echo "  sudo systemctl start docker"
    exit 1
fi

# Deactivate conda if active
if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
    echo -e "${YELLOW}⚠️  Conda environment detected, deactivating...${NC}"
    conda deactivate 2>/dev/null || true
fi

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping ROS2 containers...${NC}"
    docker-compose down
    echo -e "${GREEN}✅ Cleanup complete${NC}"
}

trap cleanup EXIT INT TERM

echo ""
echo "========================================================================"
echo "  Step 1/3: Starting Docker ROS2 Bridge"
echo "========================================================================"
echo ""

# Build image if needed
if ! docker images | grep -q "hrafnar-ros2-humble"; then
    echo "Building Docker image (this may take 5-10 minutes)..."
    docker-compose build ros2_bridge
fi

# Start ROS2 containers
echo "Starting ROSboard..."
docker-compose up -d ros2_bridge

# Wait for ROS2 to be ready
echo ""
echo "Waiting for ROS2 to initialize..."
sleep 3

# Verify ROS2 is running
if ! docker-compose exec -T ros2_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" > /dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: ROS2 failed to start${NC}"
    echo ""
    echo "Check logs with:"
    echo "  docker-compose logs ros2_bridge"
    exit 1
fi

echo -e "${GREEN}✅ ROS2 Bridge ready${NC}"
echo ""
echo "Dashboard available at: ${BLUE}http://localhost:8888${NC}"

echo ""
echo "========================================================================"
echo "  Step 2/3: Launching Isaac Sim Simulation"
echo "========================================================================"
echo ""
echo "Script: $SCRIPT"
echo ""

# Check if script has ROS2Backend enabled
if ! grep -q "ROS2Backend" "$SCRIPT"; then
    echo -e "${YELLOW}⚠️  WARNING: Script may not have ROS2Backend enabled${NC}"
    echo ""
    echo "Make sure your script includes:"
    echo '  from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend'
    echo '  config_multirotor.backends = ['
    echo '      PX4MavlinkBackend(mavlink_config),'
    echo '      ROS2Backend(...)'
    echo '  ]'
    echo ""
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ========================================
# Configure Isaac Sim's Internal ROS2
# ========================================
echo "Configuring Isaac Sim's internal ROS2 libraries..."

# Derive Isaac Sim base directory from ISAACSIM_PYTHON_EXE
# E.g., /home/user/isaacsim/python.sh -> /home/user/isaacsim
ISAACSIM_DIR=$(dirname "$ISAACSIM_PYTHON_EXE")

# Set ROS2 environment for Isaac Sim (must be set BEFORE launching Isaac Sim)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAACSIM_DIR/exts/isaacsim.ros2.bridge/humble/lib
export ROS_DOMAIN_ID=0  # Must match Docker ROS2

echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  LD_LIBRARY_PATH includes Isaac Sim ROS2 libs"
echo ""

# Launch Isaac Sim in background, capture PID
echo "Launching Isaac Sim..."
echo ""

$ISAACSIM_PYTHON_EXE "$SCRIPT" &
ISAACSIM_PID=$!

echo ""
echo "========================================================================"
echo "  Step 3/3: Monitoring Simulation"
echo "========================================================================"
echo ""
echo -e "${GREEN}✅ All systems running!${NC}"
echo ""
echo "Services:"
echo "  - Isaac Sim:       PID $ISAACSIM_PID"
echo "  - ROS2 Bridge:     Running in Docker"
echo "  - ROSboard:        http://localhost:8888"
echo ""
echo "Available commands:"
echo "  - QGroundControl:  ~/QGroundControl.AppImage"
echo "  - ROS2 topics:     docker-compose exec ros2_bridge bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
echo "  - Logs:            docker-compose logs -f ros2_bridge"
echo ""
echo "Press Ctrl+C to stop all services"
echo "========================================================================"
echo ""

# Monitor Isaac Sim process
wait $ISAACSIM_PID
ISAACSIM_EXIT_CODE=$?

echo ""
if [ $ISAACSIM_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✅ Isaac Sim exited normally${NC}"
else
    echo -e "${YELLOW}⚠️  Isaac Sim exited with code: $ISAACSIM_EXIT_CODE${NC}"
fi

echo ""
echo "Simulation complete."
echo ""
