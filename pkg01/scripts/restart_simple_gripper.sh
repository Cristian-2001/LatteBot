#!/bin/bash
# Restart script for testing the simple gripper design
# Ensures clean Gazebo state and launches farm world with bucket

echo "=========================================="
echo "SIMPLE GRIPPER TEST - RESTART SCRIPT"
echo "=========================================="
echo ""

# Kill any existing Gazebo processes
echo "[1/5] Killing existing Gazebo processes..."
killall -9 gzserver gzclient rosmaster roscore 2>/dev/null
sleep 2

# Clear Gazebo cache
echo "[2/5] Clearing Gazebo model cache..."
rm -rf ~/.gazebo/models/bucket 2>/dev/null

# Source workspace
echo "[3/5] Sourcing workspace..."
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash

# Verify URDF
echo "[4/5] Validating URDF..."
xacro src/pkg01/urdf/ur10e.urdf.xacro > /tmp/ur10e_simple_gripper.urdf
check_urdf /tmp/ur10e_simple_gripper.urdf > /tmp/urdf_check.log 2>&1

if [ $? -eq 0 ]; then
    echo "✓ URDF validation passed"
else
    echo "✗ URDF validation failed - check /tmp/urdf_check.log"
    exit 1
fi

# Launch Gazebo with farm world
echo "[5/5] Launching Gazebo with farm world..."
echo ""
echo "Gripper Design: Simple Parallel-Jaw"
echo "  - Finger length: 130mm"
echo "  - Opening range: 40-150mm"
echo "  - Friction: μ=3.0"
echo ""
echo "After launch, test with:"
echo "  rosrun pkg01 test_simple_gripper.py"
echo ""
sleep 2

roslaunch pkg01 gazebo_farm.launch
