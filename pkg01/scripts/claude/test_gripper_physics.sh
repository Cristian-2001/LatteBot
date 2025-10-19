#!/bin/bash
# Quick restart script for testing gripper-handle physics balance

echo "======================================"
echo "Gripper-Handle Physics Balance Tester"
echo "======================================"
echo ""
echo "Changes applied:"
echo "  ✓ Bucket handles: kp=500k (balanced stiffness)"
echo "  ✓ Bucket handles: mu=5.0 (high friction)"
echo "  ✓ Gripper fingers: mu=5.0 (NEW - high friction)"
echo "  ✓ Contact physics: kd=500, max_vel=0.01, min_depth=0.0005"
echo ""

# Kill any existing Gazebo processes
echo "1. Cleaning up existing Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
sleep 2

# Clear bucket model cache (CRITICAL for model changes)
echo "2. Clearing Gazebo model cache..."
rm -rf ~/.gazebo/models/bucket 2>/dev/null
echo "   ✓ Bucket cache cleared"

# Source workspace
echo "3. Sourcing workspace..."
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash
echo "   ✓ Workspace sourced"

echo ""
echo "4. Launching Gazebo with farm world + MoveIt..."
echo "   Watch for:"
echo "   - Gripper closes on handle without slip"
echo "   - Bucket lifts without falling"
echo "   - No explosive forces during motion"
echo ""
echo "======================================"
echo ""

# Launch simulation
roslaunch pkg01 gazebo_farm.launch
