#!/bin/bash
# Fix Bucket Pass-Through Issue - Complete Restart Script
# This script fully cleans Gazebo cache and restarts with enhanced physics

echo "=========================================="
echo "Bucket Pass-Through Fix - Complete Restart"
echo "=========================================="

# Step 1: Kill all Gazebo processes
echo ""
echo "[1/5] Killing all Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
sleep 2

# Verify processes are dead
if pgrep -x "gzserver" > /dev/null || pgrep -x "gzclient" > /dev/null; then
    echo "WARNING: Gazebo processes still running. Trying again..."
    killall -9 gzserver gzclient 2>/dev/null
    sleep 2
fi

# Step 2: Clear Gazebo model cache
echo ""
echo "[2/5] Clearing Gazebo model cache..."
if [ -d ~/.gazebo/models/bucket ]; then
    rm -rf ~/.gazebo/models/bucket
    echo "  ✓ Removed cached bucket model"
else
    echo "  ℹ No cached bucket model found"
fi

# Step 3: Rebuild workspace (in case changes need compilation)
echo ""
echo "[3/5] Rebuilding workspace..."
cd /home/vboxuser/lattebot_ws2
catkin_make 2>&1 | tail -n 5

# Step 4: Source workspace
echo ""
echo "[4/5] Sourcing workspace..."
source /home/vboxuser/lattebot_ws2/devel/setup.bash
echo "  ✓ Workspace sourced"

# Step 5: Launch farm world with enhanced physics
echo ""
echo "[5/5] Launching farm world with fixed bucket physics..."
echo ""
echo "=========================================="
echo "Changes Applied:"
echo "  • Bucket friction: 2.5 → 3.0 (ultra-high grip)"
echo "  • Contact stiffness: 1M → 10M (10x stiffer)"
echo "  • Contact damping: 100 → 1000 (10x damping)"
echo "  • Max vel: 0.1 → 0.01 (slower correction)"
echo "  • Min depth: 0.001 → 0.0001 (finer contact)"
echo "  • Self-collision enabled for bucket"
echo "  • Enhanced ODE solver (100 iterations)"
echo "  • Smaller time steps (0.001s)"
echo "=========================================="
echo ""
echo "Wait for Gazebo to fully load, then:"
echo "1. Close gripper around bucket handle"
echo "2. Lift slowly to test grip"
echo ""
echo "If bucket still passes through:"
echo "  - Check collision visualization (View → Collisions)"
echo "  - Verify gripper is around handle, not body"
echo "  - Try slower gripper closing speed"
echo ""

sleep 3
roslaunch pkg01 gazebo_farm.launch
