#!/bin/bash
# Helper script to properly restart Gazebo with farm world
# This ensures all changes to world and model files are loaded

echo "=========================================="
echo "Restarting Gazebo Farm Simulation"
echo "=========================================="

# Step 1: Kill all existing Gazebo processes
echo "1. Killing all Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
sleep 1

# Step 2: Clear Gazebo cache (if any)
echo "2. Clearing Gazebo cache..."
rm -rf ~/.gazebo/models/bucket 2>/dev/null

# Step 3: Source the workspace
echo "3. Sourcing workspace..."
source /home/vboxuser/lattebot_ws2/devel/setup.bash

# Step 4: Verify model exists
echo "4. Verifying bucket model..."
MODEL_PATH="/home/vboxuser/lattebot_ws2/src/pkg01/models/bucket"
if [ -f "$MODEL_PATH/model.sdf" ] && [ -f "$MODEL_PATH/model.config" ]; then
    echo "   ✓ Bucket model found"
else
    echo "   ✗ ERROR: Bucket model files missing!"
    exit 1
fi

# Step 5: Launch Gazebo
echo "5. Launching Gazebo with farm world..."
echo "=========================================="
roslaunch pkg01 gazebo_farm.launch

