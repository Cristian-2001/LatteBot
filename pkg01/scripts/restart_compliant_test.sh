#!/bin/bash

# Restart script for testing compliant gripper grasping
# Cleans up Gazebo and relaunches with bucket model

echo "================================================"
echo "COMPLIANT GRIPPER TEST - Restart Script"
echo "================================================"

# 1. Kill all Gazebo processes
echo ""
echo "[1/5] Killing Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
sleep 2

# 2. Clear cached models
echo ""
echo "[2/5] Clearing Gazebo model cache..."
rm -rf ~/.gazebo/models/bucket
echo "Bucket model cache cleared"

# 3. Source workspace
echo ""
echo "[3/5] Sourcing workspace..."
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash
echo "Workspace sourced"

# 4. Launch Gazebo with farm world
echo ""
echo "[4/5] Launching Gazebo with bucket model..."
echo "Starting in 2 seconds..."
sleep 2

roslaunch pkg01 gazebo_farm.launch &
LAUNCH_PID=$!

# Wait for simulation to start
echo ""
echo "Waiting for Gazebo to initialize (15 seconds)..."
sleep 15

# 5. Launch test script
echo ""
echo "[5/5] Ready to test compliant grasping!"
echo ""
echo "================================================"
echo "MANUAL TESTING PROCEDURE:"
echo "================================================"
echo ""
echo "1. In RViz, move the arm to 'grasp' pose:"
echo "   - Select 'manipulator' planning group"
echo "   - Choose 'grasp' from Goal State dropdown"
echo "   - Click 'Plan & Execute'"
echo ""
echo "2. Position gripper above bucket handle"
echo ""
echo "3. Close gripper to grasp_handle position:"
echo "   - Select 'gripper' planning group"
echo "   - Choose 'grasp_handle' from Goal State dropdown"
echo "   - Click 'Plan & Execute'"
echo "   - OBSERVE: Should close smoothly with NO jumping"
echo ""
echo "4. OR run automated test:"
echo "   rosrun pkg01 test_compliant_grasp.py"
echo ""
echo "================================================"
echo "EXPECTED RESULTS:"
echo "================================================"
echo ""
echo "✅ Fingers close smoothly to -0.008m position"
echo "✅ Handle remains stable between fingers"
echo "✅ No bouncing, jumping, or vibration"
echo "✅ Compliant contact (slight position variation OK)"
echo ""
echo "❌ If handle jumps: Further reduce gains in ur10e_controllers.yaml"
echo ""
echo "================================================"

# Keep script running
wait $LAUNCH_PID
