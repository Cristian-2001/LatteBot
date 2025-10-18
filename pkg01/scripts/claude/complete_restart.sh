#!/bin/bash
# Complete restart and launch script with verification

echo "=========================================="
echo "GRIPPER FIX - COMPLETE RESTART"
echo "=========================================="
echo ""

# Step 1: Pre-flight check
echo "Step 1: Running pre-flight checks..."
/home/vboxuser/lattebot_ws2/src/pkg01/scripts/preflight_check.sh
if [ $? -ne 0 ]; then
    echo ""
    echo "Pre-flight checks failed! Please fix the issues above."
    exit 1
fi

echo ""
echo "Step 2: Killing existing ROS/Gazebo processes..."
killall -9 gazebo gzserver gzclient roscore rosmaster rosout 2>/dev/null
sleep 2
echo "✓ Processes cleaned"

echo ""
echo "Step 3: Sourcing workspace..."
source /home/vboxuser/lattebot_ws2/devel/setup.bash
echo "✓ Workspace sourced"

echo ""
echo "=========================================="
echo "LAUNCHING GAZEBO + MOVEIT"
echo "=========================================="
echo ""
echo "After Gazebo fully loads, open a NEW terminal and run:"
echo ""
echo "  source /home/vboxuser/lattebot_ws2/devel/setup.bash"
echo "  rosrun pkg01 verify_gripper_setup.py"
echo ""
echo "Then test the 'open' pose in RViz MoveIt Planning tab!"
echo ""
echo "=========================================="
echo ""
echo "Launching in 3 seconds..."
sleep 3

# Launch
roslaunch pkg01 gazebo_farm.launch
