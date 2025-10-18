#!/bin/bash
# Quick fix script to restart Gazebo and verify gripper setup

echo "=================================================="
echo "GRIPPER INDEPENDENT CONTROL - STARTUP SCRIPT"
echo "=================================================="
echo ""

# Check if workspace is sourced
if [ -z "$ROS_PACKAGE_PATH" ] || ! echo "$ROS_PACKAGE_PATH" | grep -q "lattebot_ws2"; then
    echo "⚠ Sourcing workspace..."
    source /home/vboxuser/lattebot_ws2/devel/setup.bash
    echo "✓ Workspace sourced"
else
    echo "✓ Workspace already sourced"
fi

echo ""
echo "Killing any existing Gazebo/ROS processes..."
killall -9 gazebo gzserver gzclient roscore rosmaster rosout 2>/dev/null
sleep 2
echo "✓ Processes cleaned"

echo ""
echo "=================================================="
echo "LAUNCHING SIMULATION"
echo "=================================================="
echo ""
echo "Starting: roslaunch pkg01 gazebo_farm.launch"
echo ""
echo "After Gazebo loads, run in another terminal:"
echo "  source /home/vboxuser/lattebot_ws2/devel/setup.bash"
echo "  rosrun pkg01 verify_gripper_setup.py"
echo ""
echo "=================================================="
echo ""

# Launch with explicit sourcing
roslaunch pkg01 gazebo_farm.launch
