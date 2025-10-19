#!/bin/bash

# Restart script with anti-slip verification
# Ensures clean state for testing bucket grasp

echo "=========================================="
echo "ANTI-SLIP FIX - SYSTEM RESTART"
echo "=========================================="

echo ""
echo "[Step 1] Killing all Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✓ Gazebo processes terminated"
else
    echo "  ℹ No Gazebo processes running"
fi

sleep 2

echo ""
echo "[Step 2] Clearing Gazebo model cache..."
if [ -d ~/.gazebo/models/bucket ]; then
    rm -rf ~/.gazebo/models/bucket
    echo "  ✓ Bucket cache cleared"
else
    echo "  ℹ No bucket cache found"
fi

echo ""
echo "[Step 3] Sourcing workspace..."
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash
echo "  ✓ Workspace sourced"

echo ""
echo "[Step 4] Launching Gazebo farm world..."
echo "  → Gripper friction: mu=15.0 (was 10.0)"
echo "  → Gripper stiffness: kp=2M (was 1M)"
echo "  → Contact detection: 0.05mm (was 0.1mm)"
echo ""

roslaunch pkg01 gazebo_farm.launch &

LAUNCH_PID=$!

echo ""
echo "Waiting for Gazebo to start (15 seconds)..."
sleep 15

echo ""
echo "=========================================="
echo "SYSTEM READY FOR TESTING"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. In Gazebo: Enable collision visualization"
echo "   View → Collisions (pink boxes should appear)"
echo ""
echo "2. Position gripper around bucket handle using MoveIt"
echo "   - Handle between fingers"
echo "   - Gripper perpendicular to handle"
echo "   - Small gap on each side (1-2cm)"
echo ""
echo "3. Run gentle grasp test:"
echo "   rosrun pkg01 test_gentle_grasp.py"
echo ""
echo "4. OR close gripper manually (6 seconds):"
echo "   rostopic pub -1 /ur10e_robot/gripper_controller/command \\"
echo "     trajectory_msgs/JointTrajectory \\"
echo "     \"{joint_names: ['finger_joint'], \\"
echo "       points: [{positions: [0.56], time_from_start: {secs: 6}}]}\""
echo ""
echo "Expected behavior:"
echo "  ✓ Bucket stays in place during closing"
echo "  ✓ No lateral sliding or pushing away"
echo "  ✓ Firm grasp achieved"
echo "  ✓ No vibration or jittering"
echo ""
echo "If bucket still slips, see:"
echo "  ~/lattebot_ws2/src/pkg01/claude_explanations/BUCKET_SLIP_DURING_CLOSING_FIX.md"
echo ""
echo "Press Ctrl+C to stop Gazebo"
echo "=========================================="

# Wait for user interrupt
wait $LAUNCH_PID
