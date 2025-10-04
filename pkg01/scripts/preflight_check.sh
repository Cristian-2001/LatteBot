#!/bin/bash
# Pre-flight check script to verify configuration before launch

echo "=========================================="
echo "PRE-FLIGHT CONFIGURATION CHECK"
echo "=========================================="
echo ""

ERRORS=0

# Check 1: URDF transmissions use PositionJointInterface
echo "Checking URDF transmissions..."
if grep -q "hardware_interface/EffortJointInterface" /home/vboxuser/lattebot_ws2/src/pkg01/urdf/simple_gripper.urdf.xacro; then
    echo "✗ FAIL: URDF still uses EffortJointInterface"
    echo "  Expected: PositionJointInterface"
    ERRORS=$((ERRORS + 1))
else
    echo "✓ PASS: URDF uses PositionJointInterface"
fi

# Check 2: Mimic plugin removed
echo ""
echo "Checking for mimic joint plugin..."
if grep -q "mimic_joint_plugin" /home/vboxuser/lattebot_ws2/src/pkg01/urdf/simple_gripper.urdf.xacro; then
    echo "✗ FAIL: Mimic joint plugin still present"
    echo "  Should be removed for independent control"
    ERRORS=$((ERRORS + 1))
else
    echo "✓ PASS: Mimic joint plugin removed"
fi

# Check 3: Controller YAML uses position controller
echo ""
echo "Checking controller configuration..."
if grep -A 2 "gripper_controller:" /home/vboxuser/lattebot_ws2/src/pkg01/controller/ur10e_controllers.yaml | grep -q "position_controllers/JointTrajectoryController"; then
    echo "✓ PASS: Controller uses position_controllers"
else
    echo "✗ FAIL: Controller doesn't use position_controllers"
    ERRORS=$((ERRORS + 1))
fi

# Check 4: Controller has both finger joints
echo ""
echo "Checking controller joint list..."
JOINT_COUNT=$(grep -A 5 "gripper_controller:" /home/vboxuser/lattebot_ws2/src/pkg01/controller/ur10e_controllers.yaml | grep "finger_joint" | wc -l)
if [ "$JOINT_COUNT" -eq 2 ]; then
    echo "✓ PASS: Controller has both finger joints"
else
    echo "✗ FAIL: Controller has $JOINT_COUNT finger joint(s), expected 2"
    ERRORS=$((ERRORS + 1))
fi

# Check 5: MoveIt ros_controllers.yaml has both joints
echo ""
echo "Checking MoveIt ros_controllers.yaml..."
MOVEIT_COUNT=$(grep -A 8 "gripper_controller" /home/vboxuser/lattebot_ws2/src/ur10e_moveit_config/config/ros_controllers.yaml | grep "finger_joint" | wc -l)
if [ "$MOVEIT_COUNT" -eq 2 ]; then
    echo "✓ PASS: MoveIt config has both finger joints"
else
    echo "✗ FAIL: MoveIt config has $MOVEIT_COUNT finger joint(s), expected 2"
    ERRORS=$((ERRORS + 1))
fi

# Check 6: MoveIt simple_moveit_controllers.yaml has both joints
echo ""
echo "Checking MoveIt simple_moveit_controllers.yaml..."
MOVEIT2_COUNT=$(grep -A 8 "gripper_controller" /home/vboxuser/lattebot_ws2/src/ur10e_moveit_config/config/simple_moveit_controllers.yaml | grep "finger_joint" | wc -l)
if [ "$MOVEIT2_COUNT" -eq 2 ]; then
    echo "✓ PASS: MoveIt simple config has both finger joints"
else
    echo "✗ FAIL: MoveIt simple config has $MOVEIT2_COUNT finger joint(s), expected 2"
    ERRORS=$((ERRORS + 1))
fi

# Check 7: Workspace built
echo ""
echo "Checking workspace build..."
if [ -f "/home/vboxuser/lattebot_ws2/devel/setup.bash" ]; then
    echo "✓ PASS: Workspace is built"
else
    echo "✗ FAIL: Workspace not built (run catkin_make)"
    ERRORS=$((ERRORS + 1))
fi

# Summary
echo ""
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    echo "✓ ALL CHECKS PASSED - READY TO LAUNCH"
    echo "=========================================="
    echo ""
    echo "You can now run:"
    echo "  source /home/vboxuser/lattebot_ws2/devel/setup.bash"
    echo "  roslaunch pkg01 gazebo_farm.launch"
    echo ""
    exit 0
else
    echo "✗ $ERRORS CHECK(S) FAILED"
    echo "=========================================="
    echo ""
    echo "Please fix the issues above before launching."
    echo "If you need to rebuild: cd /home/vboxuser/lattebot_ws2 && catkin_make"
    echo ""
    exit 1
fi
