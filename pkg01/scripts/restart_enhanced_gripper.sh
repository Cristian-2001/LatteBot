#!/bin/bash
# Quick restart script for testing enhanced gripper physics
# Ensures Gazebo cache is cleared and new physics parameters are loaded

echo "=============================================="
echo "Enhanced Gripper Lift Test - Quick Restart"
echo "=============================================="
echo ""

# Kill any existing Gazebo processes
echo "[1/5] Killing existing Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
sleep 1

# Clear Gazebo model cache
echo "[2/5] Clearing Gazebo model cache..."
rm -rf ~/.gazebo/models/bucket
echo "  ✓ Bucket cache cleared"

# Source workspace
echo "[3/5] Sourcing workspace..."
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash
echo "  ✓ Workspace sourced"

# Verify physics changes
echo "[4/5] Verifying updated files..."
if grep -q "effort=\"500\"" src/pkg01/urdf/simple_gripper.urdf.xacro; then
    echo "  ✓ Gripper effort = 500N (enhanced)"
else
    echo "  ✗ WARNING: Gripper effort not updated!"
fi

if grep -q "mu1>3.0" src/pkg01/urdf/simple_gripper.urdf.xacro; then
    echo "  ✓ Gripper friction = 3.0 (enhanced)"
else
    echo "  ✗ WARNING: Gripper friction not updated!"
fi

if grep -q "mass value=\"0.2\"" src/pkg01/urdf/simple_gripper.urdf.xacro; then
    echo "  ✓ Finger mass = 0.2kg (rigid)"
else
    echo "  ✗ WARNING: Finger mass not updated!"
fi

if grep -q "damping=\"20.0\"" src/pkg01/urdf/simple_gripper.urdf.xacro; then
    echo "  ✓ Joint damping = 20.0 (balanced)"
else
    echo "  ✗ WARNING: Joint damping not updated!"
fi

if grep -q "springStiffness" src/pkg01/urdf/simple_gripper.urdf.xacro; then
    if grep -q "<springStiffness>50000.0</springStiffness>" src/pkg01/urdf/simple_gripper.urdf.xacro; then
        echo "  ✓ Implicit spring = 50k N/m (balanced)"
    else
        echo "  ⚠ Spring stiffness value may be incorrect"
    fi
else
    echo "  ✗ WARNING: Spring damper not found!"
fi

if grep -q "<mu>3.0</mu>" src/pkg01/models/bucket/model.sdf; then
    echo "  ✓ Bucket friction = 3.0 (enhanced)"
else
    echo "  ✗ WARNING: Bucket friction not updated!"
fi

if grep -q "p: 5000.0" src/pkg01/controller/ur10e_controllers.yaml; then
    echo "  ✓ Controller P gain = 5000 (balanced)"
else
    echo "  ✗ WARNING: Controller gain not updated!"
fi

# Launch simulation
echo ""
echo "[5/5] Launching farm simulation with MoveIt..."
echo ""
echo "=============================================="
echo "Starting in 2 seconds..."
echo "=============================================="
sleep 2

roslaunch pkg01 gazebo_farm.launch

