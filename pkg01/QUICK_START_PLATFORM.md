# Quick Start Guide: Mobile Platform

## What Was Added
Your UR10e robot is now mounted on a **linear mobile platform** that can move from **0 to 10 meters** along the X-axis, greatly extending the robot's working area.

## Quick Test (3 Steps)

### Step 1: Build and Source
```bash
cd ~/lattebot_ws2
catkin_make
source devel/setup.bash
```

### Step 2: Launch Simulation
```bash
roslaunch pkg01 gazebo_ur10e.launch
```
This will open Gazebo with the robot on the mobile platform and RViz for visualization.

### Step 3: Move the Platform
In a **new terminal**:
```bash
cd ~/lattebot_ws2
source devel/setup.bash

# Run demo (moves through several positions)
rosrun pkg01 move_platform.py

# OR move to a specific position (0.0 to 10.0 meters)
rosrun pkg01 move_platform.py _position:=7.5 _duration:=4.0
```

## What You'll See
- **Gazebo**: The platform will smoothly slide along the X-axis
- **RViz**: The robot's TF frames will update as it moves
- Watch the platform carry the entire robot arm to the new position!

## Verification
To verify everything is working correctly:
```bash
# In a new terminal (with sourced workspace)
rosrun pkg01 verify_platform.py
```

This checks:
- ✓ Simulation is running
- ✓ Platform joint is present
- ✓ Platform controller is loaded

## Platform Control Examples

### Example 1: Move to Middle Position
```bash
rosrun pkg01 move_platform.py _position:=5.0 _duration:=3.0
```

### Example 2: Move to Far End
```bash
rosrun pkg01 move_platform.py _position:=10.0 _duration:=5.0
```

### Example 3: Return to Start
```bash
rosrun pkg01 move_platform.py _position:=0.0 _duration:=5.0
```

## Monitor Platform Status
```bash
# Watch joint states (look for platform_joint)
rostopic echo /ur10e_robot/joint_states | grep -A 3 platform_joint

# List all controllers
rosservice call /ur10e_robot/controller_manager/list_controllers
```

## Architecture Summary

### Hardware Structure
```
world_platform (fixed to ground)
    └── platform_joint (prismatic, 0-10m)
        └── platform_base (1.0m x 0.8m box)
            └── robot_mount (mounting plate)
                └── base_link (UR10e arm base)
                    └── ... (6-DOF arm) ...
                        └── gripper
```

### Controllers
- `joint_state_controller`: Publishes all joint states
- `platform_controller`: Position control for linear motion (NEW)
- `arm_controller`: Position control for 6 arm joints
- `gripper_controller`: Effort control for gripper

## Files Added/Modified

### New Files
- `urdf/mobile_platform.urdf.xacro` - Platform URDF definition
- `scripts/move_platform.py` - Python control interface
- `scripts/verify_platform.py` - Verification script
- `scripts/platform_range_markers.py` - RViz visualization
- `launch/gazebo_platform_demo.launch` - Demo launch file
- `MOBILE_PLATFORM_README.md` - Detailed documentation
- `QUICK_START_PLATFORM.md` - This file

### Modified Files
- `urdf/ur10e.urdf.xacro` - Added platform include and mounting
- `controller/ur10e_controllers.yaml` - Added platform_controller
- `launch/gazebo_ur10e.launch` - Spawns platform_controller

## Integration with MoveIt
The mobile platform works seamlessly with MoveIt:

```bash
# Launch with MoveIt integration
roslaunch pkg01 gazebo_moveit.launch
```

The platform joint appears in the joint states but is not part of the arm planning group. You can:
1. Control the arm using MoveIt (ignores platform)
2. Control the platform independently using the move_platform.py script
3. Coordinate both for extended workspace operations

## Troubleshooting

### "Controller not found" error
```bash
# Make sure workspace is sourced
source ~/lattebot_ws2/devel/setup.bash

# Check if controller loaded
rosservice call /ur10e_robot/controller_manager/list_controllers
```

### Platform doesn't move
1. Check position is between 0.0 and 10.0 meters
2. Verify simulation is running (check Gazebo window)
3. Run verification: `rosrun pkg01 verify_platform.py`

### Build errors
```bash
cd ~/lattebot_ws2
catkin_make clean
catkin_make
source devel/setup.bash
```

## Next Steps
- Combine platform and arm motion for extended reach tasks
- Create trajectories that use both platform and arm
- Add sensors or cameras to the platform
- Implement coordinated platform-arm path planning

## Support
For detailed documentation, see `MOBILE_PLATFORM_README.md` in the pkg01 directory.

---
**Platform Specifications**
- Range: 0.0 to 10.0 meters (X-axis)
- Max velocity: 1.0 m/s
- Control: Position-based trajectory following
- Interface: ROS action server (FollowJointTrajectory)
