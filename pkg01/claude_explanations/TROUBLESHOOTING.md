# Troubleshooting Guide for Mobile Platform

## Issue: White pieces in RViz, errors, nothing shows in Gazebo

### Root Cause
The original implementation had several issues:
1. **Wrong order**: Tried to use `robot_mount` before the platform was instantiated
2. **Missing reference**: Referenced `world_platform` before it was created
3. **Gripper controller**: Only controlled one finger joint instead of both

### Fixes Applied

#### 1. Fixed URDF Structure Order
**File**: `pkg01/urdf/ur10e.urdf.xacro`

Changed from:
```xml
<!-- WRONG ORDER -->
<joint name="robot_mount_to_base" type="fixed">
  <parent link="robot_mount"/>  <!-- doesn't exist yet! -->
  ...
</joint>

<xacro:include filename=".../mobile_platform.urdf.xacro"/>
<xacro:mobile_platform prefix=""/>  <!-- creates robot_mount -->
```

To:
```xml
<!-- CORRECT ORDER -->
<xacro:include filename=".../mobile_platform.urdf.xacro"/>
<xacro:mobile_platform prefix=""/>  <!-- creates platform first -->

<joint name="robot_mount_to_base" type="fixed">
  <parent link="robot_mount"/>  <!-- now exists! -->
  ...
</joint>
```

#### 2. Fixed Gripper Controller
**File**: `pkg01/controller/ur10e_controllers.yaml`

Added both finger joints to the gripper controller:
```yaml
gripper_controller:
  joints:
    - left_finger_joint
    - right_finger_joint  # Added this
```

### Verification Steps

Run this test script:
```bash
cd /home/vboxuser/lattebot_ws2
./test_platform_setup.sh
```

Should show:
- ✓ URDF is valid
- ✓ Complete kinematic chain
- ✓ Platform joint found
- ✓ Platform controller configured

### Launch and Test

1. **Terminal 1** - Launch Gazebo:
```bash
cd ~/lattebot_ws2
source devel/setup.bash
roslaunch pkg01 gazebo_ur10e.launch
```

2. **Wait for Gazebo to fully load** (you should see the robot on a platform)

3. **Terminal 2** - Verify everything loaded:
```bash
cd ~/lattebot_ws2
source devel/setup.bash

# Check controllers
rosservice call /ur10e_robot/controller_manager/list_controllers

# Should see:
# - joint_state_controller: running
# - platform_controller: running
# - arm_controller: running
# - gripper_controller: running
```

4. **Terminal 3** - Move the platform:
```bash
cd ~/lattebot_ws2
source devel/setup.bash
rosrun pkg01 move_platform.py _position:=2.0
```

### Expected Behavior

**In Gazebo:**
- You should see a dark grey rectangular platform (1m x 0.8m x 0.15m)
- A grey cylindrical robot mount on top
- The UR10e robot mounted on the platform
- The gripper attached to the robot end effector

**In RViz:**
- All robot links should be properly colored (no white pieces)
- TF frames should be visible
- The robot should update as it moves

### Common Issues and Solutions

#### Issue: "Could not find joint 'platform_joint' in hardware_interface"
**Solution**: The transmission interface doesn't match the controller type.
- Check that `mobile_platform.urdf.xacro` has `PositionJointInterface`
- Check that controller YAML uses `position_controllers/JointTrajectoryController`

#### Issue: White/missing geometry in RViz
**Solution**: URDF structure issue
- Run `check_urdf <(rosrun xacro xacro src/pkg01/urdf/ur10e.urdf.xacro)`
- Verify all links are in the kinematic chain
- Make sure xacro files are included before use

#### Issue: Robot doesn't appear in Gazebo
**Solution**: Check spawn errors
- Look for errors in the launch terminal
- Verify URDF parses: `rosrun xacro xacro src/pkg01/urdf/ur10e.urdf.xacro > /tmp/test.urdf`
- Check for colliding inertial properties

#### Issue: Controllers fail to load
**Solution**: 
```bash
# Check controller manager
rosservice call /ur10e_robot/controller_manager/list_controllers

# Try loading manually
rosservice call /ur10e_robot/controller_manager/load_controller "name: 'platform_controller'"
rosservice call /ur10e_robot/controller_manager/switch_controller "{start_controllers: ['platform_controller'], stop_controllers: [], strictness: 2}"
```

#### Issue: Gazebo crashes or freezes
**Solution**:
- Reduce inertial values if too high
- Check for zero or negative masses
- Verify collision geometries don't interpenetrate

### Diagnostic Commands

```bash
# View complete robot description
rosparam get /robot_description > /tmp/robot_desc.urdf

# Check joint states
rostopic echo /ur10e_robot/joint_states -n 1

# List all TF frames
rosrun tf tf_echo /world_platform /gripper_tip

# Check for TF issues
rosrun tf view_frames
evince frames.pdf
```

### Kinematic Chain Structure

The correct structure should be:
```
world_platform (root, static in Gazebo)
  └─ platform_joint (prismatic, 0-10m)
      └─ platform_base (moving platform)
          └─ platform_to_mount (fixed)
              └─ robot_mount (mounting plate)
                  └─ robot_mount_to_base (fixed)
                      └─ base_link (UR10e base)
                          └─ ... (6-DOF arm) ...
                              └─ flange
                                  └─ gripper_base_link
                                      └─ ... (gripper) ...
```

### Files to Check if Issues Persist

1. **URDF Files**:
   - `pkg01/urdf/ur10e.urdf.xacro`
   - `pkg01/urdf/mobile_platform.urdf.xacro`
   - `pkg01/urdf/simple_gripper.urdf.xacro`

2. **Controller Config**:
   - `pkg01/controller/ur10e_controllers.yaml`

3. **Launch Files**:
   - `pkg01/launch/gazebo_ur10e.launch`

### Manual Controller Test

If automatic spawning fails, try manual control:

```bash
# Terminal 1: Launch without auto-spawning controllers
roslaunch pkg01 gazebo_ur10e.launch

# Terminal 2: Load controllers manually
rosservice call /ur10e_robot/controller_manager/load_controller "name: 'joint_state_controller'"
rosservice call /ur10e_robot/controller_manager/load_controller "name: 'platform_controller'"
rosservice call /ur10e_robot/controller_manager/load_controller "name: 'arm_controller'"
rosservice call /ur10e_robot/controller_manager/load_controller "name: 'gripper_controller'"

# Start controllers
rosservice call /ur10e_robot/controller_manager/switch_controller "{start_controllers: ['joint_state_controller', 'platform_controller', 'arm_controller', 'gripper_controller'], stop_controllers: [], strictness: 2}"
```

### Contact Information

If issues persist:
1. Check ROS logs: `roscd && cd ../log/latest`
2. Check Gazebo logs: Look in Gazebo terminal output
3. Verify all dependencies: `rosdep install --from-paths src --ignore-src -r -y`
4. Rebuild clean: `catkin_make clean && catkin_make`
