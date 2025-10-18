# Robotiq 2F-140 Gripper Integration

## Summary
Successfully replaced the custom simple gripper with the **Robotiq 2F-140** gripper from the `robotiq` ROS package.

## Changes Made

### 1. URDF Modification (`pkg01/urdf/ur10e.urdf.xacro`)

**Removed:**
- Custom simple gripper include and instantiation
- Old gripper mount configuration

**Added:**
- Robotiq 2F-140 gripper include from `robotiq_2f_140_gripper_visualization` package
- New gripper instantiation with empty prefix
- Updated mounting joint with orientation: `rpy="0 0 ${PI/2}"`

### 2. Controller Configuration (`pkg01/controller/ur10e_controllers.yaml`)

**Changed:**
- Gripper controller now controls single `finger_joint` instead of two independent finger joints
- The Robotiq gripper uses mimic joints - only `finger_joint` is actuated
- Updated controller gains for Robotiq gripper characteristics
- Joint limits: 0.0 (open) to 0.7 radians (closed)

### 3. MoveIt Configuration Files

#### `ur10e_moveit_config/config/ros_controllers.yaml`
- Updated gripper controller to control only `finger_joint`

#### `ur10e_moveit_config/config/simple_moveit_controllers.yaml`
- Updated gripper controller to control only `finger_joint`

#### `ur10e_moveit_config/config/ur10e.srdf`
- Updated gripper group to use `finger_joint` 
- Updated gripper states:
  - `open`: finger_joint = 0.0
  - `close`: finger_joint = 0.7
- Removed old custom gripper collision disable rules
- Added comprehensive Robotiq gripper collision disable rules for:
  - `robotiq_arg2f_base_link`
  - `left/right_outer_knuckle`
  - `left/right_outer_finger`
  - `left/right_inner_finger`
  - `left/right_inner_knuckle`
  - `left/right_inner_finger_pad`

### 4. Package Dependencies

**Disabled packages** (added CATKIN_IGNORE to avoid ethercat dependency):
- `robotiq_ethercat`
- `robotiq_2f_gripper_control`
- `robotiq_modbus_rtu`
- `robotiq_modbus_tcp`
- `robotiq_2f_gripper_action_server`
- `robotiq_3f_*` (all 3-finger gripper packages)
- `robotiq_ft_sensor`

Only the visualization packages are enabled, which is sufficient for Gazebo simulation.

## Robotiq 2F-140 Gripper Structure

### Link Hierarchy
```
robotiq_arg2f_base_link (mounted to flange)
├─ left_outer_knuckle (actuated via finger_joint)
│  └─ left_outer_finger (fixed)
│     └─ left_inner_finger (mimic)
│        └─ left_inner_finger_pad
├─ left_inner_knuckle (mimic)
├─ right_outer_knuckle (mimic, opposite direction)
│  └─ right_outer_finger (fixed)
│     └─ right_inner_finger (mimic)
│        └─ right_inner_finger_pad
└─ right_inner_knuckle (mimic)
```

### Joint Control
- **Primary actuated joint**: `finger_joint` (0.0 to 0.7 radians)
  - Controls `left_outer_knuckle` directly
- **Mimic joints**: All other finger joints follow `finger_joint` via mimic constraints
  - `right_outer_knuckle_joint`: mimics with multiplier=-1 (opposite direction)
  - `left/right_inner_knuckle_joint`: mimic with multiplier=-1
  - `left/right_inner_finger_joint`: mimic with multiplier=1

### Transmission
- Only `finger_joint` has a transmission with `PositionJointInterface`
- Mimic joints are handled by Gazebo's mimic joint plugin (already included in Robotiq URDF)

## Testing

### Launch Simulation
```bash
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash

# Standard simulation
roslaunch pkg01 gazebo_ur10e.launch

# With MoveIt for motion planning
roslaunch pkg01 gazebo_moveit.launch

# Farm world with bucket
roslaunch pkg01 gazebo_farm.launch
```

### Test Gripper Control
```bash
# Run gripper test script
rosrun pkg01 test_robotiq_gripper.py
```

The test script will:
1. Open the gripper (finger_joint = 0.0)
2. Close the gripper (finger_joint = 0.7)
3. Open the gripper again

### Manual Control
```bash
# Check gripper controller is loaded
rosservice call /ur10e_robot/controller_manager/list_controllers

# Monitor joint states
rostopic echo /ur10e_robot/joint_states | grep finger_joint

# Send manual position command (example using rostopic)
rostopic pub -1 /ur10e_robot/gripper_controller/command trajectory_msgs/JointTrajectory "{
  joint_names: ['finger_joint'],
  points: [{positions: [0.5], time_from_start: {secs: 2}}]
}"
```

### Verify in RViz
- The Robotiq gripper should be visible at the end effector
- In MoveIt's Motion Planning plugin, select the "gripper" group
- Use named states "open" and "close" to test gripper motion
- The gripper will show the characteristic parallel-jaw Robotiq design

## Gripper Specifications (Robotiq 2F-140)

- **Type**: Adaptive parallel-jaw gripper
- **Stroke**: 140mm between fingers
- **Joint Range**: 0.0 rad (fully open) to 0.7 rad (fully closed)
- **Control Interface**: Single `finger_joint` with mimic joints for realistic mechanism
- **Finger Pads**: Included for better contact simulation

## Key Differences from Custom Gripper

| Feature | Custom Gripper | Robotiq 2F-140 |
|---------|----------------|----------------|
| Joint Control | 2 independent joints | 1 actuated + mimic joints |
| Mechanism | Simple parallel | Linkage mechanism |
| Link Structure | 2 fingers + pads | Complex linkage (10+ links) |
| Realism | Basic | Industrial gripper |
| Controller Config | 2 joints in controller | 1 joint in controller |

## Benefits

1. **Industrial Standard**: Uses real Robotiq gripper model
2. **Realistic Kinematics**: Proper linkage mechanism with mimic joints
3. **Better Visualization**: Professional gripper appearance
4. **Compatibility**: Standard ROS package, well-maintained
5. **Simulation Accuracy**: More accurate grasping dynamics

## Notes

- The Robotiq gripper orientation is adjusted with `rpy="0 0 ${PI/2}"` to align properly with the UR10e flange
- Mimic joint constraints are handled automatically by the Robotiq URDF (uses `libroboticsgroup_gazebo_mimic_joint_plugin.so`)
- Controller only needs to command `finger_joint` - all other joints follow automatically
- For real robot integration, additional packages (ethercat, modbus) would be needed, but they're not required for Gazebo simulation

## Troubleshooting

### Gripper not visible in Gazebo/RViz
- Verify URDF loads without errors: `xacro ur10e.urdf.xacro | check_urdf`
- Check for missing meshes in `/robotiq_2f_140_gripper_visualization/meshes/`

### Gripper not responding to commands
- Check controller status: `rosservice call /ur10e_robot/controller_manager/list_controllers`
- Verify `finger_joint` is in controller: `rostopic echo /ur10e_robot/gripper_controller/state`
- Ensure action server is available: `rostopic list | grep gripper_controller`

### START_STATE_IN_COLLISION errors in MoveIt
- Run MoveIt Setup Assistant to regenerate collision rules
- Or manually add missing collision disable rules in SRDF

## Future Enhancements

1. Add Gazebo surface physics for better grasping (friction, contact parameters)
2. Tune controller gains for smoother gripper motion
3. Create named gripper states for partial openings (e.g., "half_open")
4. Add force/torque sensing if using Robotiq's FT sensor variant
