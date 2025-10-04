# Copilot Instructions for UR10e Gazebo Simulation Project

## Project Overview
This is a **ROS (Robot Operating System) Catkin workspace** for simulating a **Universal Robots UR10e manipulator with a custom gripper** in Gazebo. The workspace follows standard ROS package structure with URDF/xacro robot descriptions, Gazebo simulation support, and ros_control integration.

**Workspace Root**: `/home/vboxuser/lattebot_ws2/src`  
**Primary Package**: `pkg01` (contains UR10e robot model and simulation files)

## Architecture & Key Components

### Robot Model (URDF/Xacro)
- **`urdf/ur10e.urdf.xacro`**: Main robot description with 6-DOF UR10e arm
  - Contains full kinematic chain with DH parameters (d1=0.1807, a2=0.6127, a3=0.57155, etc.)
  - Includes inertial properties, collision meshes, and visual meshes
  - **Critical**: Uses `package://pkg01/urdf/meshes/ur10e/` for mesh paths
  - Integrates gripper via xacro include: `<xacro:include filename="$(find pkg01)/urdf/simple_gripper.urdf.xacro"/>`
  
- **`urdf/simple_gripper.urdf.xacro`**: Custom 2-finger gripper macro
  - Attachable to UR10e flange via `gripper_mount` fixed joint
  - Uses **mimic joint plugin** for symmetric finger movement (right finger mirrors left)
  - **Important**: Gripper uses `EffortJointInterface` (not PositionJointInterface)

### Gazebo Integration Pattern
**Key Convention**: Robot components require matching transmissions and controllers:

1. **Transmissions** (in URDF): Define hardware interface type
   - UR10e arm joints → `hardware_interface/PositionJointInterface`
   - Gripper fingers → `hardware_interface/EffortJointInterface`
   
2. **Controllers** (in YAML): Must match transmission interface
   - `arm_controller` → `position_controllers/JointTrajectoryController`
   - `gripper_controller` → `effort_controllers/JointTrajectoryController` ⚠️
   
**Common Error**: Mismatched interfaces cause "Could not find joint in hardware_interface" errors.

### Controller Configuration (`controller/ur10e_controllers.yaml`)
- Namespaced under `ur10e_robot:` (matches `<robotNamespace>` in URDF gazebo plugin)
- Three controllers defined:
  1. `joint_state_controller` - publishes to `/ur10e_robot/joint_states`
  2. `arm_controller` - 6 arm joints with position control (P=1000.0, D=10.0)
  3. `gripper_controller` - left finger with effort control (P=500.0, I=50.0, D=10.0)

## Developer Workflows

### Build & Source
```bash
cd /home/vboxuser/lattebot_ws2
catkin_make
source devel/setup.bash
```

### Launch Simulation
```bash
# Full simulation with Gazebo + RViz
roslaunch pkg01 gazebo_ur10e.launch

# Without RViz
roslaunch pkg01 gazebo_ur10e.launch rviz:=false

# Paused start (for debugging)
roslaunch pkg01 gazebo_ur10e.launch paused:=true
```

### Visualization Only (No Gazebo)
```bash
roslaunch pkg01 ur10e.launch  # Uses joint_state_publisher_gui
```

### Debugging Controllers
```bash
# List loaded controllers
rosservice call /ur10e_robot/controller_manager/list_controllers

# Check joint states
rostopic echo /ur10e_robot/joint_states

# Monitor controller topics
rostopic list | grep ur10e_robot
```

## Project-Specific Conventions

### File Organization
```
pkg01/
├── urdf/          # Robot models (xacro format, not raw URDF)
├── launch/        # Two patterns: gazebo_*.launch (sim) vs *.launch (viz only)
├── controller/    # YAML configs namespaced by robot name
├── config/        # RViz configs
└── meshes/        # STL/DAE files organized by robot model
```

### Xacro Usage Pattern
- Always use `$(find pkg01)` for package-relative paths
- Process with: `$(find xacro)/xacro --inorder <file>.urdf.xacro`
- Gripper attachment: Mount to `flange` link (not `tool0` or `ee_link`)

### Gazebo Plugin Requirements
1. **gazebo_ros_control plugin** in URDF sets namespace
2. **Transmissions** define joint control interfaces
3. **Controller YAML** must be namespaced identically
4. **Mimic joints** require `libroboticsgroup_gazebo_mimic_joint_plugin.so`

### Package Dependencies Pattern
When adding Gazebo features, update `package.xml` with:
- `gazebo_ros`, `gazebo_ros_control`, `gazebo_plugins` (simulation)
- `controller_manager`, `ros_control`, `ros_controllers` (control)
- Specific controller packages: `effort_controllers`, `position_controllers`, `joint_trajectory_controller`

## Critical Implementation Details

### Joint Control Interfaces
**Never mix interfaces without updating both URDF and YAML:**
- If transmission says `PositionJointInterface` → use `position_controllers/*`
- If transmission says `EffortJointInterface` → use `effort_controllers/*`
- Velocity control → `VelocityJointInterface` + `velocity_controllers/*`

### Gripper Control
The gripper uses a **mimic joint pattern**:
- Control only `left_finger_joint` via `/ur10e_robot/gripper_controller`
- `right_finger_joint` automatically mirrors via Gazebo plugin
- Multiplier=1.0, offset=0.0 for symmetric gripping

### Mesh References
All mesh paths use ROS package URLs:
```xml
<mesh filename="package://pkg01/urdf/meshes/ur10e/visual/base.dae"/>
```
Never use absolute paths - breaks portability and roslaunch.

## Common Pitfalls
1. **Forgetting to source workspace**: Always `source devel/setup.bash` after building
2. **Interface mismatch**: Check transmission vs controller type when joints don't load
3. **Namespace errors**: Controller YAML namespace must match URDF `<robotNamespace>`
4. **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y` for missing packages

## Testing New Features
1. Test URDF validity: `check_urdf <(xacro ur10e.urdf.xacro)`
2. Visualize without Gazebo first: `roslaunch pkg01 ur10e.launch`
3. Check for Gazebo errors: Look for controller loading failures in terminal
4. Verify joint control: Use `rqt` or publish to action server: `/ur10e_robot/arm_controller/follow_joint_trajectory`
