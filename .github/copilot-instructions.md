# Copilot Instructions for UR10e Mobile Manipulation System

## Project Overview
This is a **ROS (Robot Operating System) Catkin workspace** for simulating a **Universal Robots UR10e manipulator with Robotiq 2F-140 gripper mounted on a 10-meter linear mobile platform** in Gazebo. The system combines mobile manipulation with MoveIt motion planning for extended workspace operations.

**Workspace Root**: `/home/vboxuser/lattebot_ws2/src`  
**Primary Packages**: 
- `pkg01` - Robot model, platform, controllers, simulation, and custom objects
- `ur10e_moveit_config` - MoveIt planning configuration (auto-generated, manually tuned)
- `robotiq/robotiq_2f_140_gripper_visualization` - Robotiq gripper URDF and meshes

**Key Documentation**: See `pkg01/claude_explanations/*.md` for troubleshooting guides and fix summaries covering collision tuning, physics parameters, Gazebo caching, gripper integration, and control evolution.

## Architecture & Key Components

### Complete System Structure
```
world_platform (fixed ground reference, MUST have inertial properties)
  └─ platform_joint (prismatic, 0-10m X-axis motion)
      └─ platform_base (50kg moving platform)
          └─ robot_mount (mounting plate)
              └─ base_link (UR10e arm base)
                  └─ [6-DOF arm: shoulder_pan → shoulder_lift → elbow → wrist_1/2/3]
                      └─ flange
                          └─ robotiq_arg2f_base_link (Robotiq 2F-140 gripper with linkage mechanism)
```

### Robot Model (URDF/Xacro)
- **`urdf/ur10e.urdf.xacro`**: Main robot description orchestrating all components
  - Includes platform: `<xacro:include filename="$(find pkg01)/urdf/mobile_platform.urdf.xacro"/>`
  - Includes gripper: `<xacro:include filename="$(find robotiq_2f_140_gripper_visualization)/urdf/robotiq_arg2f_140_model_macro.xacro"/>`
  - **Critical ordering**: Platform macro must be instantiated BEFORE connecting base_link
  - Gripper mount orientation: `rpy="0 0 ${PI/2}"` to align with flange
  - Uses `package://pkg01/urdf/meshes/ur10e/` for mesh paths (never absolute paths)
  
- **`urdf/mobile_platform.urdf.xacro`**: Linear motion platform (0-10 meters)
  - **CRITICAL**: `world_platform` link MUST have inertial properties (even if static)
    - Without inertial properties, ros_control parser fails: "joint not in gazebo model" error
    - Use minimal values: `mass="0.001"`, `inertia="0.001"` for virtual/fixed links
  - `platform_joint`: Prismatic joint with `PositionJointInterface`
  - Gazebo static property applied to `world_platform` only (inside macro)
  - Self-contained with transmission for ros_control integration

- **Robotiq 2F-140 Gripper** (`robotiq/robotiq_2f_140_gripper_visualization/urdf/`): Industrial parallel-jaw gripper
  - Attached to UR10e flange via `gripper_mount` fixed joint
  - **Single actuated joint** (`finger_joint`, 0.0-0.7 rad) with mimic constraints for realistic linkage mechanism
  - Complex link structure: base → outer knuckles → outer fingers → inner fingers → finger pads
  - Mimic joints handled by `libroboticsgroup_gazebo_mimic_joint_plugin.so` (included in Robotiq URDF)
  - Only `finger_joint` has transmission (`PositionJointInterface`) - other joints follow automatically
  - **Disabled Robotiq packages** (CATKIN_IGNORE): ethercat, modbus, action_server, 3f grippers (avoid dependencies)
  - Only visualization package needed for Gazebo simulation

### Gazebo Integration Pattern
**Key Convention**: Robot components require matching transmissions and controllers:

1. **Transmissions** (in URDF): Define hardware interface type
   - UR10e arm joints → `hardware_interface/PositionJointInterface`
   - Gripper → `hardware_interface/PositionJointInterface` (single `finger_joint`)
   - Platform joint → `hardware_interface/PositionJointInterface`
   
2. **Controllers** (in YAML): Must match transmission interface
   - `arm_controller` → `position_controllers/JointTrajectoryController`
   - `gripper_controller` → `position_controllers/JointTrajectoryController` (controls `finger_joint`)
   - `platform_controller` → `position_controllers/JointTrajectoryController`
   
**Common Error**: Mismatched interfaces cause "Could not find joint in hardware_interface" errors. If changing transmission type, update both URDF and controller YAML.

### Controller Configuration (`controller/ur10e_controllers.yaml`)
- **Namespace**: All controllers under `ur10e_robot:` (matches `<robotNamespace>` in URDF gazebo plugin)
- **Four controllers** (all spawned together):
  1. `joint_state_controller` - publishes to `/ur10e_robot/joint_states`
  2. `platform_controller` - linear motion (0-10m, PositionJointInterface, P=500, I=10, D=50)
  3. `arm_controller` - 6 arm joints (PositionJointInterface, P=1000, D=10)
  4. `gripper_controller` - **finger_joint** (PositionJointInterface, P=1000, I=50, D=100)
- **Action servers**: Each controller exposes `follow_joint_trajectory` action for trajectory execution
- **MoveIt Integration**: Controllers must be listed in both `ros_controllers.yaml` and `simple_moveit_controllers.yaml` with ALL joints they control

### Gazebo Worlds & Custom Models
- **`world/farm.world`**: Custom SDF world with bucket model
  - Includes: sun, ground_plane, bucket at (2, 2, 2)
  - Requires `GAZEBO_MODEL_PATH` set to `$(find pkg01)/models` in launch file
- **`models/bucket/`**: Custom Gazebo model structure
  - `model.config` - metadata (name, version, sdf pointer)
  - `model.sdf` - SDF definition (links, visuals, collisions, **surface physics**)
  - `meshes/Bucket.dae` - 3D mesh file
  - **Pattern**: Models referenced in worlds use `model://bucket` URI scheme
  - **Critical**: All collision geometries MUST have `<surface>` blocks with friction (`mu`, `mu2`) and contact params (`kp`, `kd`, `max_vel`, `min_depth`) for stable grasping. Without physics properties, objects slip through gripper.

### MoveIt Integration
- **Package**: `ur10e_moveit_config` (auto-generated by MoveIt Setup Assistant)
- **SRDF** (`ur10e.srdf`): Semantic robot description
  - Planning group: `manipulator` (base_link → flange chain, excludes platform)
  - End effector: `gripper` group (includes `finger_joint` only)
  - Virtual joint: `world_platform` to `base_link` (NOT "world" frame)
  - **Extensive collision disable rules** (~90+ rules) covering:
    - Robotiq gripper linkage (outer/inner knuckles, fingers, pads)
    - Gripper components with arm links (shoulder, upper_arm, forearm, wrists)
    - Platform components with arm links (eliminates false collisions)
  - Named states: `home` (all zeros), `ready` (shoulder_lift=-1.57), `grasp`, `transport`, `place`, `lift`, `intermediate`
  - Gripper states: `open` (finger_joint=0.0), `close` (finger_joint=0.7)
  - **Common error**: "START_STATE_IN_COLLISION" = missing collision disable rules for new links

**CRITICAL**: MoveIt requires THREE components to work with Gazebo:
1. **move_group node** - motion planning services
2. **Joint state relay** - forwards `/ur10e_robot/joint_states` → `/joint_states`
3. **MoveIt RViz** - provides interactive markers for planning

**Pattern for adding MoveIt to launch files**:
```xml
<!-- Launch MoveIt move_group -->
<include file="$(find ur10e_moveit_config)/launch/move_group.launch">
  <arg name="allow_trajectory_execution" value="true"/>
</include>

<!-- Relay joint states to MoveIt's expected topic -->
<node name="joint_state_relay" pkg="topic_tools" type="relay" 
      args="/ur10e_robot/joint_states /joint_states"/>

<!-- Launch MoveIt RViz (not standard RViz) -->
<include file="$(find ur10e_moveit_config)/launch/moveit_rviz.launch"/>
```

**Common error**: Launching only Gazebo + RViz without MoveIt = no interactive motion planning available

## Developer Workflows

### Build & Source
```bash
cd /home/vboxuser/lattebot_ws2
catkin_make
source devel/setup.bash
```

### Launch Simulation Scenarios
```bash
# Standard Gazebo simulation with empty world (no motion planning)
roslaunch pkg01 gazebo_ur10e.launch

# Farm world with bucket model + MoveIt (default - enables interactive planning)
roslaunch pkg01 gazebo_farm.launch

# Farm world without MoveIt (visualization only)
roslaunch pkg01 gazebo_farm.launch moveit:=false

# Dedicated MoveIt launch (empty world + motion planning)
roslaunch pkg01 gazebo_moveit.launch

# Platform-only demo (automated movement sequence)
roslaunch pkg01 gazebo_platform_demo.launch

# Without RViz
roslaunch pkg01 gazebo_ur10e.launch rviz:=false

# Paused start (for debugging)
roslaunch pkg01 gazebo_ur10e.launch paused:=true
```

### Visualization Only (No Gazebo)
```bash
roslaunch pkg01 ur10e.launch  # Uses joint_state_publisher_gui
```

### Control Platform Motion
```bash
# Automated demo sequence (0m → 2.5m → 5m → 7.5m → 10m → return)
rosrun pkg01 demo_platform.py

# Move to specific position (0.0 to 10.0 meters)
rosrun pkg01 move_platform.py _position:=5.0 _duration:=3.0

# Verify platform integration
rosrun pkg01 verify_platform.py

# Add RViz range markers for platform visualization
rosrun pkg01 platform_range_markers.py
```

### Debugging Controllers
```bash
# List loaded controllers (should show 4: joint_state, platform, arm, gripper)
rosservice call /ur10e_robot/controller_manager/list_controllers

# Check joint states (includes platform_joint position)
rostopic echo /ur10e_robot/joint_states

# Monitor controller topics
rostopic list | grep ur10e_robot

# Controller action servers
rostopic info /ur10e_robot/platform_controller/follow_joint_trajectory
rostopic info /ur10e_robot/arm_controller/follow_joint_trajectory
rostopic info /ur10e_robot/gripper_controller/follow_joint_trajectory
```

### Testing Robotiq Gripper
```bash
# Test gripper open/close sequence
rosrun pkg01 test_robotiq_gripper.py

# Monitor joint states (check finger_joint position)
rostopic echo /ur10e_robot/joint_states | grep finger_joint

# Manual gripper control (0.0=open, 0.7=closed)
rostopic pub -1 /ur10e_robot/gripper_controller/command trajectory_msgs/JointTrajectory "{
  joint_names: ['finger_joint'],
  points: [{positions: [0.5], time_from_start: {secs: 2}}]
}"
```
**Expected**: Mimic joints follow `finger_joint` automatically, smooth opening/closing motion

### Gazebo Model Refresh (Critical for Development)
**Problem**: Changes to `.sdf`, `.world`, or `.xacro` files don't appear after relaunch.  
**Cause**: Gazebo caches models in memory and `~/.gazebo/models/`.

**Solution**: Use helper script or manual cleanup:
```bash
# Option 1: Use restart scripts (recommended)
cd /home/vboxuser/lattebot_ws2/src/pkg01/scripts
./restart_gazebo_farm.sh        # Farm world
./restart_enhanced_gripper.sh   # With gripper physics verification
./complete_restart.sh           # Full system restart

# Option 2: Manual cleanup
killall -9 gzserver gzclient
sleep 1
rm -rf ~/.gazebo/models/bucket  # Clear cached models
cd /home/vboxuser/lattebot_ws2
source devel/setup.bash
roslaunch pkg01 gazebo_farm.launch
```
**Never** assume Ctrl+C fully cleans up - always verify with `ps aux | grep gz`.

## Project-Specific Conventions

### File Organization
```
pkg01/
├── urdf/          # Robot models (xacro format, not raw URDF)
├── launch/        # Two patterns: gazebo_*.launch (sim) vs *.launch (viz only)
├── controller/    # YAML configs namespaced by robot name
├── config/        # RViz configs
├── scripts/       # Python control scripts (move_platform.py, demo_platform.py, etc.)
├── world/         # Gazebo world files (.world SDF format)
├── models/        # Custom Gazebo models (bucket, etc.) with model.config + model.sdf
└── meshes/        # STL/DAE files organized by robot model
```

### Launch File Patterns
- **Gazebo simulation**: `gazebo_*.launch` (spawns robot, loads controllers, optional RViz)
  - Core components: URDF load → Gazebo spawn → controller spawner → robot_state_publisher
  - Standard args: `paused`, `gui`, `rviz`, `world_name`
- **Visualization only**: `*.launch` (no Gazebo, uses joint_state_publisher_gui)
- **MoveIt integration**: Requires move_group + joint_state_relay + MoveIt RViz
  - `gazebo_moveit.launch` - Full setup: Gazebo + MoveIt
  - `gazebo_farm.launch` - Has optional `moveit:=true` arg (default enabled)
- **Custom worlds**: Set `GAZEBO_MODEL_PATH` env var, specify world file path

### MoveIt Named Poses (SRDF Configuration)
**Adding new named poses**: Edit `ur10e_moveit_config/config/ur10e.srdf`
- **No other files need modification** - SRDF is loaded at launch time
- After adding pose: Rebuild workspace (`catkin_make`) and restart MoveIt nodes
- Named poses available in: MoveIt RViz plugin dropdown, Python API (`move_group.setNamedTarget("pose_name")`)
- **Pattern**: Group states define joint values for specific configurations
  ```xml
  <group_state name="intermediate" group="manipulator">
    <joint name="shoulder_pan_joint" value="2.0636"/>
    <!-- ... other joints ... -->
  </group_state>
  ```
- Current poses: `home`, `ready`, `grasp`, `transport`, `place`, `lift`, `intermediate` (manipulator); `open`, `close` (gripper)

### Python Scripts Convention
All Python scripts in `scripts/` are executable and use ROS action clients:
- Import pattern: `actionlib`, `control_msgs.msg`, `trajectory_msgs.msg`
- Action server endpoint: `/ur10e_robot/<controller_name>/follow_joint_trajectory`
- Position limits enforced: platform (0-10m), arm (joint limits), gripper (effort-based)

### Xacro Usage Pattern
- Always use `$(find pkg01)` for package-relative paths
- Process with: `$(find xacro)/xacro --inorder <file>.urdf.xacro`
- Gripper attachment: Mount to `flange` link (not `tool0` or `ee_link`)
- Macro ordering critical: Platform macro → robot mounting → gripper

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

### Robotiq Package Structure
**Enabled packages** (visualization only, sufficient for Gazebo):
- `robotiq_2f_140_gripper_visualization` - URDF, meshes, visual models
- `robotiq_3f_gripper_articulated_msgs` - Message definitions (dependency)

**Disabled packages** (CATKIN_IGNORE to avoid ethercat/hardware dependencies):
- `robotiq_ethercat`, `robotiq_modbus_rtu`, `robotiq_modbus_tcp` - Hardware communication
- `robotiq_2f_gripper_control`, `robotiq_2f_gripper_action_server` - Real robot control
- `robotiq_3f_*` - All 3-finger gripper packages
- `robotiq_ft_sensor` - Force/torque sensor

**Pattern**: For simulation, only visualization packages needed. Real robot deployment requires enabling hardware packages.

## Critical Implementation Details

### Joint Control Interfaces
**Never mix interfaces without updating both URDF and YAML:**
- If transmission says `PositionJointInterface` → use `position_controllers/*`
- If transmission says `EffortJointInterface` → use `effort_controllers/*`
- Velocity control → `VelocityJointInterface` + `velocity_controllers/*`

### Gripper Control
The gripper uses **Robotiq 2F-140 with mimic joints**:
- Control single `finger_joint` via `/ur10e_robot/gripper_controller`
- Only `finger_joint` is actuated (0.0=open, 0.7=closed) using `PositionJointInterface`
- All other joints (outer/inner knuckles, fingers) follow via mimic constraints automatically
- Mimic plugin (`libroboticsgroup_gazebo_mimic_joint_plugin.so`) handles synchronized motion
- Gripper controller expects only `finger_joint` in trajectory commands

### Mesh References
All mesh paths use ROS package URLs:
```xml
<mesh filename="package://pkg01/urdf/meshes/ur10e/visual/base.dae"/>
```
Never use absolute paths - breaks portability and roslaunch.

### Custom Gazebo Models
Models in `models/` directory require specific structure:
```
bucket/
├── model.config    # Metadata: name, version, SDF pointer
├── model.sdf       # SDF definition with links, visuals, collisions, surface physics
└── meshes/
    └── Bucket.dae  # 3D mesh referenced by model.sdf
```
- Launch files must set: `<env name="GAZEBO_MODEL_PATH" value="$(find pkg01)/models:..."/>`
- World files reference with: `<uri>model://bucket</uri>`

**Surface Physics Requirements** (for graspable objects):
Every `<collision>` in SDF must include:
```xml
<surface>
  <friction>
    <ode>
      <mu>3.0</mu>      <!-- High friction for handles, matches gripper -->
      <mu2>3.0</mu2>
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>10000000.0</kp>  <!-- 10M - ultra-stiff contact (prevents pass-through) -->
      <kd>1000.0</kd>       <!-- 10x damping for stability -->
      <max_vel>0.01</max_vel>       <!-- Slow correction prevents explosive separation -->
      <min_depth>0.0001</min_depth>  <!-- 0.1mm - fine detection for thin geometry -->
    </ode>
  </contact>
</surface>
```
**Why**: Without friction/contact params, objects have near-zero friction (slip through gripper). Ultra-stiff contacts (kp=10M) prevent pass-through during lift. Match gripper friction (`mu=3.0`) for stable grasping. World physics should use 100 ODE solver iterations with ERP=0.9, CFM=0.0 for rigid contacts. See `BUCKET_PASSTHROUGH_FIX.md` for details.

## Common Pitfalls
1. **Forgetting to source workspace**: Always `source devel/setup.bash` after building
2. **Interface mismatch**: Check transmission vs controller type when joints don't load
3. **Namespace errors**: Controller YAML namespace must match URDF `<robotNamespace>`
4. **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y` for missing packages
5. **Missing inertial properties**: All links MUST have inertial properties, even static/dummy links
   - Use minimal values: `mass="0.001"`, `inertia="0.001"` for virtual links
   - Without inertial: "joint not in gazebo model" error from ros_control parser
6. **Xacro macro ordering**: Include platform macro BEFORE connecting base_link to robot_mount
7. **Gazebo model paths**: Set `GAZEBO_MODEL_PATH` env var in launch files for custom models
8. **Gazebo caching**: Model changes not appearing? Kill processes and clear cache (see Debugging section)
9. **Cannot move robot in RViz**: Launch file missing MoveIt integration (move_group + relay + MoveIt RViz)
   - Symptom: RViz shows robot but no interactive markers or Planning tab
   - Fix: Add MoveIt components or use launch file with `moveit:=true` arg
10. **START_STATE_IN_COLLISION error**: Missing collision disable rules in SRDF for new links
    - Add `<disable_collisions>` entries for new links with adjacent/parent links
    - See `COLLISION_FIX.md` for patterns
11. **Objects slipping through gripper**: Missing surface physics in model SDF
    - Add `<surface><friction><ode>` blocks with `mu=3.0` to all collision geometries
    - Add ultra-stiff `<contact>` params: `kp=10000000`, `kd=1000`, `max_vel=0.01`, `min_depth=0.0001`
    - Enable `<self_collide>true</self_collide>` on model and link
    - See `BUCKET_PASSTHROUGH_FIX.md` for comprehensive solution
12. **MoveIt cannot execute gripper**: Controller not listed in MoveIt controller configs
    - Update BOTH `ros_controllers.yaml` AND `simple_moveit_controllers.yaml`
    - List ALL joints the controller manages (for Robotiq: only `finger_joint`)
13. **Gripper not moving in Gazebo**: Check if mimic joint plugin is loaded
    - Verify `libroboticsgroup_gazebo_mimic_joint_plugin.so` in Robotiq URDF
    - Check Gazebo console for plugin loading errors
    - Only `finger_joint` should be commanded - others follow automatically

## Testing New Features
1. Test URDF validity: `check_urdf <(xacro ur10e.urdf.xacro)`
2. Visualize without Gazebo first: `roslaunch pkg01 ur10e.launch`
3. Check for Gazebo errors: Look for controller loading failures in terminal
4. Verify joint control: Use `rqt` or publish to action server: `/ur10e_robot/arm_controller/follow_joint_trajectory`
5. Test platform integration: `rosrun pkg01 verify_platform.py` before launching full simulation

````
