# Copilot Instructions for UR10e Mobile Manipulation System

## Project Overview
This is a **hybrid ROS + MQTT system** for automated dairy farm operations. The system simulates a **Universal Robots UR10e manipulator with Robotiq 2F-140 gripper mounted on a 10-meter linear mobile platform** in Gazebo, controlled via MQTT messaging from Windows-based operator interfaces.

**Architecture**: Multi-platform distributed system
- **Linux/WSL**: ROS Catkin workspace for robot simulation (Gazebo + MoveIt)
- **Windows**: Python MQTT bridges and operator UI (Tkinter keypad interface)
- **Communication**: HiveMQ Cloud broker (TLS, QoS 2) bridges MQTT ↔ ROS topics

**Workspace Roots**:
- Linux: `/home/aldo/Desktop/smerd/lattebot_ws`, `/home/vboxuser/lattebot_ws2`
- Windows: `C:\Users\cicci\Documents\Universita\Magistrale\Smart_Robotics\lattebot`

**Primary Packages**: 
- `pkg01` - Robot model, platform, controllers, simulation, and custom objects
- `ur10e_moveit_config` - MoveIt planning configuration (auto-generated, manually tuned)
- `robotiq/robotiq_2f_140_gripper_visualization` - Robotiq gripper URDF and meshes
- `roboticsgroup_gazebo_plugins` - Mimic joint plugin for gripper linkage

**Key Documentation**: See `pkg01/claude_explanations/*.md` for extensive troubleshooting guides and fix summaries covering collision tuning, physics parameters, Gazebo caching, gripper integration, and control evolution. **ALWAYS consult these files** before making physics or controller changes.

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

## MQTT/ROS Communication Architecture

### System Components
The system uses a **3-tier distributed architecture** for dairy farm automation:

1. **Operator Interface** (`pickup_site.py` on Windows)
   - Tkinter GUI (`numerical_keypad.py`) for planning cow milking sequences
   - User selects cow number (0-9) and milk quantity (liters)
   - Publishes complete sequences to MQTT topic `Pickup-Site` (configurable in `config/config.ini`)
   - Tracks globally used cows to prevent duplicate assignments
   - Generates unique sequence IDs: `SEQ0001_20251025_143022_a3b4c5d6`

2. **MQTT ↔ ROS Bridge** (`bridge_keypad2robot.py` on Linux/WSL)
   - Subscribes to MQTT topics: `Pickup-Site` (sequences), `cow/#` (individual cow notifications)
   - Queue-based processor: Thread-safe sequence execution with lock management
   - Publishes to ROS topic: `/calf_num` (format: `{start_calf}_{end_calf}`, e.g., `-1_3` or `5_7`)
   - Special values: `-1` = platform/base position, `0-9` = cow stall numbers
   - Maintains `sequences_dict` to track partial sequences per cow

3. **Robot Controller** (`robot_movement.py` on Linux/WSL)
   - Subscribes to ROS topic: `/calf_num`
   - Decodes tasks: `base2cow` (-1→N), `cow2cow` (N→M), `cow2base` (N→-1)
   - Maps cow numbers to platform positions: `{0:1m, 1:3m, 2:5m, ..., 9:19m}`
   - Orchestrates: Platform motion → MoveIt arm planning → Gazebo gripper control
   - Spawns/deletes bucket models dynamically in Gazebo simulation

### Data Flow Pattern
```
[Operator GUI] --MQTT(QoS 2)--> [HiveMQ Cloud] --MQTT--> [Bridge] --ROS--> [Robot Controller] --MoveIt/Gazebo--> [Simulation]
     ↓                                                        ↓
[Sequence Planning]                                   [Queue Management]
{cow: 3, liters: 5.2}                                {-1 → 3 → 5 → -1}
```

### MQTT Topics Convention
- **Publish** (pickup_site → bridge):
  - `Pickup-Site`: JSON sequence payload with metadata
    ```json
    {
      "sequence_id": "SEQ0001_20251025_143022_a3b4c5d6",
      "cows": [{"cow": 3, "liters": 5.2}, {"cow": 5, "liters": 4.8}, {"cow": -1, "liters": 0.0}],
      "total_cows": 2,
      "total_liters": 10.0
    }
    ```
- **Subscribe** (bridge):
  - `Pickup-Site`: Main sequence topic (configurable)
  - `cow/#`: Wildcard for individual cow notifications (e.g., `cow/3`, `cow/5`)

### Configuration Management
All MQTT/serial settings in `pkg01/config/config.ini`:
```ini
[MQTT]
Username = cristiancasali
Password = Cristian01
Broker = d5b931520be34f4c9de0a77be2fac4e3.s1.eu.hivemq.cloud
Port = 8883
Topic = Pickup-Site

[Serial]
PortName = COM5
UseDescription = no
```
**CRITICAL**: Scripts use relative paths (`../config/config.ini`) - works from `scripts/` directory

### Windows Development Environment
- **Python Virtual Environment**: `.venv` at workspace root (activate: `.venv\Scripts\Activate.ps1`)
- **Dependencies**: `requirements.txt` includes `paho-mqtt`, `pyserial`, ROS Python packages
- **Shell**: PowerShell - use `;` for command chaining, not `&&`
- **Running bridges**: 
  ```powershell
  # Terminal 1: Operator interface
  & .venv/Scripts/python.exe pkg01/scripts/pickup_site.py
  
  # Terminal 2: Serial bridge (Arduino weight sensors)
  & .venv/Scripts/python.exe pkg01/scripts/bridge_serial2MQTT.py
  
  # Terminal 3: MQTT→ROS bridge (Linux/WSL with ROS sourced)
  python3 pkg01/scripts/bridge_keypad2robot.py
  ```

### Arduino Integration (`bridge_serial2MQTT.py`)
- **Purpose**: Monitors milk bucket weight via Arduino load cells
- **Serial Protocol**: Custom binary protocol (header `\xff`, calf_num, weight bytes)
- **MQTT Publishing**: Publishes to `cow/{calf_num}` when cow finishes drinking
  - Trigger: Weight drops below `starting_weight - limit` OR 20-second timeout
  - Subscribes to `Pickup-Site` to get milk limits per cow
- **Auto-detection**: Finds Arduino by COM port description (configurable)

### Bridge State Management
**Queue Pattern** (`bridge_keypad2robot.py`):
- Thread-safe queue (`Queue[(Calf_num, Sequence)]`) with `Lock` for concurrency
- `platform_sequence`: Currently active sequence on platform (None if free)
- `sequences_dict`: Maps calf numbers to remaining sequence steps
- `is_processing`: Prevents concurrent sequence execution
- **CRITICAL**: Mark platform free (`platform_sequence = None`) after task completion

## Developer Workflows

### Build & Source
```bash
cd ~/Desktop/smerd/lattebot_ws  # Adjust to your workspace path
catkin_make
source devel/setup.bash
```

**CRITICAL**: Always source after building. Add to `~/.bashrc` for persistence:
```bash
echo "source ~/Desktop/smerd/lattebot_ws/devel/setup.bash" >> ~/.bashrc
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

### Windows: Running MQTT Bridges
```powershell
# Ensure virtual environment is activated
.venv\Scripts\Activate.ps1

# Start operator interface (Tkinter GUI)
& .venv\Scripts\python.exe pkg01\scripts\pickup_site.py

# Start serial→MQTT bridge (Arduino weight monitoring)
& .venv\Scripts\python.exe pkg01\scripts\bridge_serial2MQTT.py
```

### Linux/WSL: Running MQTT→ROS Bridge
```bash
# Source ROS workspace first
source devel/setup.bash

# Start bridge (subscribes to MQTT, publishes to ROS)
python3 src/pkg01/scripts/bridge_keypad2robot.py

# Start robot controller (executes movements)
python3 src/pkg01/scripts/robot_movement.py
```

### Complete System Startup (Multi-Platform)
1. **Linux/WSL Terminal 1**: Launch Gazebo simulation
   ```bash
   roslaunch pkg01 gazebo_farm.launch
   ```
2. **Linux/WSL Terminal 2**: Start MQTT→ROS bridge
   ```bash
   python3 src/pkg01/scripts/bridge_keypad2robot.py
   ```
3. **Linux/WSL Terminal 3**: Start robot controller
   ```bash
   python3 src/pkg01/scripts/robot_movement.py
   ```
4. **Windows Terminal 1**: Start operator interface
   ```powershell
   & .venv\Scripts\python.exe pkg01\scripts\pickup_site.py
   ```
5. **Windows Terminal 2** (optional): Start Arduino bridge
   ```powershell
   & .venv\Scripts\python.exe pkg01\scripts\bridge_serial2MQTT.py
   ```

### Control Platform Motion
```bash
# Move to specific position (0.0 to 10.0 meters)
rosrun pkg01 move_platform.py _position:=5.0 _duration:=3.0

# Get current joint states
rosrun pkg01 get_joint_states.py

# Get frame orientation
rosrun pkg01 get_frame_orientation.py

# Check wrist orientation
rosrun pkg01 wrist_orientation.py
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
# Monitor joint states (check finger_joint position)
rostopic echo /ur10e_robot/joint_states | grep finger_joint

# Manual gripper control (0.0=open, 0.7=closed)
rostopic pub -1 /ur10e_robot/gripper_controller/command trajectory_msgs/JointTrajectory "{
  joint_names: ['finger_joint'],
  points: [{positions: [0.5], time_from_start: {secs: 2}}]
}"
```
**Expected**: Mimic joints follow `finger_joint` automatically, smooth opening/closing motion

### Debugging MQTT Communication
```bash
# Check bridge connection status (look for "Successfully connected")
# In bridge_keypad2robot.py output

# Monitor ROS topic receiving MQTT data
rostopic echo /calf_num

# Test MQTT publishing (requires mosquitto-clients)
mosquitto_pub -h d5b931520be34f4c9de0a77be2fac4e3.s1.eu.hivemq.cloud \
  -p 8883 -u cristiancasali -P Cristian01 --capath /etc/ssl/certs/ \
  -t "Pickup-Site" -m '{"sequence_id":"TEST","cows":[{"cow":3,"liters":5.0}],"total_cows":1,"total_liters":5.0}'

# Check bridge subscriptions (should show Pickup-Site and cow/# topics)
# Look for "📬 Subscribed to MQTT topics" in bridge output
```

### Debugging Windows Python Environment
```powershell
# Check virtual environment packages
& .venv\Scripts\pip.exe list | Select-String "paho-mqtt|pyserial|rospy"

# Test MQTT connection from Windows
& .venv\Scripts\python.exe -c "import paho.mqtt.client as mqtt; print('MQTT OK')"

# Check config.ini loading
& .venv\Scripts\python.exe -c "import configparser; c=configparser.ConfigParser(); c.read('pkg01/config/config.ini'); print(c.get('MQTT','Broker'))"

# List COM ports (for Arduino detection)
& .venv\Scripts\python.exe -c "import serial.tools.list_ports; [print(p.device, '-', p.description) for p in serial.tools.list_ports.comports()]"
```

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
      <mu>20.0</mu>      <!-- EXTREME friction - must exceed gripper friction -->
      <mu2>20.0</mu2>
      <fdir1>0 0 1</fdir1>  <!-- Vertical friction for lift forces -->
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>3000000.0</kp>  <!-- 3M - ultra-stiff, exceeds gripper (2M) for stability -->
      <kd>3000.0</kd>     <!-- 3000 damping - maximum stabilization -->
      <max_vel>0.0001</max_vel>     <!-- Ultra-slow correction: 0.1mm/s -->
      <min_depth>0.00005</min_depth>  <!-- 0.05mm - matches gripper exactly -->
    </ode>
  </contact>
  <soft_cfm>0.0</soft_cfm>  <!-- Perfectly rigid contact -->
  <soft_erp>0.2</soft_erp>  <!-- Balanced error correction -->
</surface>
```
**CRITICAL Physics Hierarchy for Stable Grasping**:
- **Friction**: Grasped object (μ=20.0) > Gripper pads (μ=15.0) - prevents "overpowering" slip
- **Stiffness**: Grasped object (kp=3M) > Gripper (kp=2M) - object "pushes back" harder
- **Correction speed**: Grasped object (0.0001) < Gripper (0.0005) - prevents oscillation
- **Contact detection**: Must match exactly (0.00005m) for synchronized contact

**Why**: Gripper has μ=15.0 on finger pads, μ=12.0 on fingers, kp=2M, kd=2000. Objects with lower friction/stiffness will slip during dynamic lifting. The grasped object must be "stronger" than the gripper in all contact properties. World physics should use 100 ODE solver iterations with ERP=0.9, CFM=0.0 for rigid contacts. See `LIFT_SLIP_FIX_2025.md` and `BUCKET_PASSTHROUGH_FIX.md` for detailed physics tuning.

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
    - Add `<surface><friction><ode>` blocks with `mu=20.0` to all collision geometries
    - Add ultra-stiff `<contact>` params: `kp=3000000`, `kd=3000`, `max_vel=0.0001`, `min_depth=0.00005`
    - Add directional friction: `<fdir1>0 0 1</fdir1>` for vertical lift forces
    - Add soft contact: `<soft_cfm>0.0</soft_cfm>`, `<soft_erp>0.2</soft_erp>`
    - Enable `<self_collide>true</self_collide>` on model and link
    - **Rule**: Grasped object friction/stiffness must EXCEED gripper values for stable contact
    - See `LIFT_SLIP_FIX_2025.md` and `BUCKET_PASSTHROUGH_FIX.md` for comprehensive solutions
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
