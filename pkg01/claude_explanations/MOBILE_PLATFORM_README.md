# Mobile Platform for UR10e Robot

## Overview
The UR10e robot is now mounted on a mobile linear platform that can move from **0 to 10 meters** along the X-axis. This allows the robot to have an extended workspace for larger operations.

## Platform Specifications
- **Motion Type**: Linear (prismatic joint)
- **Axis**: X-axis (forward/backward)
- **Range**: 0.0 to 10.0 meters
- **Max Velocity**: 1.0 m/s
- **Max Effort**: 1000 N
- **Platform Size**: 1.0m x 0.8m x 0.15m
- **Platform Mass**: 50 kg

## Components Added
1. **mobile_platform.urdf.xacro**: New xacro file defining the platform structure
   - `world_platform`: Fixed ground reference
   - `platform_base`: Moving platform base
   - `robot_mount`: Mounting plate for the robot
   - `platform_joint`: Prismatic joint (0-10m range)

2. **platform_controller**: Position controller for the linear motion
   - Located in `ur10e_controllers.yaml`
   - Uses `PositionJointInterface` for precise positioning

3. **move_platform.py**: Python script for controlling platform position
   - Located in `pkg01/scripts/`
   - Provides easy interface to move the platform

## Usage

### 1. Launch the Simulation
```bash
cd ~/lattebot_ws2
catkin_make
source devel/setup.bash
roslaunch pkg01 gazebo_ur10e.launch
```

### 2. Control the Platform

#### Option A: Using the Python Script (Recommended)
Run the demo sequence (moves through several positions):
```bash
rosrun pkg01 move_platform.py
```

Move to a specific position:
```bash
rosrun pkg01 move_platform.py _position:=5.0 _duration:=3.0
```
- `position`: Target position in meters (0.0 to 10.0)
- `duration`: Time to reach position in seconds

#### Option B: Using ROS Topics Directly
```bash
# Send a trajectory goal
rostopic pub /ur10e_robot/platform_controller/command trajectory_msgs/JointTrajectory "
joint_names: ['platform_joint']
points:
  - positions: [5.0]
    time_from_start: {secs: 3, nsecs: 0}"
```

#### Option C: Using ROS Action Client
```python
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

client = actionlib.SimpleActionClient(
    '/ur10e_robot/platform_controller/follow_joint_trajectory',
    FollowJointTrajectoryAction
)
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['platform_joint']
point = JointTrajectoryPoint()
point.positions = [7.5]  # Target position
point.time_from_start = rospy.Duration(5.0)
goal.trajectory.points.append(point)

client.send_goal(goal)
client.wait_for_result()
```

### 3. Monitor Platform Status
```bash
# View joint states (includes platform_joint)
rostopic echo /ur10e_robot/joint_states

# Check controller status
rosservice call /ur10e_robot/controller_manager/list_controllers

# Monitor platform position
rostopic echo /ur10e_robot/joint_states | grep -A 20 platform_joint
```

## Integration with MoveIt
The platform is fully integrated with the simulation. When using MoveIt, the platform joint is available but not part of the arm planning group by default. To plan motions that include platform movement:

1. The platform_joint state is published to `/ur10e_robot/joint_states`
2. Joint state relay forwards this to `/joint_states` for MoveIt
3. You can control platform and arm independently or coordinately

## Coordinate System
- **Origin**: Platform starts at X=0 (world_platform is fixed at origin)
- **X-axis**: Platform moves along positive X direction
- **Robot base**: Mounted 0.2m above platform surface (0.15m platform + 0.05m mount)
- **Full reach**: Robot workspace extends 0-10m along X-axis plus arm reach

## Controller Parameters
The platform controller is configured with:
- **P gain**: 500.0 (position proportional gain)
- **I gain**: 10.0 (integral gain for steady-state error)
- **D gain**: 50.0 (derivative gain for damping)
- **Damping**: 50.0 (joint-level damping)
- **Friction**: 10.0 (simulated friction)

These parameters provide smooth, stable motion with good tracking performance.

## Troubleshooting

### Platform doesn't move
1. Check if controller is loaded:
   ```bash
   rosservice call /ur10e_robot/controller_manager/list_controllers
   ```
   Look for `platform_controller` with state "running"

2. Check for error messages in the Gazebo terminal

3. Verify position is within limits (0.0 to 10.0 meters)

### Platform moves too slowly/quickly
- Adjust the `duration` parameter in the move command
- Modify velocity limits in `mobile_platform.urdf.xacro` (currently 1.0 m/s)

### Controller errors
- Ensure workspace is built: `catkin_make`
- Source the workspace: `source devel/setup.bash`
- Check that transmission interface matches controller type (both use PositionJointInterface)

## Files Modified/Added
- **Added**: `pkg01/urdf/mobile_platform.urdf.xacro`
- **Added**: `pkg01/scripts/move_platform.py`
- **Modified**: `pkg01/urdf/ur10e.urdf.xacro` (includes platform, changes mounting)
- **Modified**: `pkg01/controller/ur10e_controllers.yaml` (adds platform_controller)
- **Modified**: `pkg01/launch/gazebo_ur10e.launch` (spawns platform_controller)

## Next Steps
- Combine platform and arm motion for extended workspace operations
- Add obstacles/environment for collision-free planning
- Implement coordinated platform-arm trajectories
- Add visual markers for platform position in RViz
