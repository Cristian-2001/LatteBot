# Simple Gripper Quick Reference

## Gripper Specifications
```
Design:          Parallel-jaw (2 prismatic fingers)
Finger Length:   130mm
Finger Width:    30mm  
Opening Range:   0mm (nearly touching) to 180mm (wide open)
Max Effort:      2000N per finger
Friction:        μ = 3.0
Mass per Finger: 0.25kg
Starting Gap:    40mm (home position)
```

## Joint Limits
```
Joint:           left_finger_joint, right_finger_joint
Lower Limit:     -0.025m (nearly touching)
Upper Limit:     +0.070m (wide open)
Home Position:   0.000m (40mm separation)
Velocity:        0.5 m/s max
Effort:          2000N max
```

## Common Positions
```python
# Fully open (180mm width)
left_finger:  0.070
right_finger: 0.070

# Grasp position for 15mm handle (16mm width) ✓
left_finger:  -0.012
right_finger: -0.012

# Fully closed (~0mm width, nearly touching)
left_finger:  -0.020
right_finger: -0.020
```

## Quick Commands

### Test Gripper
```bash
rosrun pkg01 test_simple_gripper.py
```

### Monitor Joint States
```bash
rostopic echo /ur10e_robot/joint_states | grep -A2 finger
```

### Manual Control (Python)
```python
import rospy, actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('gripper_control')
client = actionlib.SimpleActionClient(
    '/ur10e_robot/gripper_controller/follow_joint_trajectory',
    FollowJointTrajectoryAction)
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
point = JointTrajectoryPoint()
point.positions = [0.06, 0.06]  # Open
point.time_from_start = rospy.Duration(2.0)
goal.trajectory.points = [point]
client.send_goal_and_wait(goal)
```

### Launch with Simple Gripper
```bash
# Clean restart
./src/pkg01/scripts/restart_simple_gripper.sh

# Or standard launch
roslaunch pkg01 gazebo_farm.launch
```

## Bucket Grasping
```
Bucket Handle:   ~15mm thick, ~90mm opening
Approach:        From above, perpendicular to handle
Grasp Position:  -0.012m (16mm width) ✓ PERFECT FIT
Expected Force:  100-500N per finger
Contact Area:    ~3,900mm² per finger (excellent!)
Safety Factor:   30-150x (gripper can lift 60-300kg!)
```

## Key Files
```
URDF:       pkg01/urdf/simple_gripper.urdf.xacro
SRDF:       ur10e_moveit_config/config/ur10e.srdf
Controller: pkg01/controller/ur10e_controllers.yaml
Test:       pkg01/scripts/test_simple_gripper.py
Docs:       pkg01/SIMPLE_GRIPPER_DESIGN.md
```

## Troubleshooting
```
Issue: Gripper not moving
Fix:   Check controller loaded: rosservice call /ur10e_robot/controller_manager/list_controllers

Issue: Asymmetric closing
Fix:   Increase trajectory duration, check joint limits

Issue: Object slips
Fix:   Ensure grasp position reached, verify friction (μ=3.0)
```

## Comparison with Hook Gripper
```
Feature              Hook    Simple   Winner
-------------------  ------  -------  --------
Link count           9       5        Simple
Collision rules      120     92       Simple
Finger length        90mm    130mm    Simple
Contact area         375mm²  780mm²   Simple
Grasp versatility    Low     High     Simple
Self-collision risk  High    Low      Simple
```
