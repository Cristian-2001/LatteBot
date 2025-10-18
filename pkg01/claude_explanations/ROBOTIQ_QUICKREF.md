# Robotiq 2F-140 Quick Reference

## Quick Start

```bash
# Build workspace
cd /home/vboxuser/lattebot_ws2
catkin_make
source devel/setup.bash

# Launch with Robotiq gripper
roslaunch pkg01 gazebo_ur10e.launch

# Test gripper
rosrun pkg01 test_robotiq_gripper.py
```

## Gripper Control Commands

### Open Gripper (Python)
```python
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

client = actionlib.SimpleActionClient(
    '/ur10e_robot/gripper_controller/follow_joint_trajectory',
    FollowJointTrajectoryAction
)
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['finger_joint']
point = JointTrajectoryPoint()
point.positions = [0.0]  # 0.0 = open, 0.7 = closed
point.time_from_start = rospy.Duration(2.0)
goal.trajectory.points.append(point)
client.send_goal_and_wait(goal)
```

### Close Gripper (rostopic)
```bash
rostopic pub -1 /ur10e_robot/gripper_controller/command trajectory_msgs/JointTrajectory "{
  joint_names: ['finger_joint'],
  points: [{positions: [0.7], time_from_start: {secs: 2}}]
}"
```

## MoveIt Named States

Use in MoveIt RViz plugin or Python:

```python
import moveit_commander

gripper_group = moveit_commander.MoveGroupCommander("gripper")
gripper_group.set_named_target("open")   # Opens gripper
gripper_group.go(wait=True)

gripper_group.set_named_target("close")  # Closes gripper
gripper_group.go(wait=True)
```

## Gripper Joint States

- **Joint Name**: `finger_joint`
- **Open Position**: 0.0 radians
- **Closed Position**: 0.7 radians
- **Interface**: `PositionJointInterface`

## Controller Configuration

- **Controller Name**: `/ur10e_robot/gripper_controller`
- **Type**: `position_controllers/JointTrajectoryController`
- **Action**: `follow_joint_trajectory`
- **Controlled Joints**: `finger_joint` (only one - others are mimic)

## Gripper Links

Main links for collision checking:
- `robotiq_arg2f_base_link` - Base attached to flange
- `left_outer_knuckle`, `right_outer_knuckle` - Main knuckles
- `left_inner_finger_pad`, `right_inner_finger_pad` - Contact pads

## Common Issues

**Gripper not moving:**
```bash
# Check controller is running
rosservice call /ur10e_robot/controller_manager/list_controllers

# Should show gripper_controller as "running"
```

**Wrong orientation:**
- Gripper mount uses `rpy="0 0 ${PI/2}"` in URDF
- Modify in `ur10e.urdf.xacro` if needed

**MoveIt collision errors:**
- Check SRDF has collision disable rules for Robotiq links
- See `ROBOTIQ_GRIPPER_INTEGRATION.md` for full list

## Files Modified

1. `pkg01/urdf/ur10e.urdf.xacro` - Gripper include and mount
2. `pkg01/controller/ur10e_controllers.yaml` - Controller config
3. `ur10e_moveit_config/config/ros_controllers.yaml` - MoveIt controller
4. `ur10e_moveit_config/config/simple_moveit_controllers.yaml` - MoveIt controller
5. `ur10e_moveit_config/config/ur10e.srdf` - Gripper group and collisions

## Testing Checklist

- [ ] Workspace builds without errors (`catkin_make`)
- [ ] URDF is valid (`xacro ur10e.urdf.xacro | check_urdf`)
- [ ] Gazebo launches with gripper visible
- [ ] Gripper controller loads (check `list_controllers`)
- [ ] Test script opens/closes gripper successfully
- [ ] MoveIt shows gripper in correct orientation
- [ ] No collision errors when planning with gripper

## Resources

- Full documentation: `claude_explanations/ROBOTIQ_GRIPPER_INTEGRATION.md`
- Test script: `scripts/test_robotiq_gripper.py`
- Robotiq package: `/src/robotiq/robotiq_2f_140_gripper_visualization/`
