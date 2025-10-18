# Simple Parallel-Jaw Gripper Design

## Overview
This document describes the simplified parallel-jaw gripper design for the UR10e mobile manipulation system. The gripper has been redesigned from a hook-style gripper to a simple, robust parallel-jaw configuration optimized for grasping and lifting buckets by their handles.

## Design Philosophy
**Goal**: Create a simple, reliable gripper that can firmly grasp and lift a bucket from its handle without complex hook geometry.

**Key Requirements**:
- Sufficient finger length to wrap around bucket handles
- Wide opening range to accommodate bucket approach
- High friction and rigidity for secure grasping
- Simple geometry for predictable collision behavior
- Independent finger control for symmetric gripping

## Gripper Specifications

### Physical Dimensions
```
Component              | Dimensions (mm)        | Purpose
-----------------------|------------------------|----------------------------------
Gripper Base           | Ø40 × 50              | Mounts to UR10e flange
Gripper Palm           | 60 × 80 × 20          | Connection point for fingers
Left/Right Fingers     | 30 × 20 × 130         | Main gripping surfaces
Gripper Tip (virtual)  | Ø5                    | Reference point for planning
```

### Motion Characteristics
```
Parameter              | Value                  | Notes
-----------------------|------------------------|----------------------------------
Finger Travel          | -15mm to +60mm        | Per finger from home position
Maximum Grip Width     | ~130mm                | Fully open (between finger centers)
Minimum Grip Width     | ~40mm                 | Fully closed
Optimal Handle Grip    | ~90mm separation      | For 15mm thick handles
Closing Speed          | 0.5 m/s max           | Adjustable in trajectories
Effort per Finger      | 2000N max             | High force for secure grip
```

### Material Properties
```
Property               | Value                  | Purpose
-----------------------|------------------------|----------------------------------
Finger Mass            | 0.25kg each           | Sufficient for rigidity
Finger Friction (μ)    | 3.0                   | High grip on bucket handles
Contact Stiffness (kp) | 50M N/m               | Ultra-stiff contact
Contact Damping (kd)   | 5000 N⋅s/m            | Stable contact dynamics
Joint Damping          | 50.0 N⋅s/m            | Controlled motion
Joint Friction         | 10.0 N⋅m              | Realistic resistance
```

## Gripper Geometry

### Coordinate System
The gripper is mounted to the UR10e flange with a -90° rotation around X-axis:
- **Z-axis**: Points along finger length (outward from palm)
- **Y-axis**: Finger opening/closing direction
- **X-axis**: Perpendicular to fingers (along palm width)

### Link Structure
```
flange (UR10e)
  └─ gripper_mount (fixed, -90° X rotation)
      └─ gripper_base_link (50mm height)
          └─ base_to_palm_joint (fixed, 50mm Z offset)
              └─ gripper_palm (20mm height)
                  ├─ left_finger_joint (prismatic, Y-axis)
                  │   └─ left_finger (130mm length)
                  │
                  └─ right_finger_joint (prismatic, -Y-axis)
                      └─ right_finger (130mm length)
```

### Finger Positioning
- **Home Position**: Fingers start at ±35mm from gripper center
- **Closed Position** (-15mm travel): ~40mm separation (firm grip)
- **Grasp Position** (+8mm travel): ~86mm separation (15mm handle)
- **Open Position** (+60mm travel): ~150mm separation (wide opening)

## Grasping Strategy

### For Bucket Handles (15mm thick, ~90mm handle opening)

**Approach Sequence**:
1. **Open Gripper**: Move fingers to +60mm (fully open)
   - Gripper width: ~150mm
   - Provides clearance for bucket approach

2. **Position Above Handle**: Use MoveIt to position gripper over bucket handle
   - Align gripper Z-axis perpendicular to handle
   - Center gripper over handle midpoint

3. **Grasp Handle**: Close fingers to +8mm position
   - Gripper width: ~86mm
   - Fingers wrap around 15mm thick handle
   - High friction (μ=3.0) ensures secure grip

4. **Verify Grasp**: Monitor finger effort
   - Effort > 100N indicates solid contact
   - Both fingers should show similar effort

5. **Lift**: Execute upward trajectory
   - Gripper rigidity prevents deflection
   - Ultra-stiff contacts (kp=50M) prevent slip

## Control Integration

### ROS Control
Both fingers use `PositionJointInterface` for synchronized control:
- **Controller**: `gripper_controller` (JointTrajectoryController)
- **Namespace**: `/ur10e_robot/gripper_controller`
- **Action**: `follow_joint_trajectory`

### MoveIt Integration
- **End Effector Group**: `gripper`
- **Joints**: `left_finger_joint`, `right_finger_joint`
- **Named States**:
  - `open`: Both fingers at +0.06m
  - `close`: Both fingers at -0.01m

### Controller Gains
```yaml
gripper_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - left_finger_joint
    - right_finger_joint
  gains:
    left_finger_joint:  {p: 500, i: 50, d: 10}
    right_finger_joint: {p: 500, i: 50, d: 10}
```

## Physics Configuration

### Gazebo Properties
Each finger link has the following Gazebo configuration:
```xml
<gazebo reference="left_finger">
  <material>Gazebo/DarkGrey</material>
  <mu1>3.0</mu1>              <!-- Primary friction -->
  <mu2>3.0</mu2>              <!-- Secondary friction -->
  <kp>50000000.0</kp>         <!-- Contact stiffness (50M) -->
  <kd>5000.0</kd>             <!-- Contact damping -->
  <minDepth>0.00001</minDepth><!-- Min penetration (0.01mm) -->
  <maxVel>0.001</maxVel>      <!-- Max correction velocity -->
  <fdir1>0 0 1</fdir1>        <!-- Primary friction direction -->
  <softCfm>0.0</softCfm>      <!-- Constraint force mixing -->
  <softErp>0.9</softErp>      <!-- Error reduction parameter -->
  <selfCollide>true</selfCollide>
  <gravity>false</gravity>    <!-- Disable gravity for rigidity -->
</gazebo>
```

### World Physics (farm.world)
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>        <!-- 1ms steps -->
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>                       <!-- High iteration count -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>                           <!-- Rigid constraints -->
      <erp>0.9</erp>                           <!-- High error reduction -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.0001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Collision Configuration

### SRDF Collision Disables
The simplified gripper requires minimal collision disable rules:
```xml
<!-- Fingers with each other -->
<disable_collisions link1="left_finger" link2="right_finger" reason="Default"/>

<!-- Fingers with palm/base -->
<disable_collisions link1="gripper_palm" link2="left_finger" reason="Adjacent"/>
<disable_collisions link1="gripper_palm" link2="right_finger" reason="Adjacent"/>
<disable_collisions link1="gripper_base_link" link2="left_finger" reason="Never"/>
<disable_collisions link1="gripper_base_link" link2="right_finger" reason="Never"/>

<!-- Gripper with arm links -->
<disable_collisions link1="left_finger" link2="wrist_1_link" reason="Never"/>
<disable_collisions link1="left_finger" link2="wrist_2_link" reason="Never"/>
<disable_collisions link1="left_finger" link2="wrist_3_link" reason="Never"/>
<!-- ... similar for right_finger ... -->
```

**Note**: Hook links removed, significantly simplifying collision matrix.

## Testing and Validation

### Test Scripts
1. **`test_simple_gripper.py`**: Basic gripper motion test
   - Open/close sequences
   - Grasp position simulation
   - Joint state monitoring

2. **`test_gripper_lift.py`**: Full grasp-and-lift sequence (existing)
   - Approach bucket
   - Grasp handle
   - Lift and verify

### Expected Performance
```
Metric                     | Target           | Pass Criteria
---------------------------|------------------|---------------------------
Max Opening Width          | ≥130mm           | Can approach bucket
Grasp Force                | 100-500N         | Secure without crushing
Finger Deflection at Lift  | <0.5mm           | Rigid under load
Grasp Success Rate         | >90%             | Reliable operation
Lift Capacity              | ≥2kg             | Exceeds bucket mass
```

### Debugging Tools
```bash
# Monitor gripper state
rostopic echo /ur10e_robot/joint_states | grep finger

# Check gripper width
rosrun pkg01 test_simple_gripper.py

# Verify physics
rosrun pkg01 monitor_gripper_rigidity.py
```

## Comparison with Hook Gripper

| Feature               | Hook Gripper      | Simple Gripper    | Advantage          |
|-----------------------|-------------------|-------------------|--------------------|
| Link Count            | 7                 | 3                 | Simpler            |
| Collision Rules       | 28 (hooks only)   | 4                 | Easier management  |
| Grasp Strategy        | Hook under handle | Pinch sides       | More versatile     |
| Geometry Complexity   | High (3 links/finger) | Low (1 link/finger) | Easier to model |
| Self-Collision Risk   | High              | Low               | More reliable      |
| Computation          | Higher            | Lower             | Better performance |

## Advantages of Simple Design

1. **Predictable Behavior**: Simple geometry = predictable collision detection
2. **Lower Computational Load**: Fewer links and collision checks
3. **Easier Maintenance**: Simpler URDF and collision configuration
4. **More Versatile**: Can grasp various objects, not just handles
5. **Robust Grasping**: Direct pinch grip more reliable than hooking
6. **Better Visualization**: Cleaner appearance in RViz and Gazebo

## Limitations and Considerations

1. **Handle Orientation**: Works best when gripper approaches perpendicular to handle
2. **Maximum Handle Size**: Limited to ~120mm diameter handles
3. **Minimum Handle Size**: Requires at least ~10mm thickness for firm grip
4. **Cylindrical Objects**: Best for handles, bars, and similar geometry

## Future Enhancements

### Potential Improvements
1. **Tactile Feedback**: Add force/torque sensors for better grasp detection
2. **Adaptive Grasping**: Adjust grip force based on object weight
3. **Finger Pads**: Soft rubber pads for better grip on smooth surfaces
4. **Parallel Closing**: Ensure perfectly synchronized finger motion

### Grasp Planning
Consider integrating with MoveIt grasp planning:
```python
# Example grasp pose for bucket handle
grasp = Grasp()
grasp.grasp_pose.pose.position.z = 0.50  # Handle height
grasp.pre_grasp_approach.direction.vector.z = -1.0  # Approach from above
grasp.post_grasp_retreat.direction.vector.z = 1.0   # Lift upward
```

## Usage Examples

### Opening/Closing in Python
```python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_gripper(left_pos, right_pos, duration=2.0):
    client = actionlib.SimpleActionClient(
        '/ur10e_robot/gripper_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    client.wait_for_server()
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [left_pos, right_pos]
    point.time_from_start = rospy.Duration(duration)
    goal.trajectory.points.append(point)
    
    client.send_goal_and_wait(goal)

# Open gripper
move_gripper(0.06, 0.06)

# Grasp position (for 15mm handle)
move_gripper(0.008, 0.008)

# Close gripper
move_gripper(-0.01, -0.01)
```

### Using MoveIt Named States
```python
from moveit_commander import MoveGroupCommander

gripper = MoveGroupCommander("gripper")

# Open gripper
gripper.set_named_target("open")
gripper.go(wait=True)

# Close gripper
gripper.set_named_target("close")
gripper.go(wait=True)
```

## Troubleshooting

### Issue: Fingers Don't Close Symmetrically
**Symptoms**: One finger moves faster or further than the other
**Causes**: 
- Controller gain mismatch
- Joint friction differences
- Trajectory timing issues

**Solutions**:
1. Verify both joints listed in controller YAML
2. Check joint state feedback: `rostopic echo /ur10e_robot/joint_states`
3. Increase trajectory duration for smoother motion
4. Verify equal PID gains for both finger joints

### Issue: Object Slips During Lift
**Symptoms**: Bucket drops when lifting
**Causes**:
- Insufficient friction
- Contact stiffness too low
- Gripper not fully closed

**Solutions**:
1. Increase friction: `<mu1>3.0</mu1>` (already set)
2. Ensure grasp position reached: monitor joint states
3. Increase contact stiffness: `<kp>50000000.0</kp>` (already set)
4. Slow down lift trajectory

### Issue: Fingers Collide with Palm
**Symptoms**: MoveIt shows collision errors
**Causes**:
- Missing collision disable rules
- Finger travel exceeds limits

**Solutions**:
1. Check SRDF has `<disable_collisions link1="gripper_palm" link2="left_finger" reason="Adjacent"/>`
2. Verify finger joint limits: `-0.015` to `0.06`
3. Rebuild workspace after SRDF changes

## References

- **URDF**: `pkg01/urdf/simple_gripper.urdf.xacro`
- **SRDF**: `ur10e_moveit_config/config/ur10e.srdf`
- **Controllers**: `pkg01/controller/ur10e_controllers.yaml`
- **Test Script**: `pkg01/scripts/test_simple_gripper.py`
- **World File**: `pkg01/world/farm.world`
- **Bucket Model**: `pkg01/models/bucket/model.sdf`

## Revision History

| Date       | Version | Changes                                      |
|------------|---------|----------------------------------------------|
| 2025-10-13 | 1.0     | Initial simple gripper design replacing hooks|
