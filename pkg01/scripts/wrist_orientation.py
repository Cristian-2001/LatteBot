#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import JointConstraint, Constraints
import tf

PI = 3.141592653589793

# Initialize
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('set_flange_orientation')
group = moveit_commander.MoveGroupCommander("manipulator")

def get_current_joint_states():
    """Print current joint states of the manipulator."""
    current_joint_values = group.get_current_joint_values()
    joint_names = group.get_active_joints()
    
    print("Current joint values:")
    for i, (name, value) in enumerate(zip(joint_names, current_joint_values)):
        print(f"  {i}: {name} = {value:.4f}")
    
    return current_joint_values, joint_names


def set_flange_orientation_wrist_only(target_pose, fixed_joints):
    """
    Set flange orientation by moving only the 3 wrist joints.
    
    Args:
        target_pose: geometry_msgs/Pose - Target pose for the flange
        fixed_joints: list - Joint values for the first 3 joints to keep fixed
    
    Returns:
        bool: True if motion executed, False otherwise
    """
    joint_names = group.get_active_joints()
    
    print(f"\nTarget position (UNCHANGED):")
    print(f"  x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
    print(f"\nTarget orientation w.r.t world frame (Quaternion):")
    print(f"  x={target_pose.orientation.x:.3f}, y={target_pose.orientation.y:.3f}")
    print(f"  z={target_pose.orientation.z:.3f}, w={target_pose.orientation.w:.3f}")
    
    # Search for IK solutions using only wrist joints
    print("\nSearching for IK solutions using only wrist joints (first 3 joints fixed)...")
    group.set_pose_target(target_pose)
    
    solutions = []
    max_attempts = 50
    
    for attempt in range(max_attempts):
        # Randomize only wrist joints to find different solutions
        if attempt > 0:
            random_state = group.get_random_joint_values()
            # Keep first 3 joints fixed, randomize only wrist joints
            random_state[:3] = fixed_joints
            group.set_joint_value_target(random_state)
            rospy.sleep(0.05)
        group.set_pose_target(target_pose)
        
        # Add joint constraints to keep first 3 joints fixed
        constraints = Constraints()
        for i in range(3):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_names[i]
            joint_constraint.position = fixed_joints[i]
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        group.set_path_constraints(constraints)
        plan = group.plan()
        group.clear_path_constraints()
        
        if plan[0]:
            # Extract joint values from the plan
            if hasattr(plan[1], 'joint_trajectory') and len(plan[1].joint_trajectory.points) > 0:
                joint_values = list(plan[1].joint_trajectory.points[-1].positions)
                
                # Verify first 3 joints are still fixed (within tolerance)
                joints_fixed = all(abs(joint_values[i] - fixed_joints[i]) < 0.01 for i in range(3))
                
                if joints_fixed:
                    # Check if this is a unique solution
                    is_unique = True
                    for existing_solution in solutions:
                        if all(abs(joint_values[i] - existing_solution[i]) < 0.01 for i in range(len(joint_values))):
                            is_unique = False
                            break
                    
                    if is_unique:
                        solutions.append(joint_values)
                        wrist_values = joint_values[3:]
                        print(f"\nSolution {len(solutions)}:")
                        print(f"  Wrist joints: {[f'{v:.4f}' for v in wrist_values]}")
        
        group.clear_pose_targets()
        
        if len(solutions) >= 3:
            break
    
    if len(solutions) > 0:
        print(f"\n\n{'='*60}")
        print(f"Found {len(solutions)} different IK solution(s) using only wrist joints")
        print(f"{'='*60}")
        
        # Display all solutions
        current_joint_values = group.get_current_joint_values()
        for i, solution in enumerate(solutions):
            print(f"\nSolution {i+1}:")
            print("  First 3 joints (FIXED):", [f"{v:.4f}" for v in solution[:3]])
            print("  Wrist joints (wrist_1, wrist_2, wrist_3):", [f"{v:.4f}" for v in solution[3:]])
        
        # Ask user to choose
        print(f"\n{'='*60}")
        choice = input(f"Select solution to execute (1-{len(solutions)}), or 'n' to cancel: ")
        
        if choice.lower() != 'n':
            try:
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(solutions):
                    selected_solution = solutions[choice_idx]
                    
                    print(f"\nExecuting Solution {choice_idx + 1}:")
                    print("  Wrist joints change:", [f"{selected_solution[i+3] - current_joint_values[i+3]:.4f}" for i in range(3)])
                    
                    # Move to the selected solution
                    group.set_joint_value_target(selected_solution)
                    group.go(wait=True)
                    group.stop()
                    group.clear_pose_targets()
                    
                    print("\nFlange orientation changed using wrist joints only!")
                    return True
                else:
                    print(f"Invalid choice. Please select a number between 1 and {len(solutions)}")
            except ValueError:
                print("Invalid input. Please enter a number.")
        else:
            print("Motion cancelled.")
    else:
        print("\nNo valid IK solution found using only wrist joints.")
        print("This might mean the target orientation is not reachable without moving the first 3 joints.")
    
    return False


# Get current joint values and pose
current_joint_values, joint_names = get_current_joint_states()
current_pose = group.get_current_pose().pose

# Keep first 3 joints (shoulder_pan, shoulder_lift, elbow) fixed
fixed_joints = current_joint_values[:3]

# Set target pose - keep current position, change only orientation
target_pose = Pose()

# Keep current position UNCHANGED
target_pose.position = current_pose.position
print(f"\nUsing current flange position: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")

# Orientation relative to world frame: Convert Euler angles (roll, pitch, yaw) to quaternion
roll = 0   # Rotation around world X-axis
pitch = 0     # Rotation around world Y-axis
yaw = PI/2       # Rotation around world Z-axis

# Alternative orientations:
# Gripper pointing down: roll=0, pitch=PI, yaw=0
# Gripper horizontal: roll=PI/2, pitch=0, yaw=0

q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
target_pose.orientation.x = q[0]
target_pose.orientation.y = q[1]
target_pose.orientation.z = q[2]
target_pose.orientation.w = q[3]

print(f"Target orientation w.r.t world frame (RPY): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")

# Execute the orientation change
set_flange_orientation_wrist_only(target_pose, fixed_joints)

moveit_commander.roscpp_shutdown()