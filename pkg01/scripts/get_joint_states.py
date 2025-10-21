#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf

PI = 3.141592653589793

# Initialize
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('set_flange_orientation')
group = moveit_commander.MoveGroupCommander("manipulator")

# Set target pose for flange frame
target_pose = Pose()

# Position (relative to base_link)
target_pose.position.x = -0.7#0.23
target_pose.position.y = 0.0#-0.4
target_pose.position.z = 0.800#0.12#0.630

# Orientation: Convert Euler angles (roll, pitch, yaw) to quaternion
# Example: Point gripper downward (pitch = PI)
roll = PI/2
pitch = 0
yaw = 0

# Or specify custom orientation:
# roll = 0.0      # Rotation around X-axis
# pitch = PI/2    # Rotation around Y-axis (90 degrees)
# yaw = PI/4      # Rotation around Z-axis (45 degrees)

q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
target_pose.orientation.x = q[0]
target_pose.orientation.y = q[1]
target_pose.orientation.z = q[2]
target_pose.orientation.w = q[3]

print(f"Target orientation (RPY): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
print(f"Target orientation (Quaternion): x={q[0]:.3f}, y={q[1]:.3f}, z={q[2]:.3f}, w={q[3]:.3f}")

# Search for multiple IK solutions
print("\nSearching for alternative IK solutions...")
group.set_pose_target(target_pose)

solutions = []
max_attempts = 20

for attempt in range(max_attempts):
    # Randomize starting configuration to find different solutions
    if attempt > 0:
        group.set_random_target()
        rospy.sleep(0.1)
    
    group.set_pose_target(target_pose)
    plan = group.plan()
    
    if plan[0]:
        # Extract joint values from the plan
        if hasattr(plan[1], 'joint_trajectory') and len(plan[1].joint_trajectory.points) > 0:
            joint_values = list(plan[1].joint_trajectory.points[-1].positions)
            
            # Check if this is a new unique solution
            is_unique = True
            for existing_solution in solutions:
                if all(abs(joint_values[i] - existing_solution[i]) < 0.01 for i in range(len(joint_values))):
                    is_unique = False
                    break
            
            if is_unique:
                solutions.append(joint_values)
                print(f"\nSolution {len(solutions)}:")
                print("Joint values:", [f"{v:.4f}" for v in joint_values])
    
    group.clear_pose_targets()
    
    if len(solutions) >= 2:
        break

if len(solutions) > 0:
    print(f"\n\n{'='*60}")
    print(f"Found {len(solutions)} different IK solution(s)")
    print(f"{'='*60}")
    
    joint_names = group.get_active_joints()
    
    # Display all solutions
    for i, solution in enumerate(solutions):
        print(f"\nSolution {i+1}:")
        print("Joint names:", joint_names)
        print("Joint values:", [f"{v:.4f}" for v in solution])
    
    # Ask user to choose
    print(f"\n{'='*60}")
    choice = input(f"Select solution to execute (1-{len(solutions)}): ")
    
    try:
        choice_idx = int(choice) - 1
        if 0 <= choice_idx < len(solutions):
            selected_solution = solutions[choice_idx]
            
            print(f"\nExecuting Solution {choice_idx + 1}:")
            print("Joint values:", [f"{v:.4f}" for v in selected_solution])
            
            # Move to the selected solution
            group.set_joint_value_target(selected_solution)
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            
            print("\nFlange moved to selected IK solution!")
        else:
            print(f"Invalid choice. Please select a number between 1 and {len(solutions)}")
    except ValueError:
        print("Invalid input. Please enter a number.")
else:
    print("\nNo valid IK solution found for this pose")

moveit_commander.roscpp_shutdown()