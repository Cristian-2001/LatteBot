import sys

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import tf

# Initialize
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('save_pose_example')
group = moveit_commander.MoveGroupCommander("arm")

# Set your target pose and compute IK
target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.2
target_pose.position.z = 0.3

q = tf.transformations.quaternion_from_euler()
target_pose.orientation.w = 1.0

group.set_pose_target(target_pose)
plan = group.plan()  # This computes IK without moving

# Save the joint values
if plan[0]:  # Check if planning succeeded
    saved_joint_values = group.get_current_joint_values()  # Or extract from plan
    print("Saved joint values:", saved_joint_values)

# Later, reuse these joint values directly (no IK needed)
group.set_joint_value_target(saved_joint_values)
group.go(wait=True)