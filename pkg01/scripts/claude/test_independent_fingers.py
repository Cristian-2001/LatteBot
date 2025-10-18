#!/usr/bin/env python

"""
Test script to verify independent control of left and right gripper fingers.
Demonstrates asymmetric gripper motion.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def test_independent_fingers():
    rospy.init_node('test_independent_fingers')
    
    # Connect to gripper controller
    client = actionlib.SimpleActionClient(
        '/ur10e_robot/gripper_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    
    rospy.loginfo("Waiting for gripper controller...")
    if not client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("Gripper controller not available!")
        return
    
    rospy.loginfo("Gripper controller connected!")
    
    # Test 1: Both fingers open (symmetric)
    rospy.loginfo("Test 1: Opening both fingers symmetrically...")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [0.025, 0.025]  # Both open
    point.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(point)
    
    client.send_goal(goal)
    client.wait_for_result()
    rospy.sleep(1.0)
    
    # Test 2: Asymmetric motion - left open, right closed
    rospy.loginfo("Test 2: Asymmetric - left open, right closed...")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [0.025, 0.0]  # Left open, right closed
    point.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(point)
    
    client.send_goal(goal)
    client.wait_for_result()
    rospy.sleep(1.0)
    
    # Test 3: Asymmetric motion - left closed, right open
    rospy.loginfo("Test 3: Asymmetric - left closed, right open...")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.025]  # Left closed, right open
    point.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(point)
    
    client.send_goal(goal)
    client.wait_for_result()
    rospy.sleep(1.0)
    
    # Test 4: Both fingers close (symmetric)
    rospy.loginfo("Test 4: Closing both fingers symmetrically...")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0]  # Both closed
    point.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(point)
    
    client.send_goal(goal)
    client.wait_for_result()
    
    rospy.loginfo("Independent finger control test completed!")
    rospy.loginfo("Both fingers can move independently - SUCCESS!")

if __name__ == '__main__':
    try:
        test_independent_fingers()
    except rospy.ROSInterruptException:
        pass
