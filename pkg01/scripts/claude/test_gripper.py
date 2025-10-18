#!/usr/bin/env python3
"""
Test script to verify gripper mimic joint functionality.
Opens and closes the gripper to confirm right finger mirrors left finger.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def test_gripper():
    """Test gripper open/close motion"""
    rospy.init_node('test_gripper_node')
    
    # Connect to gripper controller action server
    client = actionlib.SimpleActionClient(
        '/ur10e_robot/gripper_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    
    rospy.loginfo("Waiting for gripper controller...")
    if not client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("Gripper controller not available!")
        return
    
    rospy.loginfo("Gripper controller connected!")
    
    # Test sequence: close -> open -> close
    positions = [
        ("CLOSING", -0.01),   # Close gripper
        ("OPENING", 0.02),    # Open gripper
        ("CLOSING", 0.0)      # Return to neutral
    ]
    
    for action, position in positions:
        rospy.loginfo(f"\n{action} gripper to position: {position}")
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(2.0)
        
        goal.trajectory.points.append(point)
        
        client.send_goal(goal)
        client.wait_for_result(timeout=rospy.Duration(5.0))
        
        result = client.get_result()
        if result:
            rospy.loginfo(f"✓ Gripper motion completed: {result.error_code}")
        else:
            rospy.logwarn("⚠ Gripper motion timeout")
        
        rospy.sleep(1.0)
    
    rospy.loginfo("\n✓ Gripper test complete!")
    rospy.loginfo("Check Gazebo/RViz to verify right finger mirrored left finger motion")

if __name__ == '__main__':
    try:
        test_gripper()
    except rospy.ROSInterruptException:
        pass
