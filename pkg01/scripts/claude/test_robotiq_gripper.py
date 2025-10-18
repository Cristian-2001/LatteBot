#!/usr/bin/env python3
"""
Test script for Robotiq 2F-140 gripper control.
Opens and closes the gripper using the finger_joint controller.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def control_gripper(position, duration=2.0):
    """
    Control the Robotiq gripper.
    
    Args:
        position: Target position (0.0 = open, 0.7 = closed)
        duration: Time to reach position (seconds)
    """
    rospy.loginfo(f"Controlling gripper to position: {position}")
    
    # Create action client
    client = actionlib.SimpleActionClient(
        '/ur10e_robot/gripper_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    
    rospy.loginfo("Waiting for gripper action server...")
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logerr("Gripper action server not available!")
        return False
    
    # Create trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['finger_joint']
    
    # Add trajectory point
    point = JointTrajectoryPoint()
    point.positions = [position]
    point.velocities = [0.0]
    point.time_from_start = rospy.Duration(duration)
    goal.trajectory.points.append(point)
    
    # Send goal and wait
    rospy.loginfo(f"Sending gripper command...")
    client.send_goal(goal)
    
    if client.wait_for_result(timeout=rospy.Duration(duration + 2.0)):
        result = client.get_result()
        rospy.loginfo(f"Gripper movement completed: {result}")
        return True
    else:
        rospy.logwarn("Gripper movement timed out!")
        return False

def main():
    rospy.init_node('test_robotiq_gripper', anonymous=True)
    rospy.loginfo("=== Robotiq 2F-140 Gripper Test ===")
    
    try:
        # Wait for robot to be ready
        rospy.sleep(2.0)
        
        # Test sequence: Open -> Close -> Open
        rospy.loginfo("\n1. Opening gripper...")
        control_gripper(0.0, duration=2.0)
        rospy.sleep(1.0)
        
        rospy.loginfo("\n2. Closing gripper...")
        control_gripper(0.7, duration=2.0)
        rospy.sleep(1.0)
        
        rospy.loginfo("\n3. Opening gripper again...")
        control_gripper(0.0, duration=2.0)
        
        rospy.loginfo("\n=== Test completed successfully ===")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error during test: {e}")

if __name__ == '__main__':
    main()
