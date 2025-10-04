#!/usr/bin/env python

"""
Script to control the mobile platform position along the linear rail.
The platform can move from 0 to 10 meters along the X axis.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_platform(position, duration=5.0):
    """
    Move the platform to a specific position.
    
    Args:
        position (float): Target position in meters (0.0 to 10.0)
        duration (float): Time to reach the target position in seconds
    """
    # Validate position
    if position < 0.0 or position > 10.0:
        rospy.logerr("Position must be between 0.0 and 10.0 meters!")
        return False
    
    # Create action client
    client = actionlib.SimpleActionClient(
        '/ur10e_robot/platform_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    
    rospy.loginfo("Waiting for platform controller action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to platform controller!")
    
    # Create goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['platform_joint']
    
    # Create trajectory point
    point = JointTrajectoryPoint()
    point.positions = [position]
    point.velocities = [0.0]
    point.time_from_start = rospy.Duration(duration)
    
    goal.trajectory.points.append(point)
    
    # Send goal
    rospy.loginfo("Moving platform to position: %.2f meters", position)
    client.send_goal(goal)
    
    # Wait for result
    client.wait_for_result()
    result = client.get_result()
    
    if result:
        rospy.loginfo("Platform reached target position!")
        return True
    else:
        rospy.logerr("Failed to move platform")
        return False

def main():
    rospy.init_node('move_platform_node')
    
    rospy.loginfo("Mobile Platform Controller")
    rospy.loginfo("=" * 50)
    rospy.loginfo("This script moves the platform along the linear rail")
    rospy.loginfo("Platform range: 0.0 to 10.0 meters")
    rospy.loginfo("")
    
    # Example usage - move to different positions
    if rospy.has_param('~position'):
        position = rospy.get_param('~position')
        duration = rospy.get_param('~duration', 5.0)
        move_platform(position, duration)
    else:
        # Demo sequence
        rospy.loginfo("Running demo sequence...")
        positions = [0.0, 5.0, 10.0, 2.5, 0.0]
        
        for pos in positions:
            rospy.loginfo("")
            rospy.loginfo("-" * 50)
            if not move_platform(pos, duration=3.0):
                break
            rospy.sleep(1.0)  # Wait a bit between moves
        
        rospy.loginfo("")
        rospy.loginfo("=" * 50)
        rospy.loginfo("Demo complete!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
