#!/usr/bin/env python3

"""
Interactive demo that moves the platform through a sequence of positions
while printing status information.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class InteractivePlatformDemo:
    def __init__(self):
        rospy.init_node('interactive_platform_demo')
        
        # Track current position
        self.current_position = 0.0
        rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_state_callback)
        
        # Action client
        self.client = actionlib.SimpleActionClient(
            '/ur10e_robot/platform_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
    def joint_state_callback(self, msg):
        if 'platform_joint' in msg.name:
            idx = msg.name.index('platform_joint')
            self.current_position = msg.position[idx]
    
    def move_to(self, position, duration=3.0):
        """Move platform to target position"""
        print("\n" + "="*60)
        print("Moving platform from {:.2f}m to {:.2f}m".format(
            self.current_position, position))
        print("="*60)
        
        # Create goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['platform_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        
        # Send goal
        self.client.send_goal(goal)
        
        # Show progress
        start_time = rospy.Time.now()
        rate = rospy.Rate(4)  # 4 Hz updates
        
        while not rospy.is_shutdown():
            if self.client.get_state() in [3, 4]:  # SUCCEEDED or ABORTED
                break
                
            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(100, (elapsed / duration) * 100)
            
            # Progress bar
            bar_length = 40
            filled = int(bar_length * progress / 100)
            bar = "█" * filled + "░" * (bar_length - filled)
            
            print("\rProgress: [{}] {:.1f}% | Position: {:.2f}m".format(
                bar, progress, self.current_position), end='')
            
            rate.sleep()
        
        print("\n✓ Reached target position: {:.2f}m\n".format(self.current_position))
        rospy.sleep(1.0)
        
    def run(self):
        print("\n" + "="*60)
        print(" "*15 + "MOBILE PLATFORM INTERACTIVE DEMO")
        print("="*60)
        print("\nThis demo will move the platform through several positions")
        print("to demonstrate its range of motion (0-10 meters).")
        print("\nWaiting for controller...")
        
        self.client.wait_for_server()
        print("✓ Connected to platform controller!\n")
        
        rospy.sleep(2.0)
        
        # Demo sequence with descriptions
        moves = [
            (0.0, "Return to START position", 2.0),
            (2.5, "Move to 25% of range", 3.0),
            (5.0, "Move to MIDDLE position", 3.0),
            (7.5, "Move to 75% of range", 3.0),
            (10.0, "Move to MAXIMUM position", 4.0),
            (5.0, "Return to MIDDLE", 4.0),
            (0.0, "Return to START", 4.0),
        ]
        
        for position, description, duration in moves:
            print("\n" + "-"*60)
            print("NEXT: {}".format(description))
            print("-"*60)
            rospy.sleep(1.0)
            self.move_to(position, duration)
        
        print("\n" + "="*60)
        print(" "*20 + "DEMO COMPLETE!")
        print("="*60)
        print("\nThe platform can move anywhere from 0.0 to 10.0 meters.")
        print("\nTo control it manually, use:")
        print("  rosrun pkg01 move_platform.py _position:=<pos>")
        print("\nWhere <pos> is between 0.0 and 10.0")
        print("")

if __name__ == '__main__':
    try:
        demo = InteractivePlatformDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user.")
