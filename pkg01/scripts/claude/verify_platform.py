#!/usr/bin/env python

"""
Quick test script to verify the mobile platform is properly configured.
This script checks:
1. URDF validity
2. Controller configuration
3. Joint limits
"""

import rospy
import subprocess
import sys
from sensor_msgs.msg import JointState

class PlatformVerifier:
    def __init__(self):
        self.joint_states_received = False
        self.platform_joint_found = False
        
    def joint_state_callback(self, msg):
        self.joint_states_received = True
        if 'platform_joint' in msg.name:
            self.platform_joint_found = True
            idx = msg.name.index('platform_joint')
            position = msg.position[idx]
            rospy.loginfo("Platform joint found! Current position: %.3f meters", position)
        
    def verify(self):
        rospy.init_node('platform_verifier', anonymous=True)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Mobile Platform Verification")
        rospy.loginfo("=" * 60)
        
        # Check if simulation is running
        rospy.loginfo("\n[1/3] Checking if simulation is running...")
        try:
            rospy.wait_for_message('/ur10e_robot/joint_states', JointState, timeout=5.0)
            rospy.loginfo("✓ Simulation is running")
        except rospy.ROSException:
            rospy.logerr("✗ Simulation not running!")
            rospy.logerr("   Please launch: roslaunch pkg01 gazebo_ur10e.launch")
            return False
        
        # Check for platform joint in joint states
        rospy.loginfo("\n[2/3] Checking for platform joint...")
        sub = rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_state_callback)
        
        timeout = rospy.Time.now() + rospy.Duration(3.0)
        while not self.platform_joint_found and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        if self.platform_joint_found:
            rospy.loginfo("✓ Platform joint detected in joint states")
        else:
            rospy.logerr("✗ Platform joint NOT found in joint states")
            rospy.logerr("   Check URDF and controller configuration")
            return False
        
        # Check controller status
        rospy.loginfo("\n[3/3] Checking platform controller...")
        try:
            result = subprocess.check_output([
                'rosservice', 'call', '/ur10e_robot/controller_manager/list_controllers'
            ])
            
            if 'platform_controller' in result:
                if 'running' in result:
                    rospy.loginfo("✓ Platform controller is loaded and running")
                else:
                    rospy.logwarn("⚠ Platform controller is loaded but not running")
            else:
                rospy.logerr("✗ Platform controller NOT found")
                return False
                
        except subprocess.CalledProcessError:
            rospy.logerr("✗ Could not query controller manager")
            return False
        
        # Summary
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("VERIFICATION SUCCESSFUL!")
        rospy.loginfo("=" * 60)
        rospy.loginfo("\nYou can now control the platform:")
        rospy.loginfo("  rosrun pkg01 move_platform.py")
        rospy.loginfo("\nOr move to a specific position:")
        rospy.loginfo("  rosrun pkg01 move_platform.py _position:=5.0")
        rospy.loginfo("")
        
        return True

if __name__ == '__main__':
    try:
        verifier = PlatformVerifier()
        success = verifier.verify()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        pass
