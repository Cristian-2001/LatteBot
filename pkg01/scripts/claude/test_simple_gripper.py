#!/usr/bin/env python3
"""
Test script for the simplified parallel-jaw gripper.
Demonstrates opening, closing, and grasping capabilities.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class SimpleGripperTest:
    def __init__(self):
        rospy.init_node('simple_gripper_test', anonymous=True)
        
        # Action client for gripper control
        self.gripper_client = actionlib.SimpleActionClient(
            '/ur10e_robot/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for gripper controller...")
        if not self.gripper_client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Gripper controller not available!")
            return
        
        rospy.loginfo("Gripper controller connected!")
        
        # Subscribe to joint states for monitoring
        self.joint_states = None
        self.joint_sub = rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_state_callback)
        
        # Wait for initial joint state
        rospy.sleep(1.0)
    
    def joint_state_callback(self, msg):
        """Store latest joint states"""
        self.joint_states = msg
    
    def move_gripper(self, left_pos, right_pos, duration=2.0):
        """
        Move gripper fingers to specified positions.
        
        Args:
            left_pos: Position for left finger (0=closed, 0.06=fully open)
            right_pos: Position for right finger (0=closed, 0.06=fully open)
            duration: Time to complete motion
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [left_pos, right_pos]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        
        return self.gripper_client.get_result()
    
    def open_gripper(self, duration=2.0):
        """Open gripper to maximum width (~180mm between finger centers)"""
        rospy.loginfo("Opening gripper...")
        return self.move_gripper(0.07, 0.07, duration)
    
    def close_gripper(self, duration=2.0):
        """Close gripper to minimum (nearly touching, ~10mm gap)"""
        rospy.loginfo("Closing gripper...")
        return self.move_gripper(-0.020, -0.020, duration)
    
    def grasp_position(self, duration=2.0):
        """Move to optimal position for grasping 15mm bucket handle"""
        rospy.loginfo("Moving to grasp position (for 15mm handle)...")
        # Position fingers to grip a ~15mm thick handle
        # Start at 40mm (20mm each), close by 12mm (8mm remaining = 16mm gap)
        return self.move_gripper(-0.012, -0.012, duration)
    
    def get_gripper_width(self):
        """Calculate current gripper width from joint states"""
        if self.joint_states is None:
            return None
        
        try:
            idx_left = self.joint_states.name.index('left_finger_joint')
            idx_right = self.joint_states.name.index('right_finger_joint')
            
            left_pos = self.joint_states.position[idx_left]
            right_pos = self.joint_states.position[idx_right]
            
            # Gripper width calculation:
            # Fingers start at 20mm from center (40mm total separation)
            # Width = starting_separation + 2 * finger_displacement
            width_mm = (40.0 + (left_pos * 1000) + (right_pos * 1000))
            
            return width_mm
        except (ValueError, IndexError):
            return None
    
    def print_gripper_state(self):
        """Print current gripper configuration"""
        width = self.get_gripper_width()
        if width is not None:
            rospy.loginfo(f"Current gripper width: {width:.1f}mm")
        
        if self.joint_states:
            try:
                idx_left = self.joint_states.name.index('left_finger_joint')
                idx_right = self.joint_states.name.index('right_finger_joint')
                
                left_pos = self.joint_states.position[idx_left]
                right_pos = self.joint_states.position[idx_right]
                left_effort = self.joint_states.effort[idx_left] if len(self.joint_states.effort) > idx_left else 0
                right_effort = self.joint_states.effort[idx_right] if len(self.joint_states.effort) > idx_right else 0
                
                rospy.loginfo(f"  Left finger:  pos={left_pos:.4f}m  effort={left_effort:.1f}N")
                rospy.loginfo(f"  Right finger: pos={right_pos:.4f}m  effort={right_effort:.1f}N")
            except (ValueError, IndexError):
                pass
    
    def run_test_sequence(self):
        """Run a complete test sequence"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("SIMPLE PARALLEL-JAW GRIPPER TEST SEQUENCE")
        rospy.loginfo("="*60 + "\n")
        
        # Test 1: Open gripper
        rospy.loginfo("Test 1: Opening gripper to maximum width")
        self.open_gripper(duration=3.0)
        rospy.sleep(1.0)
        self.print_gripper_state()
        rospy.loginfo("")
        
        # Test 2: Grasp position for bucket handle
        rospy.loginfo("Test 2: Moving to grasp position for bucket handle")
        rospy.loginfo("(Simulating grip on ~15mm thick handle)")
        self.grasp_position(duration=3.0)
        rospy.sleep(1.0)
        self.print_gripper_state()
        rospy.loginfo("")
        
        # Test 3: Close fully
        rospy.loginfo("Test 3: Closing gripper fully")
        self.close_gripper(duration=3.0)
        rospy.sleep(1.0)
        self.print_gripper_state()
        rospy.loginfo("")
        
        # Test 4: Return to open
        rospy.loginfo("Test 4: Returning to open position")
        self.open_gripper(duration=3.0)
        rospy.sleep(1.0)
        self.print_gripper_state()
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST SEQUENCE COMPLETED")
        rospy.loginfo("="*60 + "\n")
        
        rospy.loginfo("Gripper Specifications:")
        rospy.loginfo("  - Finger length: 130mm (can wrap around handles)")
        rospy.loginfo("  - Finger width: 30mm (strong contact area)")
        rospy.loginfo("  - Maximum opening: ~180mm (very wide for approach)")
        rospy.loginfo("  - Minimum closing: ~0mm (nearly touching)")
        rospy.loginfo("  - Grasp range: 10-180mm (versatile)")
        rospy.loginfo("  - High friction: μ=3.0 (firm grip)")
        rospy.loginfo("  - Max effort: 2000N per finger")

def main():
    try:
        tester = SimpleGripperTest()
        tester.run_test_sequence()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
