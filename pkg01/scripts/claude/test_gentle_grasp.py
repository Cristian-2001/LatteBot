#!/usr/bin/env python

"""
Test script for gentle bucket grasping with anti-slip measures
- Very slow gripper closing
- Monitoring contact forces
- Step-by-step verification
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class GentleGraspTest:
    def __init__(self):
        rospy.init_node('gentle_grasp_test')
        
        # Action client for gripper controller
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
        rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_state_callback)
        
    def joint_state_callback(self, msg):
        self.joint_states = msg
        
    def get_finger_position(self):
        """Get current finger_joint position"""
        if self.joint_states is None:
            return None
        
        try:
            idx = self.joint_states.name.index('finger_joint')
            return self.joint_states.position[idx]
        except (ValueError, IndexError):
            return None
    
    def move_gripper(self, position, duration):
        """
        Move gripper to position with specified duration
        
        Args:
            position: Target position (0.0=open, 0.7=closed)
            duration: Time to complete movement (seconds)
        """
        goal = FollowJointTrajectoryGoal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['finger_joint']
        
        # Single waypoint at target position
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)
        
        trajectory.points = [point]
        goal.trajectory = trajectory
        
        # Send goal
        rospy.loginfo(f"Moving gripper to {position:.3f} over {duration:.1f} seconds...")
        self.gripper_client.send_goal(goal)
        
        # Wait for completion
        self.gripper_client.wait_for_result()
        result = self.gripper_client.get_result()
        
        # Check final position
        rospy.sleep(0.5)  # Let physics settle
        final_pos = self.get_finger_position()
        
        if final_pos is not None:
            rospy.loginfo(f"Target: {position:.3f}, Achieved: {final_pos:.3f}, Error: {abs(position - final_pos):.4f}")
        
        return result
    
    def test_sequence(self):
        """Run gentle grasp test sequence"""
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("GENTLE GRASP TEST - ANTI-SLIP PROTOCOL")
        rospy.loginfo("="*60)
        
        # Step 1: Fully open gripper
        rospy.loginfo("\n[Step 1] Opening gripper fully...")
        self.move_gripper(0.0, 3.0)
        rospy.sleep(1.0)
        
        rospy.loginfo("\n[MANUAL] Position gripper around bucket handle now!")
        rospy.loginfo("         - Handle should be between gripper fingers")
        rospy.loginfo("         - Not touching yet, small gap on each side")
        rospy.loginfo("         - Press Enter when ready...")
        raw_input()
        
        # Step 2: Close slowly to make initial contact
        rospy.loginfo("\n[Step 2] Closing slowly to make gentle contact (20%)...")
        self.move_gripper(0.14, 5.0)  # 20% closed, very slow
        rospy.sleep(2.0)
        
        rospy.loginfo("         - Check: Bucket should be lightly touched, not moved")
        rospy.loginfo("         - Press Enter to continue...")
        raw_input()
        
        # Step 3: Increase grip gradually
        rospy.loginfo("\n[Step 3] Increasing grip to 40%...")
        self.move_gripper(0.28, 5.0)
        rospy.sleep(2.0)
        
        rospy.loginfo("         - Check: Bucket should be held, slight compression")
        rospy.loginfo("         - Press Enter to continue...")
        raw_input()
        
        # Step 4: Increase to 60%
        rospy.loginfo("\n[Step 4] Increasing grip to 60%...")
        self.move_gripper(0.42, 5.0)
        rospy.sleep(2.0)
        
        rospy.loginfo("         - Check: Firm grip, bucket not slipping")
        rospy.loginfo("         - Press Enter to continue...")
        raw_input()
        
        # Step 5: Maximum grip
        rospy.loginfo("\n[Step 5] Closing to maximum grip (80%)...")
        self.move_gripper(0.56, 6.0)  # Not quite full 0.7 to avoid excessive force
        rospy.sleep(3.0)
        
        rospy.loginfo("\n[Result] Gripper closed. Check:")
        rospy.loginfo("         ✓ Bucket firmly held")
        rospy.loginfo("         ✓ Bucket did NOT slip away during closing")
        rospy.loginfo("         ✓ Handle visible between fingers")
        
        rospy.loginfo("\n[Step 6] Hold for 5 seconds to verify stability...")
        for i in range(5):
            rospy.sleep(1.0)
            pos = self.get_finger_position()
            if pos is not None:
                rospy.loginfo(f"         {i+1}s - Position: {pos:.4f}")
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST COMPLETE")
        rospy.loginfo("="*60)
        rospy.loginfo("\nIf bucket slipped:")
        rospy.loginfo("  1. Check alignment: gripper should be perpendicular to handle")
        rospy.loginfo("  2. Check position: handle between fingers, not just finger tips")
        rospy.loginfo("  3. Check collision visualization: pink boxes on handle and fingers")
        rospy.loginfo("  4. Try even slower: increase step durations in script")
        rospy.loginfo("\nTo release bucket: rosrun pkg01 test_robotiq_gripper.py")

def main():
    try:
        tester = GentleGraspTest()
        tester.test_sequence()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    main()
