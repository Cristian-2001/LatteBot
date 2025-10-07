#!/usr/bin/env python
"""
Test script to verify gripper lift capability with enhanced grip strength.
Monitors joint states and provides real-time grip force feedback.
"""

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class GripperLiftTest:
    def __init__(self):
        rospy.init_node('gripper_lift_test', anonymous=True)
        
        # Action client for gripper control
        self.gripper_client = actionlib.SimpleActionClient(
            '/ur10e_robot/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server(timeout=rospy.Duration(10.0))
        rospy.loginfo("Gripper controller connected!")
        
        # Subscribe to joint states
        self.current_state = None
        rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_state_callback)
        
        rospy.sleep(1.0)  # Wait for first joint state
        
    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_state = msg
        
    def get_gripper_state(self):
        """Get current gripper finger positions and efforts"""
        if not self.current_state:
            return None, None, None, None
            
        try:
            left_idx = self.current_state.name.index('left_finger_joint')
            right_idx = self.current_state.name.index('right_finger_joint')
            
            left_pos = self.current_state.position[left_idx]
            right_pos = self.current_state.position[right_idx]
            left_effort = self.current_state.effort[left_idx] if self.current_state.effort else 0.0
            right_effort = self.current_state.effort[right_idx] if self.current_state.effort else 0.0
            
            return left_pos, right_pos, left_effort, right_effort
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Could not read gripper state: {e}")
            return None, None, None, None
    
    def close_gripper(self, position=-0.012, duration=3.0):
        """
        Close gripper to specified position.
        
        Args:
            position: Target position (negative = closed, 0 = neutral, positive = open)
                     Default -0.012 is near full closure for grasping
            duration: Time to reach position (seconds)
        """
        rospy.loginfo(f"Closing gripper to position {position}...")
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(timeout=rospy.Duration(duration + 2.0))
        
        result = self.gripper_client.get_result()
        state = self.gripper_client.get_state()
        
        rospy.sleep(0.5)  # Let it settle
        
        # Report final state
        left_pos, right_pos, left_effort, right_effort = self.get_gripper_state()
        if left_pos is not None:
            rospy.loginfo("=" * 60)
            rospy.loginfo("GRIPPER STATE AFTER CLOSING:")
            rospy.loginfo(f"  Left finger:  pos={left_pos:+.5f}m  effort={left_effort:+.2f}N")
            rospy.loginfo(f"  Right finger: pos={right_pos:+.5f}m  effort={right_effort:+.2f}N")
            rospy.loginfo(f"  Total grip force: ~{abs(left_effort + right_effort):.2f}N")
            rospy.loginfo(f"  Estimated friction force: ~{abs(left_effort + right_effort) * 3.0:.2f}N")
            rospy.loginfo("=" * 60)
            
            # Check if fully closed
            if left_pos < -0.010 and right_pos < -0.010:
                rospy.loginfo("✓ Gripper is CLOSED (good for grasping)")
            elif left_pos < -0.005:
                rospy.loginfo("○ Gripper is PARTIALLY CLOSED")
            else:
                rospy.logwarn("✗ Gripper is OPEN (not grasping)")
                
            # Check effort
            total_effort = abs(left_effort + right_effort)
            if total_effort > 100:
                rospy.loginfo("✓ HIGH grip force detected (excellent for lifting)")
            elif total_effort > 50:
                rospy.loginfo("○ MODERATE grip force (should hold light objects)")
            elif total_effort > 0:
                rospy.loginfo("○ LOW grip force (may slip under load)")
            else:
                rospy.loginfo("- No effort data available (check controller)")
        
        return state == actionlib.GoalStatus.SUCCEEDED
    
    def open_gripper(self, position=0.04, duration=2.0):
        """
        Open gripper to specified position.
        
        Args:
            position: Target position (positive = open)
            duration: Time to reach position (seconds)
        """
        rospy.loginfo(f"Opening gripper to position {position}...")
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(timeout=rospy.Duration(duration + 2.0))
        
        result = self.gripper_client.get_result()
        state = self.gripper_client.get_state()
        
        rospy.loginfo("Gripper opened")
        return state == actionlib.GoalStatus.SUCCEEDED
    
    def monitor_grip(self, duration=10.0):
        """
        Monitor gripper state over time to check grip stability during lifting.
        
        Args:
            duration: How long to monitor (seconds)
        """
        rospy.loginfo(f"Monitoring gripper for {duration} seconds...")
        rospy.loginfo("(Try lifting the bucket now with MoveIt)")
        rospy.loginfo("")
        
        rate = rospy.Rate(5)  # 5 Hz monitoring
        start_time = rospy.Time.now()
        
        last_left_pos = None
        slip_detected = False
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            left_pos, right_pos, left_effort, right_effort = self.get_gripper_state()
            
            if left_pos is not None:
                # Check for slip (fingers opening unexpectedly)
                if last_left_pos is not None:
                    pos_change = left_pos - last_left_pos
                    if pos_change > 0.002:  # Opening by more than 2mm
                        if not slip_detected:
                            rospy.logwarn(f"⚠ SLIP DETECTED! Fingers opened by {pos_change*1000:.1f}mm")
                            slip_detected = True
                
                last_left_pos = left_pos
                
                total_effort = abs(left_effort + right_effort)
                rospy.loginfo(
                    f"[{(rospy.Time.now() - start_time).to_sec():5.1f}s] "
                    f"Pos: L={left_pos:+.4f} R={right_pos:+.4f}m | "
                    f"Effort: {total_effort:6.2f}N"
                )
            
            rate.sleep()
        
        if not slip_detected:
            rospy.loginfo("✓ No slip detected during monitoring period!")
        
    def run_full_test(self):
        """Run complete gripper test sequence"""
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("GRIPPER LIFT TEST - Enhanced Grip Strength")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        
        # Step 1: Open gripper
        rospy.loginfo("STEP 1: Opening gripper...")
        self.open_gripper(position=0.04, duration=2.0)
        rospy.sleep(1.0)
        
        # Step 2: Prompt user to position
        rospy.loginfo("")
        rospy.loginfo("STEP 2: Position gripper around bucket handle using MoveIt")
        rospy.loginfo("        Press ENTER when ready to close gripper...")
        raw_input()
        
        # Step 3: Close gripper with strong force
        rospy.loginfo("")
        rospy.loginfo("STEP 3: Closing gripper with 500N effort limit...")
        success = self.close_gripper(position=-0.012, duration=3.0)
        
        if not success:
            rospy.logwarn("Gripper closing may have failed. Check controller status.")
            return
        
        rospy.sleep(1.0)
        
        # Step 4: Monitor during lift
        rospy.loginfo("")
        rospy.loginfo("STEP 4: Monitor grip stability during lifting")
        rospy.loginfo("        Use MoveIt to plan and execute upward motion now!")
        rospy.loginfo("        Monitoring will start in 3 seconds...")
        rospy.sleep(3.0)
        
        self.monitor_grip(duration=15.0)
        
        # Summary
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST COMPLETE")
        rospy.loginfo("=" * 60)
        rospy.loginfo("")
        rospy.loginfo("Expected results with enhanced physics:")
        rospy.loginfo("  ✓ Grip force > 100N (up to 500N)")
        rospy.loginfo("  ✓ Friction force > 300N (3x grip force)")
        rospy.loginfo("  ✓ No slip during lift (2kg bucket = 19.6N)")
        rospy.loginfo("  ✓ Stable contact (no jitter or bouncing)")
        rospy.loginfo("")
        rospy.loginfo("If bucket still slips, see GRIPPER_LIFT_FIX.md for troubleshooting")
        rospy.loginfo("")

def main():
    try:
        tester = GripperLiftTest()
        tester.run_full_test()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")

if __name__ == '__main__':
    main()
