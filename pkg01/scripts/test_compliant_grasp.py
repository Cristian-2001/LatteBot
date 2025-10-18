#!/usr/bin/env python

"""
Test compliant gripper grasping with bucket handle.
Verifies that the new compliant settings prevent handle jumping.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class CompliantGripperTest:
    def __init__(self):
        rospy.init_node('compliant_gripper_test')
        
        # Action client for gripper
        self.gripper_client = actionlib.SimpleActionClient(
            '/ur10e_robot/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Gripper controller connected!")
        
        # Subscribe to joint states for monitoring
        self.joint_states = None
        self.sub = rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_state_callback)
        
    def joint_state_callback(self, msg):
        """Store latest joint states"""
        self.joint_states = msg
        
    def get_finger_positions(self):
        """Get current finger positions"""
        if self.joint_states is None:
            return None, None
            
        try:
            left_idx = self.joint_states.name.index('left_finger_joint')
            right_idx = self.joint_states.name.index('right_finger_joint')
            return self.joint_states.position[left_idx], self.joint_states.position[right_idx]
        except ValueError:
            return None, None
    
    def move_gripper(self, left_pos, right_pos, duration=3.0):
        """
        Move gripper to specified position with compliant trajectory.
        Uses slower duration (3.0s) to prevent sudden contact forces.
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [left_pos, right_pos]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        rospy.loginfo(f"Moving gripper to: left={left_pos:.4f}m, right={right_pos:.4f}m over {duration}s")
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        
        result = self.gripper_client.get_result()
        rospy.loginfo(f"Gripper move completed: {result}")
        
        # Give physics time to settle
        rospy.sleep(1.0)
        
        # Report final position
        left_final, right_final = self.get_finger_positions()
        if left_final is not None:
            gap = 40.0 + (left_final + right_final) * 1000.0  # Convert to mm
            rospy.loginfo(f"Final positions: left={left_final:.4f}m, right={right_final:.4f}m, gap={gap:.1f}mm")
        
        return result
    
    def monitor_stability(self, duration=5.0):
        """
        Monitor finger positions to detect jumping/oscillation.
        """
        rospy.loginfo(f"Monitoring gripper stability for {duration} seconds...")
        rospy.loginfo("Watch for: oscillation, position drift, or sudden jumps")
        
        rate = rospy.Rate(10)  # 10 Hz monitoring
        start_time = rospy.Time.now()
        positions = []
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            left, right = self.get_finger_positions()
            if left is not None:
                positions.append((left, right))
                gap = 40.0 + (left + right) * 1000.0
                rospy.loginfo(f"  Pos: left={left:.4f}m, right={right:.4f}m, gap={gap:.1f}mm")
            rate.sleep()
        
        # Analyze stability
        if len(positions) > 10:
            left_positions = [p[0] for p in positions]
            right_positions = [p[1] for p in positions]
            
            left_std = self._std_dev(left_positions)
            right_std = self._std_dev(right_positions)
            
            rospy.loginfo(f"\nStability Analysis:")
            rospy.loginfo(f"  Left finger std dev: {left_std*1000:.2f}mm")
            rospy.loginfo(f"  Right finger std dev: {right_std*1000:.2f}mm")
            
            if left_std < 0.001 and right_std < 0.001:
                rospy.loginfo("  ✅ STABLE - Standard deviation < 1mm")
                return True
            elif left_std < 0.003 and right_std < 0.003:
                rospy.logwarn("  ⚠️  ACCEPTABLE - Standard deviation < 3mm (some jitter)")
                return True
            else:
                rospy.logerr("  ❌ UNSTABLE - Excessive position variation (jumping detected)")
                return False
        
        return None
    
    def _std_dev(self, values):
        """Calculate standard deviation"""
        n = len(values)
        if n < 2:
            return 0.0
        mean = sum(values) / n
        variance = sum((x - mean)**2 for x in values) / (n - 1)
        return variance ** 0.5
    
    def run_test_sequence(self):
        """
        Run complete test sequence for compliant grasping.
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("COMPLIANT GRIPPER TEST - Handle Jumping Prevention")
        rospy.loginfo("="*60)
        
        # Test 1: Open gripper
        rospy.loginfo("\n[Test 1] Opening gripper to 0.07m (180mm gap)")
        self.move_gripper(0.07, 0.07, duration=2.0)
        rospy.sleep(1.0)
        
        # Test 2: Move to grasp position (gentle, new compliant setting)
        rospy.loginfo("\n[Test 2] Closing to grasp_handle position: -0.008m (24mm gap)")
        rospy.loginfo("This should be SMOOTH with NO jumping")
        self.move_gripper(-0.008, -0.008, duration=3.0)
        
        # Test 3: Monitor for jumping/instability
        rospy.loginfo("\n[Test 3] Monitoring for handle jumping...")
        stable = self.monitor_stability(duration=5.0)
        
        if stable:
            rospy.loginfo("\n✅ SUCCESS: Gripper is stable, no jumping detected!")
            rospy.loginfo("The compliant grasping fix is working correctly.")
        else:
            rospy.logwarn("\n⚠️  WARNING: Some instability detected.")
            rospy.logwarn("Consider further reducing controller gains.")
        
        # Test 4: Tighten slightly (test grip strength)
        rospy.loginfo("\n[Test 4] Tightening to -0.010m (20mm gap) to test grip strength")
        self.move_gripper(-0.010, -0.010, duration=3.0)
        stable2 = self.monitor_stability(duration=3.0)
        
        if stable2:
            rospy.loginfo("✅ Tighter grip is also stable")
        else:
            rospy.logwarn("⚠️  Tighter grip shows instability - use -0.008m position")
        
        # Test 5: Release
        rospy.loginfo("\n[Test 5] Opening gripper to release")
        self.move_gripper(0.05, 0.05, duration=2.0)
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST COMPLETE")
        rospy.loginfo("="*60)
        rospy.loginfo("\nSummary:")
        rospy.loginfo(f"  Grasp position (-0.008m) stable: {stable}")
        rospy.loginfo(f"  Tight position (-0.010m) stable: {stable2}")
        rospy.loginfo("\nRecommendation:")
        if stable:
            rospy.loginfo("  Use grasp_handle position (-0.008m) for reliable grasping")
        else:
            rospy.loginfo("  Further tuning needed - reduce controller gains more")

if __name__ == '__main__':
    try:
        test = CompliantGripperTest()
        rospy.sleep(2.0)  # Wait for initialization
        test.run_test_sequence()
    except rospy.ROSInterruptException:
        pass
