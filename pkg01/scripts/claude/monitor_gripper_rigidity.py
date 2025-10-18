#!/usr/bin/env python
"""
Real-time gripper rigidity monitor.
Detects bending, deflection, and position drift during grasping.
"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
import math

class RigidityMonitor:
    def __init__(self):
        rospy.init_node('gripper_rigidity_monitor', anonymous=True)
        
        # State tracking
        self.last_positions = {'left': None, 'right': None}
        self.closed_position = None
        self.max_deflection = 0.0
        self.deflection_count = 0
        
        # Thresholds
        self.CLOSED_THRESHOLD = -0.010  # Considered "closed" below this
        self.DEFLECTION_WARNING = 0.0005  # Warn if opens by 0.5mm
        self.DEFLECTION_CRITICAL = 0.002  # Critical if opens by 2mm
        
        # Publishers
        self.marker_pub = rospy.Publisher('/gripper_rigidity_marker', Marker, queue_size=10)
        
        # Subscriber
        rospy.Subscriber('/ur10e_robot/joint_states', JointState, self.joint_callback)
        
        rospy.loginfo("==============================================")
        rospy.loginfo("  GRIPPER RIGIDITY MONITOR - ACTIVE")
        rospy.loginfo("==============================================")
        rospy.loginfo("Monitoring for finger bending and deflection...")
        rospy.loginfo("")
        rospy.loginfo("Thresholds:")
        rospy.loginfo(f"  Closed position: < {self.CLOSED_THRESHOLD}m")
        rospy.loginfo(f"  Warning deflection: > {self.DEFLECTION_WARNING*1000:.1f}mm")
        rospy.loginfo(f"  Critical deflection: > {self.DEFLECTION_CRITICAL*1000:.1f}mm")
        rospy.loginfo("")
        
    def joint_callback(self, msg):
        try:
            left_idx = msg.name.index('left_finger_joint')
            right_idx = msg.name.index('right_finger_joint')
            
            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]
            left_effort = msg.effort[left_idx] if msg.effort else 0.0
            right_effort = msg.effort[right_idx] if msg.effort else 0.0
            
            # Check if gripper is closed
            is_closed = left_pos < self.CLOSED_THRESHOLD and right_pos < self.CLOSED_THRESHOLD
            
            if is_closed:
                # Record closed position as baseline
                if self.closed_position is None:
                    self.closed_position = left_pos
                    rospy.loginfo(f"✓ Gripper CLOSED at {left_pos:.5f}m (baseline set)")
                    rospy.loginfo("  Monitoring for deflection during lift...")
                
                # Calculate deflection (opening from baseline)
                deflection_left = left_pos - self.closed_position
                deflection_right = right_pos - self.closed_position
                max_deflection_now = max(deflection_left, deflection_right)
                
                # Track maximum deflection seen
                if max_deflection_now > self.max_deflection:
                    self.max_deflection = max_deflection_now
                
                # Check for deflection/bending
                if max_deflection_now > self.DEFLECTION_CRITICAL:
                    rospy.logerr(f"🔴 CRITICAL DEFLECTION: {max_deflection_now*1000:.2f}mm - FINGERS BENDING!")
                    self.publish_marker("CRITICAL", ColorRGBA(1.0, 0.0, 0.0, 1.0))
                    self.deflection_count += 1
                    
                elif max_deflection_now > self.DEFLECTION_WARNING:
                    rospy.logwarn(f"⚠️  WARNING: Deflection {max_deflection_now*1000:.2f}mm - Slight bending detected")
                    self.publish_marker("WARNING", ColorRGBA(1.0, 0.5, 0.0, 1.0))
                    
                else:
                    # All good - rigid grip
                    if self.deflection_count == 0:  # Only log periodically if stable
                        rospy.loginfo_throttle(2.0, 
                            f"✓ RIGID: deflection={max_deflection_now*1000:.3f}mm, "
                            f"effort=L:{abs(left_effort):.1f}N R:{abs(right_effort):.1f}N")
                        self.publish_marker("RIGID", ColorRGBA(0.0, 1.0, 0.0, 1.0))
                
                # Check for asymmetry (uneven bending)
                asymmetry = abs(deflection_left - deflection_right)
                if asymmetry > 0.001:  # 1mm difference
                    rospy.logwarn(f"⚠️  ASYMMETRIC: L={deflection_left*1000:.2f}mm R={deflection_right*1000:.2f}mm (diff={asymmetry*1000:.2f}mm)")
                
            else:
                # Gripper open or opening
                if self.closed_position is not None:
                    # Was closed, now opening
                    rospy.logwarn(f"⚠️  Gripper OPENING: L={left_pos:.5f}m R={right_pos:.5f}m")
                    if self.max_deflection > 0:
                        rospy.loginfo(f"📊 Session stats: Max deflection={self.max_deflection*1000:.3f}mm, Warnings={self.deflection_count}")
                    self.closed_position = None
                    self.max_deflection = 0.0
                    self.deflection_count = 0
                    
        except (ValueError, IndexError) as e:
            pass  # Joint not found, ignore
    
    def publish_marker(self, text, color):
        """Publish visual marker in RViz showing rigidity status"""
        marker = Marker()
        marker.header.frame_id = "gripper_base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rigidity_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.03
        marker.color = color
        
        marker.text = text
        marker.lifetime = rospy.Duration(0.5)
        
        self.marker_pub.publish(marker)
    
    def run(self):
        rospy.spin()

def main():
    try:
        monitor = RigidityMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("")
        rospy.loginfo("==============================================")
        rospy.loginfo("  Rigidity monitor stopped")
        rospy.loginfo("==============================================")

if __name__ == '__main__':
    main()
