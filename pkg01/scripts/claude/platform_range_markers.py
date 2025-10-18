#!/usr/bin/env python

"""
Publishes RViz markers to visualize the platform's range of motion.
Shows start position (0m) and end position (10m) markers.
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_range_markers():
    """Create markers showing the platform's 0-10m range"""
    marker_array = MarkerArray()
    
    # Start position marker (at 0m)
    start_marker = Marker()
    start_marker.header.frame_id = "world_platform"
    start_marker.header.stamp = rospy.Time.now()
    start_marker.ns = "platform_range"
    start_marker.id = 0
    start_marker.type = Marker.CUBE
    start_marker.action = Marker.ADD
    start_marker.pose.position.x = 0.0
    start_marker.pose.position.y = 0.0
    start_marker.pose.position.z = 0.5
    start_marker.pose.orientation.w = 1.0
    start_marker.scale.x = 0.2
    start_marker.scale.y = 1.0
    start_marker.scale.z = 1.0
    start_marker.color.r = 0.0
    start_marker.color.g = 1.0
    start_marker.color.b = 0.0
    start_marker.color.a = 0.5
    marker_array.markers.append(start_marker)
    
    # End position marker (at 10m)
    end_marker = Marker()
    end_marker.header.frame_id = "world_platform"
    end_marker.header.stamp = rospy.Time.now()
    end_marker.ns = "platform_range"
    end_marker.id = 1
    end_marker.type = Marker.CUBE
    end_marker.action = Marker.ADD
    end_marker.pose.position.x = 10.0
    end_marker.pose.position.y = 0.0
    end_marker.pose.position.z = 0.5
    end_marker.pose.orientation.w = 1.0
    end_marker.scale.x = 0.2
    end_marker.scale.y = 1.0
    end_marker.scale.z = 1.0
    end_marker.color.r = 1.0
    end_marker.color.g = 0.0
    end_marker.color.b = 0.0
    end_marker.color.a = 0.5
    marker_array.markers.append(end_marker)
    
    # Rail/track line
    rail_marker = Marker()
    rail_marker.header.frame_id = "world_platform"
    rail_marker.header.stamp = rospy.Time.now()
    rail_marker.ns = "platform_range"
    rail_marker.id = 2
    rail_marker.type = Marker.LINE_STRIP
    rail_marker.action = Marker.ADD
    rail_marker.scale.x = 0.05  # Line width
    rail_marker.color.r = 1.0
    rail_marker.color.g = 1.0
    rail_marker.color.b = 0.0
    rail_marker.color.a = 0.7
    
    # Create line from 0 to 10m
    p1 = Point()
    p1.x = 0.0
    p1.y = 0.0
    p1.z = 0.0
    rail_marker.points.append(p1)
    
    p2 = Point()
    p2.x = 10.0
    p2.y = 0.0
    p2.z = 0.0
    rail_marker.points.append(p2)
    
    marker_array.markers.append(rail_marker)
    
    # Text labels
    for i, (pos, text) in enumerate([(0.0, "START (0m)"), (10.0, "END (10m)")]):
        text_marker = Marker()
        text_marker.header.frame_id = "world_platform"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "platform_range"
        text_marker.id = 10 + i
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = pos
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.2
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = text
        marker_array.markers.append(text_marker)
    
    return marker_array

def main():
    rospy.init_node('platform_range_markers')
    
    pub = rospy.Publisher('/platform_range_markers', MarkerArray, queue_size=10, latch=True)
    
    rospy.loginfo("Publishing platform range markers...")
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        markers = create_range_markers()
        pub.publish(markers)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
