#!/usr/bin/env python3
import rospy
import tf2_ros
from tf2_ros import TransformException

# Initialize the ROS node FIRST
rospy.init_node('get_orientation_node', anonymous=True)

# Now you can create the tf buffer and listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Give it a moment to fill the buffer
rospy.sleep(1.0)

try:
    # Look up the transform
    transform = tf_buffer.lookup_transform('flange', 'base_link', rospy.Time(0), rospy.Duration(1.0))

    # Get the position
    position = transform.transform.translation
    print(f"Position: x={position.x}, y={position.y}, z={position.z}")
    
    # Get the orientation as a quaternion
    orientation = transform.transform.rotation
    print(f"Orientation (quaternion): x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
    
    # Convert to Euler angles if needed
    from tf.transformations import euler_from_quaternion
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
    
except TransformException as ex:
    rospy.logerr(f"Could not get transform: {ex}")