#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from move_platform import move_platform

def platform_home():
    """
    Move the platform to the home position (0.0).
    """
    move_platform(0.0)

def get_cow_position(cow_num: String):
    """
    Map cow number to platform position.
    """
    try:
        num = int(cow_num.data)
        if 0 <= num <= 10:
            position = float(num)
        else:
            rospy.logwarn("Cow number out of range (0-10). Defaulting to position 0.0")
            position = 0.0
    except ValueError:
        rospy.logwarn("Invalid cow number received. Defaulting to position 0.0")
        position = 0.0
    finally:
        move_platform(position)

def robot_listener():
    rospy.init_node('robot_listener', anonymous=True)
    rospy.Subscriber("cow_num", String, get_cow_position)
    print("Robot listener node started, waiting for cow numbers...")
    rospy.spin()

if __name__ == '__main__':
    robot_listener()