#!/usr/bin/env python3
import tkinter as tk
import rospy
from std_msgs.msg import String

from numerical_keypad import NumericalKeypad

class pickupSite():
    def __init__(self):
        self.root = tk.Tk()
        self.keypad = NumericalKeypad(self.root, callback=self._rospy_talker)
        self._init_rospy()

    def _init_rospy(self):
        rospy.init_node('pickup_site', anonymous=True)
        self.pub = rospy.Publisher('cow_num', String, queue_size=10)
        self.rate = rospy.Rate(1) # 1hz

    def _rospy_talker(self, value):
        cow_num = str(value)
        rospy.loginfo(cow_num)
        self.pub.publish(cow_num)

    def keypad_loop(self):
        while not rospy.is_shutdown():
            self.root.mainloop()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        pickup_site = pickupSite()
        pickup_site.keypad_loop()
    except rospy.ROSInterruptException:
        pass