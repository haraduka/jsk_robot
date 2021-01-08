#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist

class OrganizeBaseCommand:
    def __init__(self):
        rospy.init_node("organize_base_command")

        self.hz = rospy.get_param("~hz", 5)
        self.twist_array = []

        rospy.Subscriber("base_controller/command_unorganized", Twist,
                callback=self.command_callback, queue_size=2)
        self.pub = rospy.Publisher("base_controller/command", Twist,
                queue_size=2)
        rospy.Timer(rospy.Duration(1.0/self.hz), self.main_callback)

    def command_callback(self, msg):
        self.twist_array.append(
                [msg.linear.x, msg.linear.y, msg.linear.z,
                 msg.angular.x, msg.angular.y, msg.angular.z])

    def main_callback(self, event):
        pub_msg = Twist()
        if len(self.twist_array) > 0:
            arr = np.asarray(self.twist_array, dtype=np.float32)
            twist = np.average(arr, axis=0)
        else:
            twist = np.zeros(6, dtype=np.float32)
        pub_msg.linear.x = twist[0]
        pub_msg.linear.y = twist[1]
        pub_msg.linear.z = twist[2]
        pub_msg.angular.x = twist[3]
        pub_msg.angular.y = twist[4]
        pub_msg.angular.z = twist[5]
        self.pub.publish(pub_msg)
        self.twist_array = []



def main():
    organizeBaseCommand = OrganizeBaseCommand()
    rospy.spin()

if __name__ == '__main__':
    main()
