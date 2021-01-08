#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomToTwist:
    def __init__(self):
        rospy.init_node("odom_to_twist")

        self.hz = rospy.get_param("~hz", 5)
        self.twist_array = []

        rospy.Subscriber("odom_combined", Odometry,
                callback=self.odom_callback, queue_size=2)
        self.pub = rospy.Publisher("twist_from_odom", Twist,
                queue_size=2)
        rospy.Timer(rospy.Duration(1.0/self.hz), self.main_callback)

    def odom_callback(self, msg):
        twist = msg.twist.twist
        self.twist_array.append(
                [twist.linear.x, twist.linear.y, twist.linear.z,
                 twist.angular.x, twist.angular.y, twist.angular.z])

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
    odomToTwist = OdomToTwist()
    rospy.spin()

if __name__ == '__main__':
    main()
