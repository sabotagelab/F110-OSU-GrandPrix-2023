#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class Safety(object):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        self.cosines = np.cos(
            np.arange(-3.14159274101, 3.14159274101, 0.00582315586507)
        )
        self.brake_pub = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=1)
        self.bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size=1)
        self.driver = AckermannDriveStamped()
        self.collision_bool = Bool()
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.speed = 0

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, data):
        distances = np.array(data.ranges)
        if abs(self.speed) > 0.1:
            close_ttc = np.min(np.abs(distances / (self.speed * self.cosines)))
            if close_ttc < 0.28:
                self.driver.header = data.header
                self.collision_bool.data = True
                self.brake_pub.publish(self.driver)
                self.bool_pub.publish(self.collision_bool)
            else:
                self.collision_bool.data = False
                self.bool_pub.publish(self.collision_bool)


def main():
    rospy.init_node("safety_node")
    sn = Safety()
    rospy.spin()


if __name__ == "__main__":
    main()
