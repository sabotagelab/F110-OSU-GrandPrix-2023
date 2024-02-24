#!/usr/bin/env python
import rospy
import numpy as np
from numba import njit, float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

driver = AckermannDriveStamped()
collision_bool = Bool()
driver.drive.speed = 0.0


@njit(nogil=True, fastmath=True, cache=True)
def cosine_computer():
    angles = np.arange(-3.14159274101, 3.14159274101, 0.00582315586507)
    cosines = np.cos(angles)
    return cosines


@njit(float64(float64, float64[:], float64[:]), nogil=True, fastmath=True, cache=True)
def ttc_computer(speed, distances, cosines):
    speed_arr = speed * cosines
    TTC = distances / speed_arr
    ret = np.min(np.abs(TTC))
    return ret


def scan_callback(data):
    head = data.header
    distances = np.array(data.ranges)
    if abs(speed) > 0.1:
        close_ttc = ttc_computer(speed, distances, cosines)
        if close_ttc < 0.35:
            apply_break(head)
            print(
                "\n\n\n ##############################  applying break ########################\n\n\n"
            )
        else:
            collision_bool.data = False
            bool_pub.publish(collision_bool)


def odom_callback(data):
    global speed
    speed = data.twist.twist.linear.x


def apply_break(head):
    driver.header = head
    collision_bool.data = True
    brake_pub.publish(driver)
    bool_pub.publish(collision_bool)


speed = 0.0
if __name__ == "__main__":
    rospy.init_node("emergency_brake_node")
    cosines = cosine_computer()
    brake_pub = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=1)
    bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()
