#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32
from assignment_python.msg import scan_range
from numba import njit


@njit(nogil=True, fastmath=True, cache=True)
def filter_laser(distances):
    # removes NaN or Inf data from scan
    for d in distances:
        if np.isnan(d) or np.isinf(d):
            return False
    return True


def get_laser(data):
    # finds minima and maxima and publishes range
    distances = np.array(data.ranges)
    check = False
    check = filter_laser(distances)
    if check:
        min_distance, max_distance = np.min(distances), np.max(distances)
        ranger = scan_range()
        ranger.closest, ranger.farthest = min_distance, max_distance
        ranger.header = data.header
        range_pub.publish(ranger)
        min_pub.publish(min_distance)
        max_pub.publish(max_distance)


if __name__ == "__main__":
    rospy.init_node("assignment1")
    min_pub = rospy.Publisher("/closest_point", Float32, queue_size=1)
    max_pub = rospy.Publisher("/farthest_point", Float32, queue_size=1)
    range_pub = rospy.Publisher("/scan_range", scan_range, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, get_laser)
    rospy.spin()
