#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32
from assignment_python.msg import scan_range
from numba import njit
from numba import float32 as f32

@njit(nogil=True, fastmath=True, cache=True)
def filterLaser(distances):
#removes NaN or Inf data from scan
	for d in distances:
		if np.isnan(d) or np.isinf(d):
			return False
	return True

def Getlaser(data):
#finds minima and maxima and publishes range
	distances = np.array(data.ranges)
	check = False
	check = filterLaser(distances)
	if check :
		minD, maxD = np.min(distances), np.max(distances)
		ranger = scan_range()
		ranger.closest, ranger.farthest = minD, maxD
		ranger.header = data.header
		rangepub.publish(ranger)
		minpub.publish(minD)
		maxpub.publish(maxD)


if __name__ == '__main__':
	rospy.init_node('assignment1')
	minpub = rospy.Publisher( '/closest_point', Float32, queue_size=1)
	maxpub = rospy.Publisher( '/farthest_point', Float32, queue_size=1)
	rangepub = rospy.Publisher('/scan_range', scan_range, queue_size=1)
	rospy.Subscriber('/scan', LaserScan, Getlaser)
	rospy.spin()
