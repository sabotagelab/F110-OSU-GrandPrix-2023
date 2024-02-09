#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from scipy.ndimage.filters import uniform_filter1d

#PID CONTROL PARAMS
kp = 0.25
kd = -1.25 
prev_error = 0.0 
error = 0.0

class reactive_follow_gap:
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def preprocess_lidar(self, ranges):
        minpoint = np.min(ranges)
        proc_ranges = uniform_filter1d(ranges, size=5)
        proc_ranges[np.less(proc_ranges,minpoint)] = 0
        proc_ranges[np.greater(proc_ranges,3)] = 4
        return proc_ranges

    def find_max_gap(self, free_ranges, ranges):
        diffrences = np.abs(np.diff(free_ranges))
        index = -1
        global_score = 0
        gap = []
        for i in range(len(diffrences)):
            if diffrences[i] <= 0.13:
                gap.append(ranges[i])
            if diffrences[i] > 0.13 or i == (len(diffrences)-1):
                if len(gap) > 5:
                    gap_score, maxind = self.find_best_point(gap)
                    if gap_score > global_score:
                        global_score = gap_score
                        if maxind > 0: index = i - (len(gap) - maxind)
                    gap *= 0
                else:
                    gap *= 0

        return index
    
    def find_best_point(self, gap_):
        angular_gap = 5 #degrees
        car_length = 0.5
        gap_score = 0
        gap = np.array(gap_)
        maxind = -1
        if len(gap) > (3*angular_gap) :
            gap_score += 100
            if max(gap) > 2*car_length:
                gap_score += np.max(gap)*10
            maxind = int((np.argmin(np.abs(gap - np.mean(gap))) + np.argmax(gap))/2)
        return gap_score, maxind

    def drive_calculator(self, index, ranges):
        angle_ = math.radians(index /3) - math.pi/2
        angle = abs(math.degrees(angle_))
        #velocity = 1.0
        if angle > 10: velocity = 0.4
        if angle > 5: velocity = 0.9
        if angle <= 5: velocity = 1.2
        return angle_, velocity
    
    def pid_control(self, error, velocity):
        global prev_error
        global kp
        global kd 
        angle = kp*error + kd*(error - prev_error)
        if abs(angle) < math.pi/90 : angle = 0.0
        prev_error = error
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = np.array(data.ranges[270:810])
        proc_ranges = self.preprocess_lidar(ranges)
        index = self.find_max_gap(proc_ranges, ranges)
        angle,velocity = self.drive_calculator(index, ranges)
        self.pid_control(angle, velocity)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)