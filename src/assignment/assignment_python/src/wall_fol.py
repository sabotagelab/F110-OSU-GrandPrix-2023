#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 3.2
kd = -1.5 #TODO
ki = 0 # TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.75
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def getRange(self, data):
        angle_ = 58
        index_b = 810
        index_a = index_b - (angle_ * 3)
        a = data.ranges[index_a]
        b = data.ranges[index_b]
        angle = math.radians(angle_)
        alpha = math.atan2((a*math.cos(angle) - b),(a*math.sin(angle)))
        Dt = b*math.cos(alpha) + 1.24*math.sin(alpha)
        #print(" DT  ", Dt, ' a ', a, ' b ', b, ' alp ', math.degrees(alpha), '\n')
        return Dt

    def pid_control(self, error):
        global integral
        global prev_error
        global kp
        global ki
        global kd 
        inp = kp*error + kd*(error - prev_error) + ki*integral
        integral += error
        prev_error = error
        angle = -math.atan2(inp, 6)
        #print('error ', error, inp, math.degrees(angle))
        velocity = self.Getvelocity(angle)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        #print(error, angle, '\n')
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, rightDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        global DESIRED_DISTANCE_LEFT
        error = (DESIRED_DISTANCE_LEFT - rightDist)
        #error = math.atan((DESIRED_DISTANCE_LEFT - leftDist)/3)
        return error

    def lidar_callback(self, data):
        #if self.filterLaser((np.array(data.ranges))):
        rightDist = self.getRange(data)
        error = self.followLeft(data, rightDist)
        self.pid_control(error)

    def filterLaser(self,distances):
        for d in distances:
            if np.isnan(d) or np.isinf(d):
                return False
        return True

    def Getvelocity(self,angle):
        angle = abs(math.degrees(angle))
        if angle > 20: return 0.5
        if angle > 10: return 0.6
        if angle <= 10: return 0.8

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)