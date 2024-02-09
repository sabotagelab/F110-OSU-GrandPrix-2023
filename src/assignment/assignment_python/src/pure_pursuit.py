#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math
from tf import TransformListener
from scipy.spatial import KDTree

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        with open (("/home/unmesh/rcws/logs/waypoints.csv") , 'r') as p:
            point_data = list(csv.reader(p))
            p.close()

        self.traj_ = []
        for i in range(len(point_data)):
            self.traj_.append((float(point_data[i][0]), float(point_data[i][1])))
        self.kdtree = KDTree(self.traj_, leafsize=1000)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/shapes', Marker, queue_size=10)
        self.tf_list = TransformListener()
        self.slowbyskip = 10 #skips 10 odom data points to reduce frequency
        self.targetf = "/base_link"  # base_link
        self.sourcef = "/map" # global map 
    
    def pointVisual(self, pointlist):
        points = Marker()
        points.header.stamp = rospy.Time(0)
        points.header.frame_id = self.sourcef
        points.ns = "markers"
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x=0.05
        points.scale.y=0.05
        points.color.b = 1.0
        points.color.a = 1.0
        pp = []
        for i in pointlist:
            p=Point()
            p.x, p.y = self.traj_[i]
            p.z = 0
            pp.append(p)
        points.points = pp
        self.marker_pub.publish(points)

    def transformer(self, points):
        #self.tf_list.waitForTransform (targetf, sourcef, rospy.Time(0), rospy.Duration(1))
        for i in range(len(points)):
            p=PoseStamped()
            p.header.frame_id = self.sourcef
            p.header.stamp = rospy.Time(0)
            p.pose.position.x, p.pose.position.y = points[i]
            p.pose.orientation.w = 1.0
            pose_tf = self.tf_list.transformPose(self.targetf , p)
            if pose_tf.pose.position.x > 0 :
                return pose_tf, i

    def Getvelocity(self,angle):
        angle = abs(math.degrees(angle))
        if angle <= 10: return 1.0
        if angle > 10: return 0.8
        return 0.5
        

    def driver(self, angle, velocity):
        if -0.6 < angle < 0.6:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0.45*angle
            drive_msg.drive.speed = velocity
            self.drive_pub.publish(drive_msg)
        else: self.driver(0, 0)
        #print(angle, velocity)

    def pose_callback(self, pose_msg):
        if self.slowbyskip != 0:
            self.slowbyskip -= 1
        else:
            x, y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
            closeIndex = sorted(self.kdtree.query_ball_point((x, y), 0.85))
            if len(closeIndex) > 1:
                closepoints = [self.traj_[closeIndex[0]], self.traj_[closeIndex[-1]]]
                pose_tf, index = self.transformer(closepoints)
                xg, yg = pose_tf.pose.position.x, pose_tf.pose.position.y
                L = (xg)**2 + (yg)**2
                angle = 2*yg / L
                self.driver(angle, self.Getvelocity(angle))
                if index < closeIndex[-1]: self.pointVisual(closeIndex[(int(len(closeIndex)/2)):])
                if index > closeIndex[0]: self.pointVisual(closeIndex[:(int(len(closeIndex)/2))])
                self.slowbyskip = 10
            else: self.driver(0, 0)

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()