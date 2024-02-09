#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math
import csv
from os.path import expanduser
from tf import TransformListener
from scipy.spatial import cKDTree as KDTree
from sensor_msgs.msg import  LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
lp = lg.LaserProjection()

class Waypoint(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        home = expanduser('~')
        f = home +'/f1ten-scripts/sides.csv'
        with open (f , 'r') as p:
            point_data = list(csv.reader(p))
            p.close()

        self.traj_ = []
        self.occupancy = []
        for i in range(len(point_data)):
            self.traj_.append((float(point_data[i][0]), float(point_data[i][1])))
        self.kdtree = KDTree(self.traj_)
        self.odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.pose_callback)
        self.publisher = rospy.Publisher("/left", Path, queue_size=1, latch=True)
        self.rightpublisher = rospy.Publisher("/right", Path, queue_size=1, latch=True)
        #self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)
        self.tf_list = TransformListener()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.slowbyskip = 1 #skips 10 odom data points to reduce frequency
        self.targetf = "/car1_30m/laser"  # base_link
        self.sourcef = "/map" # global map 
        self.right =[]
        self.left = []
        self.mine = []
        self.lastx = -111.0
        self.lasty = -111.0
    
    def csv_writer(self, data, file):
        for i in data:
            file.write('%f, %f\n' % (i[0], i[1]))
        file.close()
        print('Goodbye')
    
    def save_waypoints(self):
        home = expanduser('~')
        left_file = open(home +'/f1ten-scripts/csv/leftwp.csv', 'w')
        right_file = open(home +'/f1ten-scripts/csv/rightwp.csv', 'w')
        refs_file = open(home +'/f1ten-scripts/csv/refwp.csv', 'w')
        self.csv_writer(self.mine, refs_file)
        self.csv_writer(self.left, left_file)
        self.csv_writer(self.right, right_file)
        
    
    def publish_plan(self, left=True):
        msg = Path()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        if left: points = self.left
        if not left: points= self.right
        for i in points:
            pose = PoseStamped()
            pose.pose.position.x = float(i[0]) 
            pose.pose.position.y = float(i[1]) 
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1
            msg.poses.append(pose)
        print("Publishing points")
        if left:
            self.publisher.publish(msg)
        else:
            self.rightpublisher.publish(msg)

    def transformer(self, points):
        #self.tf_list.waitForTransform (targetf, sourcef, rospy.Time(0), rospy.Duration(1))
        for i in range(len(points)):
            p=PoseStamped()
            p.header.frame_id = self.sourcef
            p.header.stamp = rospy.Time(0)
            p.pose.position.x, p.pose.position.y = points[i]
            p.pose.orientation.w = 1.0
            pose_tf = self.tf_list.transformPose(self.targetf , p)
            xm, ym = pose_tf.pose.position.x, pose_tf.pose.position.y
            if ym > 0 and 0 < xm < 0.2:
                angle_ = 180*math.atan2(ym, xm)/math.pi
                if angle_ > 0:
                    angle = int(3*angle_) + 534
                    if 0 < angle < len(self.occupancy)-10:
                        xl, yl = self.occupancy[angle]
                        dist_laser = min(0.93*math.hypot(xl, yl), 1.5)
                        #print(self.occupancy[angle], xm, ym, angle, 180*math.atan2(ym, xm)/math.pi)
                        dist_my = math.hypot(xm, ym)
                        if dist_my < dist_laser:
                            if points[i] not in self.left and points[i] not in self.right:
                                print("appending left")
                                self.left.append(points[i])
            if ym < 0 and 0 < xm < 0.2 :
                angle_ = 180*math.atan2(ym, xm)/math.pi
                if angle_ < 0:
                    angle = int(3*abs(angle_)) - 6
                    if 0 < angle < len(self.occupancy)-10:
                        xl, yl = self.occupancy[angle]
                        dist_laser = min(0.93*math.hypot(xl, yl), 1.5)
                        dist_my = math.hypot(xm, ym)
                        if dist_my < dist_laser:
                            if points[i] not in self.left and points[i] not in self.right:
                                print("appending right")
                                self.right.append(points[i])

    def pose_callback(self, pose_msg):
        
        x, y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
        if (abs(x - self.lastx) > 0.1) or (abs(y - self.lasty) > 0.1):     
            self.lastx, self.lasty = x, y
            closeIndex = sorted(self.kdtree.query_ball_point((x, y), 1.5)) #1.4
            closepoints = []
            if len(closeIndex) > 1:
                if (x,y) not in self.mine:
                    self.mine.append((x,y))
                for i in closeIndex:
                    if self.traj_[i] not in self.left:
                        closepoints.append(self.traj_[i])
                self.transformer(closepoints)
                self.publish_plan()
                self.publish_plan(False)

    def scan_callback(self, msg):
        pc2_msg = lp.projectLaser(msg)
        point_generator = pc2.read_points(pc2_msg)
        i = 0
        self.occupancy = []#[(0,0)]*599
        # we can access a generator in a loop
        for point in point_generator:
            self.occupancy.append((point[0], point[1]))


def main():
    rospy.init_node('waypoint_logger_node')
    pp = Waypoint()
    rospy.spin()
    pp.save_waypoints()
if __name__ == '__main__':
    main()