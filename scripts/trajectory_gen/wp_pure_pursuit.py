#!/usr/bin/env python
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from os.path import expanduser
from scipy.spatial import cKDTree as KDTree
from sensor_msgs.msg import LaserScan
from tf import TransformListener
from visualization_msgs.msg import Marker
import csv
import laser_geometry.laser_geometry as lg
import math
import rospy
import sensor_msgs.point_cloud2 as pc2


lp = lg.LaserProjection()


class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        home = expanduser('~')
        self.horizon = 0.8
        f = home +'/f1ten-scripts/ways.csv'
        with open (f , 'r') as p:
            point_data = list(csv.reader(p))
            p.close()
        data_size = len(point_data)
        self.trajectory_ = [(0.0, 0.0)]*data_size
        self.left_traj = [(0.0, 0.0)]*data_size
        self.right_traj = [(0.0, 0.0)]*data_size
        for i in range(data_size):
            self.trajectory_[i] = (float(point_data[i][0]), float(point_data[i][1]))
            self.left_traj[i] = (float(point_data[i][4]), float(point_data[i][5]))
            self.right_traj[i] = (float(point_data[i][2]), float(point_data[i][3]))
        self.kdtree = KDTree(self.trajectory_, leaf_size=10)
        self.leftkdtree = KDTree(self.left_traj, leaf_size=10)
        self.rightkdtree = KDTree(self.right_traj, leaf_size=10)
        self.odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.marker2_pub = rospy.Publisher('/shapes2', Marker, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.tf_list = TransformListener()
        self.targetf = "/car1_30m/laser"  # base_link
        self.sourcef = "map"
        self.col = False
        self.kp = 0.23
        self.kd = -1.25
        self.prev_error = 0.0
        self.safety_radius = 0.15
        self.future_col = False
        self.kdt_occ = None
        self.occupancy = [0]*599
        self.pc_padding = 0.05

    def point_viz(self, point_list):
        """Visualizes the points in rviz."""
        points = Marker()
        points.header.stamp = rospy.Time(0)
        points.header.frame_id = self.targetf
        points.ns = "markers2"
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x=0.05
        points.scale.y=0.05
        points.color.b = 1.0
        points.color.a = 0.5
        pp = []
        for i in point_list:
            p=Point()
            p.x, p.y = i
            p.z = 0
            pp.append(p)
        points.points = pp
        self.marker2_pub.publish(points)

    def publish_plan(self, traj, waypoint_publisher):
        msg = Path()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        for i in range(len(traj)):
            pose = PoseStamped()
            pose.pose.position.x = traj[i][0]
            pose.pose.position.y = traj[i][1]
            pose.pose.orientation.w = 1
            msg.poses.append(pose)

        waypoint_publisher.publish(msg) 

    def transformer(self, points):
        xlist = 0.0
        ind = 0.0
        ptf = PoseStamped()
        for i in range(len(points)):
            p=PoseStamped()
            p.header.frame_id = self.sourcef
            p.header.stamp = rospy.Time(0)
            p.pose.position.x, p.pose.position.y = points[i]
            p.pose.orientation.w = 1.0
            pose_tf = self.tf_list.transformPose(self.targetf, p)
            if pose_tf.pose.position.x > xlist:
                ind = i
                xlist = pose_tf.pose.position.x
                ptf = pose_tf
        return ptf, ind

    def get_velocity(self, angle):
        angle = abs(math.degrees(angle))
        vel = 1.3 - angle/40.0
        if vel < 0.7:
            vel = 0.7
        self.horizon = vel*1.5
        if self.horizon < 1.2:
            self.horizon = 1.2
        return vel

    def driver(self, angle, velocity):
        """Publishes the drive message."""
        error = angle
        angle = self.kp*error + self.kd*(error - self.prev_error)
        if abs(angle) < math.pi/90.0:
            angle = 0.0
        self.prev_error = error
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ""
        drive_msg.drive.steering_angle = angle*0.85
        drive_msg.drive.speed = self.get_velocity(error)
        self.drive_pub.publish(drive_msg)

    def emergency_stop(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ""
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = 0
        self.drive_pub.publish(drive_msg)

    def helper(self, close_index, traj):
        if len(close_index) > 1:
            close_points = [traj[close_index[0]], traj[close_index[1]], traj[close_index[-2]], traj[close_index[-1]]]
            pose_tf, index = self.transformer(close_points)
            xg, yg = pose_tf.pose.position.x, pose_tf.pose.position.y
            return xg, yg

    def collision_checker(self, xg, yg):
        # discretizes the path between 0,0 to xg,yg and checks for obstacles on the path
        r = self.safety_radius
        d = r * 0.8  # step size for descretization
        distance = math.sqrt((xg)**2 + (yg)**2)
        iters = int(distance / d)  # min(int(distance / d), 4) #number of points to check
        # if iters < 2: return True
        dd, ind = self.kdt_occ.query([(xg, yg)], k=1)
        if dd[0] < r:
            return True
        else:
            slope = math.atan2(yg, xg)
            if slope > 0.85:
                return True  # not possible to turn above 45-50 degrees
            pts = [(0.0, 0.0)]*iters
            for i_ in range(1,iters):
                xn = i_*d*math.cos(slope) + 0.2
                yn = i_*d*math.sin(slope) 
                pts[i_ - 1] = (xn, yn)
                dd, ind = self.kdt_occ.query([(xn, yn)], k = 1)
                if dd[0] < r:
                    return True
            self.point_viz(pts)
        return False

    def pose_callback(self, pose_msg):
        self.future_col = False
        x, y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
        close_index = sorted(self.kdtree.query_ball_point((x, y), self.horizon))
        if len(close_index) > 1:
            if self.kdt_occ != None:
                xg, yg = self.helper(close_index, self.trajectory_)
                if self.collision_checker(xg, yg):
                    close_index = sorted(self.rightkdtree.query_ball_point((x, y), self.horizon))
                    xg, yg = self.helper(close_index, self.right_traj)
                    if self.collision_checker(xg, yg):
                        #print("right collision", xg, yg)
                        close_index = sorted(self.leftkdtree.query_ball_point((x, y), self.horizon))
                        xg, yg= self.helper(close_index, self.left_traj)
                        if self.collision_checker(xg, yg):
                            #print("all three blocked", xg, yg)
                            self.emergency_stop()
                            self.future_col = True
                if not self.future_col:
                    L = (xg)**2 + (yg)**2
                    angle = 0.0
                    if L > 0.0:
                        angle = 2.0*yg / L
                    self.driver(angle, self.get_velocity(angle))
        # self.kdt_occ = None
        # else: self.driver(0, 0)

    def scan_callback(self, msg):
        pc2_msg = lp.projectLaser(msg)
        point_generator = pc2.read_points(pc2_msg)
        i = 0
        j = 0
        self.occupancy = [(0, 0)] * 599
        # we can access a generator in a loop
        for point in point_generator:
            i += 1
            if i > 240 and i < 840:
                self.occupancy[j] = (point[0], point[1])

                j = j + 1
        self.kdt_occ = KDTree(self.occupancy)


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    waypoint_mid = rospy.Publisher("/wp_mid", Path, queue_size=1, latch=True)
    waypoint_left = rospy.Publisher("/wp_left", Path, queue_size=1, latch=True)
    waypoint_right = rospy.Publisher("/wp_right", Path, queue_size=1, latch=True)
    pp.publish_plan(pp.traj_, waypoint_mid)
    pp.publish_plan(pp.right_traj, waypoint_right)
    pp.publish_plan(pp.left_traj, waypoint_left)
    rospy.spin()
    rospy.sleep(1)
if __name__ == '__main__':
    main()