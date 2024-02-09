#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
import math
import csv
from os.path import expanduser
from tf import TransformListener
from scipy.spatial import cKDTree as KDTree
from sensor_msgs.msg import  LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
lp = lg.LaserProjection()


class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        home = expanduser('~')
        self.horizon = 0.8
        f = home +'/f1ten-scripts/traj.csv'
        with open (f , 'r') as p:
            point_data = list(csv.reader(p))
            p.close()
        datsize = len(point_data)
        self.traj_ = [(0.0, 0.0)]*datsize
        self.refvels = []
        # self.left_traj = [(0.0, 0.0)]*datsize
        # self.right_traj = [(0.0, 0.0)]*datsize
        for i in range(datsize):
            self.traj_[i] = (0.1*float(point_data[i][0]), 0.1*float(point_data[i][1]))
            self.refvels.append(float(point_data[i][2])/7.5)
            # self.left_traj[i] = (float(point_data[i][4]), float(point_data[i][5]))
            # self.right_traj[i] = (float(point_data[i][2]), float(point_data[i][3]))
        self.kdtree = KDTree(self.traj_, leafsize=10)
        # self.leftkdtree = KDTree(self.left_traj, leafsize=10)
        # self.rightkdtree = KDTree(self.right_traj, leafsize=10)
        self.odom_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.marker2_pub = rospy.Publisher('/shapes2', Marker, queue_size=1)
        #self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.tf_list = TransformListener()
        self.targetf = "/car1_30m/laser"  # base_link
        self.sourcef = "map" # global map 
        self.col = False
        self.kp = 0.28
        self.kd = -1.25 
        self.prev_error = 0.0 
        self.safety_radius = 0.15 #30 cm
        self.future_col = False
        self.kdt_occ = None
        self.occupancy = [0]*599
        self.pc_padding = 0.05
        
    def pointVis(self, pointlist):
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
        points.color.a = 1.0
        pp = []
        for i in pointlist:
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
        #self.tf_list.waitForTransform (targetf, sourcef, rospy.Time(0), rospy.Duration(1))
        xlist = 0.0
        ind = 0
        ptf = PoseStamped()
        for i in range(len(points)):
            p=PoseStamped()
            p.header.frame_id = self.sourcef
            p.header.stamp = rospy.Time(0)
            p.pose.position.x, p.pose.position.y = points[i]
            p.pose.orientation.w = 1.0
            pose_tf = self.tf_list.transformPose(self.targetf , p)
            #print(pose_tf.pose.position.x, pose_tf.pose.position.y)
            if pose_tf.pose.position.x > xlist and abs(pose_tf.pose.position.y)< 1 :
                ind = i
                xlist = pose_tf.pose.position.x
                ptf = pose_tf
        return ptf, ind

    def Getvelocity(self,angle):
        angle = abs(math.degrees(angle))
        vel = 2 - angle/40.0
        if vel < 0.7: vel = 0.7
        # self.horizon= 0.1 + 10.5/(angle + 0.001)
        # if self.horizon > 1.2: self.horizon = 1.2
        # if self.horizon < 0.3: self.horizon = 0.3
        return vel
        

    def driver(self, angle, velocity):
        
        error = angle
        angle = self.kp*error + self.kd*(error - self.prev_error)
        if abs(angle) < math.pi/90.0 : angle = 0.0
        self.prev_error = error
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ""
        drive_msg.drive.steering_angle = angle#*0.85
        drive_msg.drive.speed = velocity# self.Getvelocity(error) #velocity
        self.drive_pub.publish(drive_msg)
    
    def emergency_stop(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ""
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = 0
        self.drive_pub.publish(drive_msg)

    def helper(self, closeIndex, traj):
        if len(closeIndex) > 1:
            closepoints = [traj[closeIndex[0]], traj[closeIndex[1]], traj[closeIndex[-2]], traj[closeIndex[-1]]]
            pose_tf, index = self.transformer(closepoints)
            xg, yg = pose_tf.pose.position.x, pose_tf.pose.position.y
            return xg, yg, index
            
    
    def collision_checker(self, xg, yg):
        #discretizes the path between 0,0 to xg,yg and checks for obstacles on the path
        r = self.safety_radius
        d = r*0.8 #step size for descretization
        Dist = math.sqrt((xg)**2 + (yg)**2)
        iters = int(Dist / d)# min(int(Dist / d), 4) #number of points to check
        #if iters < 2: return True TODO why this hurts why car not taking other trajs
        dd, ind = self.kdt_occ.query([(xg, yg)], k = 1)
        if dd[0] < r: 
            return True
        else:
            slope = math.atan2(yg, xg)
            if slope > 0.85: 
                return True #not possible to turn above 45-50 degrees
            #print("checking iters", iters, Dist, xg, yg)
            pts = [(0.0, 0.0)]*iters
            for i_ in range(1,iters):
                xn = i_*d*math.cos(slope) + 0.2
                yn = i_*d*math.sin(slope) 
                pts[i_ - 1] = (xn, yn)
                dd, ind = self.kdt_occ.query([(xn, yn)], k = 1)
                if dd[0] < r:
                    return True
            self.pointVis(pts)
        return False

    
    def pose_callback(self, pose_msg):
        self.future_col = False
        x, y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
        closeIndex = sorted(self.kdtree.query_ball_point((x, y), self.horizon))
        if len(closeIndex) > 1:
            xg, yg, index = self.helper(closeIndex, self.traj_)
            self.horizon = 0.8*self.refvels[int(index)]
            if self.horizon < 0.3: self.horizon = 0.3
            L = (xg)**2 + (yg)**2
            angle = 0.0
            if L > 0.0: angle = 2.0*yg / L
            self.driver(angle, self.Getvelocity(angle))
            self.pointVis([(xg, yg)])
        #self.kdt_occ = None        
        #else: self.driver(0, 0)
    
       

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    waypoint_mid= rospy.Publisher("/wp_mid", Path, queue_size=1, latch=True)
    pp.publish_plan(pp.traj_, waypoint_mid)
        
    #atexit.register(pp.emergency_stop)
    rospy.spin()
    rospy.sleep(1)
if __name__ == '__main__':
    main()