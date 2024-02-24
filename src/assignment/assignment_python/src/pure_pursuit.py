#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
import math
from tf import TransformListener
from scipy.spatial import KDTree


class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """

    def __init__(self):
        with open(("./logs/waypoints.csv"), "r") as p:
            point_data = list(csv.reader(p))
            p.close()

        self.trajectory = []
        for i in range(len(point_data)):
            self.trajectory.append((float(point_data[i][0]), float(point_data[i][1])))
        self.kdtree = KDTree(self.trajectory, leafsize=1000)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/shapes", Marker, queue_size=10)
        self.tf_list = TransformListener()
        self.slow_by_skip = 10  # skips 10 odom data points to reduce frequency
        self.target_frame = "/base_link"
        self.source_frame = "/map"

    def visualize_point(self, point_list):
        points = Marker()
        points.header.stamp = rospy.Time(0)
        points.header.frame_id = self.source_frame
        points.ns = "markers"
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.05
        points.scale.y = 0.05
        points.color.b = 1.0
        points.color.a = 1.0
        pp = []
        for i in point_list:
            p = Point()
            p.x, p.y = self.trajectory[i]
            p.z = 0
            pp.append(p)
        points.points = pp
        self.marker_pub.publish(points)

    def transformer(self, points):
        for i in range(len(points)):
            p = PoseStamped()
            p.header.frame_id = self.source_frame
            p.header.stamp = rospy.Time(0)
            p.pose.position.x, p.pose.position.y = points[i]
            p.pose.orientation.w = 1.0
            pose_tf = self.tf_list.transformPose(self.target_frame, p)
            if pose_tf.pose.position.x > 0:
                return pose_tf, i

    def get_velocity(self, angle):
        angle = abs(math.degrees(angle))
        if angle <= 10:
            return 1.0
        if angle > 10:
            return 0.8
        return 0.5

    def driver(self, angle, velocity):
        if -0.6 < angle < 0.6:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0.45 * angle
            drive_msg.drive.speed = velocity
            self.drive_pub.publish(drive_msg)
        else:
            self.driver(0, 0)

    def pose_callback(self, pose_msg):
        if self.slow_by_skip != 0:
            self.slow_by_skip -= 1
        else:
            x, y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
            close_index = sorted(self.kdtree.query_ball_point((x, y), 0.85))
            if len(close_index) > 1:
                nearby_points = [
                    self.trajectory[close_index[0]],
                    self.trajectory[close_index[-1]],
                ]
                pose_tf, index = self.transformer(nearby_points)
                xg, yg = pose_tf.pose.position.x, pose_tf.pose.position.y
                L = (xg) ** 2 + (yg) ** 2
                angle = 2 * yg / L
                self.driver(angle, self.get_velocity(angle))
                if index < close_index[-1]:
                    self.visualize_point(close_index[(int(len(close_index) / 2)) :])
                if index > close_index[0]:
                    self.visualize_point(close_index[: (int(len(close_index) / 2))])
                self.slow_by_skip = 10
            else:
                self.driver(0, 0)


def main():
    rospy.init_node("pure_pursuit_node")
    pp = PurePursuit()
    rospy.spin()


if __name__ == "__main__":
    main()
