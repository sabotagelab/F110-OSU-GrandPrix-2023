#!/usr/bin/env python

import csv
import math
import numpy as np
from os.path import expanduser

import rospy
from geometry_msgs.msg import PointStamped, Point
import laser_geometry.laser_geometry as lg
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int8, Float32
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs

from scipy.spatial import cKDTree as KDTree

lp = lg.LaserProjection()


class CheckCenterTraj:
    def __init__(self):
        rospy.init_node('check_center_traj')

        home = expanduser('~')
        f = home +'/f1ten-scripts/ways.csv'
        with open (f , 'r') as p:
            point_data = list(csv.reader(p))
            p.close()
        datsize = len(point_data)
        traj_ = [(0.0, 0.0)]*datsize
        left_traj_ = [(0.0, 0.0)]*datsize
        right_traj_ = [(0.0, 0.0)]*datsize
        for i in range(datsize):
            traj_[i] = (float(point_data[i][0]), float(point_data[i][1]))
            left_traj_[i] = (float(point_data[i][4]), float(point_data[i][5]))
            right_traj_[i] = (float(point_data[i][2]), float(point_data[i][3]))
        self.center_traj = KDTree(traj_, leafsize=10)
        self.left_traj = KDTree(left_traj_, leafsize=10)
        self.right_traj = KDTree(right_traj_, leafsize=10)
        self.marker2_pub = rospy.Publisher('/shapes2', Marker, queue_size=1)


        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        #self.pc_pub = rospy.Publisher('/transformed_points', PointCloud, queue_size=10)
        self.center_enum_pub = rospy.Publisher('/center_traj_free', Int8, queue_size=10)
        self.alt_traj_enum_pub = rospy.Publisher('/alt_traj', Int8, queue_size=10)
        self.obs_velocity_pub = rospy.Publisher('/obst_velocity', Float32, queue_size=10)

        #self.publish_static_dynamic = False

        #self.tf_listener = TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # PARAMETER FOR CHECKING POINT INTERSECTION WITH TRAJECTORIES
        self.traj_safety_radius = 0.13

        # PARAMETER FOR VELOCITY THRESHOLD BEFORE CONSIDERING POINT AS "DYNAMIC"
        self.dynamic_thresh = 1.0

        # USE THESE PARAMETERS TO FILTER SCANS
        # self.min_r = 0
        # self.max_r = 0.5
        self.min_angle = 360
        self.max_angle = 750
        self.max_horizon_sq = 2.0**2

        # TRACK THE MAP POINTS TO CALCULATE THEIR VELOCITY
        self.laser_points = []
        self.laser_points_ids = []
        self.map_points = []
        self.save_ts = rospy.Time.now()

    def pointVis(self, pointlist):
        points = Marker()
        points.header.stamp = rospy.Time(0)
        points.header.frame_id = "/map"
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
        for i in pointlist:
            p=Point()
            p.x, p.y = i
            p.z = 0
            pp.append(p)
        points.points = pp
        self.marker2_pub.publish(points)

    def laser_callback(self, scan):
        # Convert laser scan to point cloud
        # only convert scans within the filter parameters
        pc2_msg = lp.projectLaser(scan)
        point_generator = pc2.read_points(pc2_msg)
        i = 0
        laser_points = []
        laser_points_ids = []
        map_points = []
        transform = self.tf_buffer.lookup_transform('map', scan.header.frame_id, rospy.Time())
        #print('total points to transform', len(point_generator))
        # we can access a generator in a loop
        for point in point_generator:
            i += 1 #todo: angle crop doesnt work on cornerssad
            if i > self.min_angle and i < self.max_angle and i%2 == 0:
                dist = point[0]**2 + point[1]**2
                if dist < self.max_horizon_sq:
                    point_stamped = PointStamped()
                    point_stamped.header = scan.header
                    point_stamped.point.x = point[0]
                    point_stamped.point.y = point[1]
                    try:
                        map_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                        p = [map_point_stamped.point.x, map_point_stamped.point.y]
                        laser_points.append([point[0], point[1]])
                        laser_points_ids.append(i)
                        map_points.append(p)
                    except Exception as err:
                        #print(err)
                        rospy.logwarn('Failed to transform point to map frame')

        self.pointVis(map_points)
        self.publish_enums(map_points, laser_points, laser_points_ids)

    def publish_enums(self, map_points, laser_points, laser_points_ids):
        # print('publishing enums')
        not_free_flag = False
        #print(map_points)
        if len(map_points) > 0:
            min_dists_to_center_traj, _ = self.center_traj.query(map_points, k=1)
            min_of_min_dists_to_center = min(min_dists_to_center_traj)
            # print('min dist to center ', min_of_min_dists_to_center)
            if min_of_min_dists_to_center <= self.traj_safety_radius:
                not_free_flag = True
                min_dists_to_left_traj, ii = self.left_traj.query(map_points, k = 1)
                min_of_min_dists_to_left = min(min_dists_to_left_traj)
                # print('min dist to left ', min_of_min_dists_to_left)
                if min_of_min_dists_to_left >= self.traj_safety_radius:
                    self.alt_traj_enum_pub.publish(1)
                else:
                    min_dists_to_right_traj, iii = self.right_traj.query(map_points, k = 1)
                    min_of_min_dists_to_right = min(min_dists_to_right_traj)
                    # print('min dist to right ', min_of_min_dists_to_right)
                    if min_of_min_dists_to_right >= self.traj_safety_radius:
                        self.alt_traj_enum_pub.publish(2)
                    else:
                        # print('all blocked')
                        self.alt_traj_enum_pub.publish(3)

        if not not_free_flag:
            self.center_enum_pub.publish(0)
        else:
            #self.publish_static_dynamic = True

            # pass only the LASER points that are close to the center trajectory
            #obs_in_center = min_dists_to_center_traj[min_dists_to_center_traj <= self.traj_safety_radius]
            filter_ids = np.where(min_dists_to_center_traj <= self.traj_safety_radius)[0]
            obs_laser_points = [laser_points[i] for i in filter_ids]  
            obs_laser_ids = [laser_points_ids[i] for i in filter_ids]  
            #self.compute_velocity(map_points)
            self.compute_velocity(obs_laser_points, obs_laser_ids)

        #if self.publish_static_dynamic:
        #    self.publish_static_dynamic = False

    def compute_velocity(self, laser_points, laser_points_ids):
	if len(self.laser_points) == 0:
            # don't have enough reference points to compute velocity, assume static obstacle for now
            self.center_enum_pub.publish(1)
            self.laser_points = laser_points
	    self.laser_points_ids = laser_points_ids
            self.save_ts = rospy.Time.now()
            return None

        obs_laser_points = []
        obs_laser_points_ids = []
        old_obs_laser_points = []
        for i, laser_id in enumerate(laser_points_ids):
            if laser_id in self.laser_points_ids:  # if laser point id matches an id in the old scan, keep both points
                obs_laser_points.append(laser_points[i])  # keep the laser point at the index associated with the laser_id
                obs_laser_points_ids.append(laser_id)  # save that laser_id for next time
                old_laser_id_match = self.laser_points_ids.index(laser_id)  # get the index of the matching laser point from last scan
                old_obs_laser_points.append(self.laser_points[old_laser_id_match])  # keep that last scan

        #obs_centroid = np.mean(obs_laser_points, axis=0)
        #old_obs_centroid = np.mean(old_obs_laser_points, axis=0)
        
	#if len(self.map_points) == 0:
        #    # don't have enough reference point to compute velocity, assume static obstacle for now
        #    self.center_enum_pub.publish(1)
        #    self.map_points = map_points
        #    self.save_ts = rospy.Time.now()
        #    return None
        ## Compare with last map points to calculate instantaneous velocity
        #if len(map_points) > len(self.map_points):
        #    cap_len = len(self.map_points)
        #else:
        #    cap_len = len(map_points)
        #dist = np.array(map_points[:cap_len]) - np.array(self.map_points[:cap_len])
        #dist = np.linalg.norm(obs_centroid - old_obs_centroid)
        dist = np.array(obs_laser_points) - np.array(old_obs_laser_points)
        time_diff = rospy.Time.now() - self.save_ts
        time_diff_s = time_diff.to_sec()
        laser_points_vel = dist / time_diff_s

        if np.isinf(laser_points_vel).any() or np.isnan(laser_points_vel).any():
            # problem calculating velocity, save laser points for next time, assume a static obstacle for now
            #self.map_points = map_points
            self.laser_points = obs_laser_points
            self.laser_points_ids = obs_laser_points_ids
            self.save_ts = rospy.Time.now()
            self.center_enum_pub.publish(1)
            #print("Map Points velocity")
            #print("time diff", time_diff_s)
            return None

        bin_size, bin_edges = np.histogram(laser_points_vel, bins=10)
        #bin_size, bin_edges = np.histogram(map_points_vel, bins=10)
        max_bin_index = np.argmax(bin_size)
        max_bin_low = bin_edges[max_bin_index]
        max_bin_high = bin_edges[max_bin_index+1]
        mode_vel = np.mean([max_bin_low, max_bin_high])
        laser_points_vel = mode_vel
        print('Velocity', laser_points_vel)

        if laser_points_vel > self.dynamic_thresh:
            self.obs_velocity_pub.publish(laser_points_vel)
            self.center_enum_pub.publish(2)
        else:
            self.center_enum_pub.publish(1)

        # Save the laser points for the next observation
        self.laser_points = obs_laser_points
        self.laser_points_ids = obs_laser_points_ids
        self.save_ts = rospy.Time.now()

        #self.publish_to_base_link(map_points)
        return None

    def publish_to_base_link(self, map_points):
        # Transform points to base link frame
        transform = self.tf_buffer.lookup_transform('car1_30m/laser', 'map', rospy.Time(0))
        base_link_points = []
        for i, point in enumerate(map_points):
            try:
                point_with_vel = PointStamped()
		point_with_vel.header = point.header
                point_with_vel.point.x = point[0]
                point_with_vel.point.y = point[1]
                point_with_vel.point.z = self.map_points_vel[i]  # replace z coord with velocity
                base_link_point = tf2_geometry_msgs.do_transform_point(point_with_vel, transform)
                base_link_points.append(base_link_point)
            except Exception as err:
                #print(err)
                rospy.logwarn('Failed to transform point to map frame')

        # Publish transformed points
        pc = PointCloud()
        pc.points = base_link_points
        self.pc_pub.publish(pc)


if __name__ == '__main__':
    try:
        node = CheckCenterTraj()
        rospy.sleep(0.1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
