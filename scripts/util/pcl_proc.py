'''
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

ros::Publisher padded_cloud_pub;
laser_geometry::LaserProjection projector;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
  sensor_msgs::PointCloud2 cloud_msg;
  projector.projectLaser(*scan_msg, cloud_msg);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  pcl::PointCloud<pcl::PointXYZ> padded_cloud;
  padded_cloud += cloud;

  float padding_distance = 0.1; // Change this value as needed
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(padding_distance);
  octree.setInputCloud(padded_cloud.makeShared());
  octree.addPointsFromInputCloud();
  octree.switchBuffers();
  octree.setInputCloud(cloud.makeShared());
  octree.addPointsFromInputCloud();
  std::vector<int> new_points;
  octree.getPointIndicesFromNewVoxels(new_points);

  for (int i = 0; i < new_points.size(); ++i) {
    pcl::PointXYZ new_point;
    new_point.x = cloud.points[new_points[i]].x;
    new_point.y = cloud.points[new_points[i]].y;
    new_point.z = cloud.points[new_points[i]].z;
    padded_cloud.points.push_back(new_point);
  }

  sensor_msgs::PointCloud2 padded_cloud_msg;
  pcl::toROSMsg(padded_cloud, padded_cloud_msg);
  padded_cloud_msg.header = scan_msg->header;
  padded_cloud_pub.publish(padded_cloud_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_to_padded_point_cloud");
  ros::NodeHandle nh;
  ros::Subscriber laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);
  padded_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("padded_cloud", 1);
  ros::spin();
  return 0;
}


# '''
# #!/usr/bin/env python

# import rospy
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import LaserScan, PointCloud2
# from laser_geometry import LaserProjection
# from pcl_conversions import pcl_conversions
# from pcl import PointXYZ, PointCloud, octree
# import numpy as np

# padded_cloud_pub = None
# projector = LaserProjection()

# def laser_scan_callback(scan_msg):
#     global padded_cloud_pub, projector
#     cloud_msg = projector.projectLaser(scan_msg)
#     cloud = pc2.read_points(cloud_msg, field_names=("x", "y", "z"))
#     cloud = np.array(list(cloud), dtype=np.float32)
#     padded_cloud = PointCloud()
#     padded_cloud.from_array(cloud)

#     padding_distance = 0.1 # Change this value as needed
#     octree_resolution = 0.01 # Change this value as needed
#     octree = octree.OctreePointCloudChangeDetector(PointXYZ(octree_resolution))
#     octree.setInputCloud(padded_cloud)
#     octree.addPointsFromInputCloud()
#     octree.switchBuffers()
#     octree.setInputCloud(PointCloud.from_array(cloud))
#     octree.addPointsFromInputCloud()
#     new_points = octree.getPointIndicesFromNewVoxels()

#     for i in new_points:
#         new_point = PointXYZ(cloud[i, 0], cloud[i, 1], cloud[i, 2])
#         padded_cloud.push_back(new_point)

#     padded_cloud_msg = PointCloud2()
#     pcl_conversions.fromPCL(padded_cloud, padded_cloud_msg)
#     padded_cloud_msg.header = scan_msg.header
#     padded_cloud_pub.publish(padded_cloud_msg)

# def main():
#     global padded_cloud_pub
#     rospy.init_node("laser_scan_to_padded_point_cloud")
#     rospy.Subscriber("scan", LaserScan, laser_scan_callback)
#     padded_cloud_pub = rospy.Publisher("padded_cloud", PointCloud2, queue_size=1)
#     rospy.spin()

# if __name__ == '__main__':
#     main()


