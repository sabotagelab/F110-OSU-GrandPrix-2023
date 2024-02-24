#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
ros::Publisher grid_pub;
nav_msgs::OccupancyGrid occ_grid;



void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
 
    float x =  120.0;
    float y =  0.0;
    int m[57600];
    int rmax = 240; //max range
    float adjust = 3.0;
    
    occ_grid.header.frame_id = "/car1_30m/base_link";
    occ_grid.info.resolution = 0.01;
    occ_grid.info.width = 240;
    occ_grid.info.height = 240;
    occ_grid.info.origin.position.y = -1.2;
    occ_grid.info.origin.position.x = 0.0;
    occ_grid.info.origin.orientation.w = 1.0;

    for (int i = 0; i <  rmax; i++)
    {
        for (int j = 0; j <  rmax; j++)
        {
            float r = sqrt(pow((x - i),2) + pow((y - j),2));
            float theta = ((atan2((y-j),(x-i)))*57.2958) -90;
            int index = static_cast<int>(fabs(theta*3));
            m[j +  rmax*i] = 60;
            if (index < 1080){
                float meas_r = (scan_msg->ranges[index])*100;
                //if (r > (meas_r + adjust)){m[j +  rmax*i] = 60;}
                if ((fabs(r - meas_r) <= adjust) && meas_r <= r){m[j +  rmax*i] = 100;}
                //if (r < meas_r){m[j +  rmax*i] = 10;}
                }
        }
    }
    std::vector<signed char> a(m, m+57600);
    occ_grid.data = a;
    grid_pub.publish(occ_grid);  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_grid");
	ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1, scan_callback);
    grid_pub = n.advertise<nav_msgs::OccupancyGrid>("grid", 1);

	ros::spin();
	return 0;
}


// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <laser_geometry/laser_geometry.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/octree/octree.h>

// ros::Publisher padded_cloud_pub;
// laser_geometry::LaserProjection projector;

// void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
//   sensor_msgs::PointCloud2 cloud_msg;
//   projector.projectLaser(*scan_msg, cloud_msg);
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   pcl::fromROSMsg(cloud_msg, cloud);

//   pcl::PointCloud<pcl::PointXYZ> padded_cloud;
//   padded_cloud += cloud;

//   float padding_distance = 0.1; // Change this value as needed
//   pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(padding_distance);
//   octree.setInputCloud(padded_cloud.makeShared());
//   octree.addPointsFromInputCloud();
//   octree.switchBuffers();
//   octree.setInputCloud(cloud.makeShared());
//   octree.addPointsFromInputCloud();
//   std::vector<int> new_points;
//   octree.getPointIndicesFromNewVoxels(new_points);

//   for (int i = 0; i < new_points.size(); ++i) {
//     pcl::PointXYZ new_point;
//     new_point.x = cloud.points[new_points[i]].x;
//     new_point.y = cloud.points[new_points[i]].y;
//     new_point.z = cloud.points[new_points[i]].z;
//     padded_cloud.points.push_back(new_point);
//   }

//   sensor_msgs::PointCloud2 padded_cloud_msg;
//   pcl::toROSMsg(padded_cloud, padded_cloud_msg);
//   padded_cloud_msg.header = scan_msg->header;
//   padded_cloud_pub.publish(padded_cloud_msg);
// }

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "laser_scan_to_padded_point_cloud");
//   ros::NodeHandle nh;
//   ros::Subscriber laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);
//   padded_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("padded_cloud", 1);
//   ros::spin();
//   return 0;
// }