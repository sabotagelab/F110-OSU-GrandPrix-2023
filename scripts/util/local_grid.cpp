// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
ros::Publisher grid_pub;
nav_msgs::OccupancyGrid occ_grid;


void scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg)
{

  float x = 12.0;
  float y = 0.0;
  int m[576];
  int rmax = 24;   //max range
  float adjust = 2;

  occ_grid.header.frame_id = "/car1_30m/base_link";
  occ_grid.info.resolution = 0.1;
  occ_grid.info.width = 24;
  occ_grid.info.height = 24;
  occ_grid.info.origin.position.y = -1.2;
  occ_grid.info.origin.orientation.w = 1.0;

  for (int i = 0; i < rmax; i++) {
    for (int j = 0; j < rmax; j++) {
      float r = sqrt(pow((x - i), 2) + pow((y - j), 2));
      float theta = ((atan2((y - j), (x - i))) * 57.2958) - 90;
      int index = static_cast<int>(fabs(theta * 3));
      m[j + rmax * i] = 60;
      if (index < 1080) {
        float meas_r = (scan_msg->ranges[index]) / 0.1;
        //if (r > (meas_r + adjust)){m[j +  rmax*i] = 60;}
        if (fabs(r - meas_r) < adjust) {m[j + rmax * i] = 100;}
        //if (r < meas_r){m[j +  rmax*i] = 10;}
      }
    }
  }
  std::vector<signed char> a(m, m + 576);
  occ_grid.data = a;
  grid_pub.publish(occ_grid);
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "local_grid");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, scan_callback);
  grid_pub = n.advertise<nav_msgs::OccupancyGrid>("grid", 1);

  ros::spin();
  return 0;
}
