#include <iostream>
#include "ros/ros.h"
#include "racing_core/scan_range.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
ros::Publisher max_pub, min_pub, range_pub;

void GetLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int size = msg->ranges.size();
	//std_msgs::Float32 arr[] = msg->ranges;
	float min = msg->ranges.at(0), max = msg->ranges.at(0);
    for (int i =0; i < size; ++i)
    {
      if ( msg->ranges.at(i)< min) {
            min = msg->ranges.at(i);
        }
 
        if (msg->ranges.at(i)> max) {
            max = msg->ranges.at(i);
        }
    }
    racing_core::scan_range ranger;
    std_msgs::Float32 minD, maxD;
    minD.data = min;
    maxD.data = max;
    ranger.closest = min;
    ranger.farthest = max;
    ranger.header = msg->header;
    range_pub.publish(ranger);
    min_pub.publish(minD);
    max_pub.publish(maxD);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "a1");
	ros::NodeHandle n;
	range_pub = n.advertise<racing_core::scan_range>("scan_range", 10);
	min_pub = n.advertise<std_msgs::Float32>("closest_point", 1);
	max_pub = n.advertise<std_msgs::Float32>("farthest_point", 1);
	ros::Subscriber sub = n.subscribe("scan", 1, GetLaser);
	ros::spin();
	return 0;
}