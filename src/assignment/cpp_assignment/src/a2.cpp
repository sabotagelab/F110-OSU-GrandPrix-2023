#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

ros::Publisher brake_pub, bool_pub;
float speed = 0.0;
ackermann_msgs::AckermannDriveStamped driver;
std_msgs::Bool booler;
float cosines[1080];

void GetLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int size = msg->ranges.size();
    booler.data = false;
    if (fabs(speed) > 0.1)
    {
        float TTC = 5.0;
        for (int j =0; j < size; ++j)
        {
          TTC = msg->ranges.at(j)/(cosines[j] * speed);
          if (fabs(TTC) < 0.35)
          {
            driver.header = msg->header;
            brake_pub.publish(driver);
            booler.data = true;
            bool_pub.publish(booler);
            break;
          }
                     
        }
    }
    if (booler.data == false) {bool_pub.publish(booler);}
    
}

void GetOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    ::speed = msg->twist.twist.linear.x;
}


int main(int argc, char **argv)
{
    int index = 0;
    for (float i = -3.14159274101; i < 3.14159274101; i = i + 0.00582315586507)
    {
        ::cosines[index] = cos(i);
        index++;
    }
    ros::init(argc, argv, "a2");
	ros::NodeHandle n;
	brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 1);
	bool_pub = n.advertise<std_msgs::Bool>("brake_bool", 1);
	ros::Subscriber sub = n.subscribe("scan", 1, GetLaser);
    ros::Subscriber subodom = n.subscribe("odom", 1, GetOdom);
	ros::spin();
	return 0;
}