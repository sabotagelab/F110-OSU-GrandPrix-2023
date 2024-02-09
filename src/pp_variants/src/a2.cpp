#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

ros::Publisher brake_pub;
float speed = 0.0;
ackermann_msgs::AckermannDriveStamped driver;
std_msgs::Bool booler;
float cosines[1080];            
float stop_vel = 1.0;
float input_speed = 0.0;


void GetLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int size = msg->ranges.size();

    if (::booler.data)
    {
        if (::input_speed > ::stop_vel && abs(::speed) > 0)
        {
            driver.drive.speed = 0.0;
            brake_pub.publish(driver);
        }

        else
        {
            ::booler.data = false;
            ::stop_vel = 0;
        }
    }
    if (!::booler.data)
    {
        float TTC = 5.0;
        for (int j =270; j < 810; ++j)
        {
            if (msg->ranges.at(j) > 0.005)
            {
                TTC = msg->ranges.at(j)/(cosines[j] * ::input_speed);
                if (fabs(TTC) < 0.35)
                {
                    driver.drive.speed = 0.0;
                    brake_pub.publish(driver);
                    ::booler.data = true;
                    ::stop_vel = (msg->ranges.at(j)/0.35) - 0.3;
                    if (::stop_vel < 0.1){::stop_vel = 0.1;}
                    std::cout<<j<<" stoppping "<< msg->ranges.at(j)<< " stop_vel "<< ::stop_vel <<" TTC "<<TTC<<std::endl; 
                    //ROS_INFO_STREAM( " stop_vel "<< ::stop_vel <<" TTC "<<TTC);           
                    break;
                }
            }
          
                     
        }
    }
    
}

void GetOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    ::speed = msg->twist.twist.linear.x;
}

void GetInput(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    ::input_speed = fabs(msg->drive.speed) + 0.02;
    ::driver = *msg;
}

int main(int argc, char **argv)
{
    int index = 0;
    for (float i = -3.14159; i < 3.14159; i = i + 0.00581776)
    {
        ::cosines[index] = cos(i);
        index++;
    }
    ros::init(argc, argv, "a2");
	ros::NodeHandle n;
    booler.data = false;
	brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/safety", 1);
	//bool_pub = n.advertise<std_msgs::Bool>("brake_bool", 1);
	ros::Subscriber sub = n.subscribe("/scan", 1, GetLaser);
    ros::Subscriber subodom = n.subscribe("/vesc/odom", 1, GetOdom);
    ros::Subscriber input_ = n.subscribe("/vesc/low_level/ackermann_cmd_mux/input/navigation", 1, GetInput);
	ros::spin();
	return 0;
}