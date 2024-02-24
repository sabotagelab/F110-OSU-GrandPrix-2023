#include <cmath>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


class Safety
{
// The class that handles emergency braking

private:
  ros::NodeHandle n;
  double speed;
  ros::Publisher brake_pub, bool_pub;
  ros::Subscriber scan_sub, odom_sub;
  ackermann_msgs::AckermannDriveStamped driver;
  std_msgs::Bool collision_bool;
  float cosines[1080];
  int index = 0;

public:
  Safety()
  {
    n = ros::NodeHandle();
    speed = 0.0;
    brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 1);
    bool_pub = n.advertise<std_msgs::Bool>("brake_bool", 1);
    scan_sub = n.subscribe("scan", 1, &Safety::scan_callback, this);
    odom_sub = n.subscribe("odom", 1, &Safety::odom_callback, this);
    for (float i = -3.14159274101; i < 3.14159274101; i = i + 0.00582315586507) {
      cosines[index] = cos(i);
      index++;
    }
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr & odom_msg)
  {
    speed = odom_msg->twist.twist.linear.x;
  }

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr & msg)
  {
    int size = msg->ranges.size();
    collision_bool.data = false;
    if (fabs(speed) > 0.1) {
      float TTC = 5.0;
      for (int j = 0; j < size; ++j) {
        TTC = msg->ranges.at(j) / (cosines[j] * speed);
        if (fabs(TTC) < 0.35) {
          driver.header = msg->header;
          brake_pub.publish(driver);
          collision_bool.data = true;
          bool_pub.publish(collision_bool);
          break;
        }
      }
    }
    if (collision_bool.data == false) {bool_pub.publish(collision_bool);}
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "safety_node");
  Safety sn;
  ros::spin();
  return 0;
}
