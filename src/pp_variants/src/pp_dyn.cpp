#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
// using namespace std;
using namespace std;
struct PointXYV
{
  float data[3];
};


class PurePursuit
{
private:
  bool future_col_;
  float cur_vel;
  float horizon;
  float max_horizon;
  float min_horizon;
  float prev_error;
  float safety_radius_;
  int last_index;
  ros::NodeHandle n;
  ros::Publisher drive_pub;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber vel_odom_sub;
  std::string source_frame;
  std::string target_frame;
  std::vector<PointXYV> reference_wp;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;

public:
  geometry_msgs::TransformStamped map_to_laser;
  PurePursuit()
  : odom_sub_(), path_pub_(), marker_pub_(),
    drive_pub(), target_frame(), source_frame(), prev_error(),
    tf2_listener(tf_buffer), vel_odom_sub(), cur_vel(),
    max_horizon(), min_horizon(), horizon(), safety_radius_(),
    last_index()
  {
    odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
    vel_odom_sub = n.subscribe("/vesc/odom", 1, &PurePursuit::vel_callback, this);
    path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
    marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
    target_frame = "car1_30m/laser";
    source_frame = "map";
    prev_error = 0.0f;
    horizon = 0.0f;
    safety_radius_ = 0.15f;
    cur_vel = 0.0f;
    max_horizon = 1.8f;
    min_horizon = 0.5f;
    last_index = -1;
  }

  void point_loader(std::vector<std::vector<float>> a)
  {
    for (int i = 0; i < a.size(); i++) {
      for (int j = 0; j < a[i].size(); j++) {
        PointXYV p1;
        p1.data[0] = 0.1f * a[i][0];
        p1.data[1] = 0.1f * a[i][1];
        p1.data[2] = a[i][2] / 7.5f;
        reference_wp.push_back(p1);
      }
    }
    publishPlan();
  }

  void pointVis(std::pair<float, float> pts)
  {
    visualization_msgs::Marker points;
    points.header.stamp = ros::Time(0);
    points.header.frame_id = source_frame;
    points.ns = "markers2";
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.b = 1.0;
    points.color.a = 0.5;
    std::vector<geometry_msgs::Point> pp;
    geometry_msgs::Point p;
    p.x = pts.first;
    p.y = pts.second;
    p.z = 0;
    pp.push_back(p);
    points.points = pp;
    marker_pub_.publish(points);
  }


  void transformer(const std::vector<int> indices, float * xg, float * yg, int * index_wp)
  {
    float xoi = 0.0f;
    float yoi = 0.0f;
    int ind = 0;
    geometry_msgs::PoseStamped ptf;
    map_to_laser = tf_buffer.lookupTransform(target_frame,
      source_frame, ros::Time(0), ros::Duration(1.0) );

    for (int i = 0; i < indices.size(); ++i) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = source_frame;
      p.header.stamp = ros::Time(0);
      p.pose.position.x = reference_wp[indices[i]].data[0];
      p.pose.position.y = reference_wp[indices[i]].data[1];
      p.pose.orientation.w = 1.0;
      geometry_msgs::PoseStamped pose_tf;
      try {
        tf2::doTransform(p, pose_tf, map_to_laser);
      } catch (tf2::TransformException & ex) {
        ROS_ERROR("%s", ex.what());
        *xg = xoi;
        *yg = yoi;
        *index_wp = ind;
      }

      if (pose_tf.pose.position.x > xoi) {
        ind = i;
        xoi = pose_tf.pose.position.x;
        yoi = pose_tf.pose.position.y;
      }
    }
    *xg = xoi;
    *yg = yoi;
    *index_wp = ind;
  }

  float get_velocity(float angle, int index)
  {
    angle = std::abs(angle * 57.295f);
    // reference velocity profile
    float refvel = reference_wp[index].data[2];
    // custom velocity profile
    float vel = 2.0f - angle / 65.0f;
    if (vel < 1.15f) {vel = 1.15f;}
    return refvel;
  }

  void driver(float angle, int near_ind)
  {
    float kp = 0.33f;
    float error = angle;
    angle = kp * error;
    if (std::abs(angle) < 0.035f) {
      angle = 0.0f;
    }
    prev_error = error;
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "";
    drive_msg.drive.steering_angle = angle;
    float velocity = get_velocity(angle, near_ind);
    drive_msg.drive.speed = velocity;   // velocity
    drive_pub.publish(drive_msg);
  }

  void publishPlan()
  {
    nav_msgs::Path msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i < reference_wp.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = reference_wp[i].data[0];
      pose.pose.position.y = reference_wp[i].data[1];
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    path_pub_.publish(msg);
  }


  std::vector<int> ball_query(float x, float y, float r)
  {
    float rsq = r * r;
    std::vector<int> indices;
    for (int i = 0; i < reference_wp.size(); ++i) {
      float xi = reference_wp[i].data[0];
      float yi = reference_wp[i].data[1];
      float dis = (x - xi) * (x - xi) + (y - yi) * (y - yi);
      if (dis < rsq) {
        indices.push_back(i);
      }
    }
    return indices;
  }

  int nearest_wp(std::vector<int> indices, float x, float y)
  {
    float minD = 1000.0f;
    int idx = 0;
    for (int i = 0; i < indices.size(); ++i) {
      float xi = reference_wp[indices[i]].data[0];
      float yi = reference_wp[indices[i]].data[1];
      float dis = (x - xi) * (x - xi) + (y - yi) * (y - yi);
      if (dis < minD) {
        minD = dis;
        idx = indices[i];
      }
    }
    return idx;
  }


  void vel_callback(const nav_msgs::Odometry::ConstPtr & poser)
  {
    cur_vel = poser->twist.twist.linear.x;
  }

  void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_msg)
  {
    float x = pose_msg->pose.pose.position.x;
    float y = pose_msg->pose.pose.position.y;
    float r = std::min(std::max(min_horizon, max_horizon * cur_vel / 1.8f), max_horizon);
    std::vector<int> indices = ball_query(x, y, r);
    if (indices.size() > 0) {
      std::vector<std::vector<float>> pts;
      float xg, yg;
      int ind;
      transformer(indices, &xg, &yg, &ind);
      pointVis(std::make_pair(reference_wp[indices[ind]].data[0],
        reference_wp[indices[ind]].data[1]));
      float L = (xg * xg) + (yg * yg);
      float angle = 0.0f;
      if (L > 0) {
        last_index = indices[ind];
        int near_ind = nearest_wp(indices, x, y);
        angle = 2 * yg / L;
        driver(angle, near_ind);
      }
    }
  }
};


class WaypointServer
{
public:
  std::vector<std::vector<float>> load(std::string waypoint_file)
  {
    const int rows_num = 3;
    boost::char_separator<char> sep(",", "", boost::keep_empty_tokens);
    std::ifstream ifs(waypoint_file.c_str());
    std::string line;
    std::vector<std::vector<float>> wps;
    while (ifs.good()) {
      getline(ifs, line);
      if (line.empty()) {break;}
      tokenizer tokens(line, sep);
      std::vector<float> data;
      tokenizer::iterator it = tokens.begin();
      for (; it != tokens.end(); ++it) {
        std::stringstream ss;
        float d;
        ss << *it;
        ss >> d;
        data.push_back(d);
      }
      wps.push_back(data);
    }
    return wps;
  }
};

int main(int argc, char ** argv)
{
  std::string fname = "./data/traj.csv";
  WaypointServer waypoint_server;
  std::vector<std::vector<float>> data_wp = waypoint_server.load(fname);
  ros::init(argc, argv, "pp_dyn");
  PurePursuit pp;
  pp.point_loader(data_wp);
  ros::spin();
  return 0;
}
