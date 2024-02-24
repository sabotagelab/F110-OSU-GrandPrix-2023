#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
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

using namespace std;
struct PointXY
{
  float data[2];
};


class PurePursuit
{
private:
  ros::NodeHandle n;
  bool future_col_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher drive_pub;
  std::string target_frame;
  std::string source_frame;
  float prev_error;
  std::vector<PointXY> reference_wp;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;

public:
  geometry_msgs::TransformStamped map_to_laser;
  PurePursuit()
  : odom_sub_(), path_pub_(), marker_pub_(),
    drive_pub(), target_frame(), source_frame(),
    prev_error(), tf2_listener(tf_buffer)
  {
    odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
    path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
    marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
    target_frame = "car1_30m/laser";
    source_frame = "map";
    prev_error = 0.0f;
  }

  void point_loader(std::vector<std::vector<float>> a)
  {
    for (int i = 0; i < a.size(); i++) {
      for (int j = 0; j < a[i].size(); j++) {
        PointXY p1;
        float x = 0.1f * a[i][0];
        float y = 0.1f * a[i][1];
        p1.data[0] = x;
        p1.data[1] = y;
        reference_wp.push_back(p1);
      }
    }
    publishPlan();
    std::cout << reference_wp.size() << std::endl;
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

  void transformer(const std::vector<int> indices, float * xg, float * yg, int * index_pt)
  {
    float xoi = 0.0f;
    float yoi = 0.0f;
    int ind = 0;
    geometry_msgs::PoseStamped ptf;
    map_to_laser = tf_buffer.lookupTransform(
      target_frame,
      source_frame,
      ros::Time(0),
      ros::Duration(1.0));

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
        *index_pt = ind;
      }

      if (pose_tf.pose.position.x > xoi) {
        ind = i;
        xoi = pose_tf.pose.position.x;
        yoi = pose_tf.pose.position.y;
      }
    }
    *xg = xoi;
    *yg = yoi;
    *index_pt = ind;
  }

  float get_velocity(float angle)
  {
    angle = std::abs(angle * 57.295f);
    float vel = 2.0f - angle / 65.0f;
    if (vel < 1.15f) {
      vel = 1.15f;
    }
    return vel;
  }


  void driver(float angle, float velocity)
  {
    float kp = 0.32f;
    float error = angle;
    angle = kp * error;
    if (std::abs(angle) < 0.033f) {
      angle = 0.0f;
    }
    prev_error = error;
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "";
    drive_msg.drive.steering_angle = angle;
    velocity = get_velocity(angle);
    drive_msg.drive.speed = velocity;
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


  void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_msg)
  {
    float x = pose_msg->pose.pose.position.x;
    float y = pose_msg->pose.pose.position.y;
    float r = 0.9;

    std::vector<int> indices = ball_query(x, y, r);
    if (indices.size() > 0) {
      float xg, yg;
      int ind;
      transformer(indices, &xg, &yg, &ind);
      pointVis(
        std::make_pair(
          reference_wp[indices[ind]].data[0],
          reference_wp[indices[ind]].data[1]));

      float L = (xg * xg) + (yg * yg);
      float angle = 0.0f;
      if (L > 0) {
        angle = 2 * yg / L;
        float velocity = 0.0f;
        driver(angle, velocity);
      }
    } else {driver(0.0f, 0.0f);}
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
  std::vector<std::vector<float>> datawp = waypoint_server.load(fname);
  std::cout << datawp.size() << std::endl;
  ros::init(argc, argv, "pure_pursuit_node");
  PurePursuit pp;
  pp.point_loader(datawp);
  ros::spin();
  return 0;
}
