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
struct PointXY
{
  float data[2];
};


class PurePursuit
{
private:
  bool future_col_;
  float cur_vel;
  float prev_error;
  float safety_radius_;
  ros::NodeHandle n;
  ros::Publisher drive_pub;
  ros::Publisher leftpath_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher rightpath_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber vel_odom_sub;
  std::string source_frame;
  std::string target_frame;
  std::vector<PointXY> left_wp;
  std::vector<PointXY> occupancy;
  std::vector<PointXY> reference_wp;
  std::vector<PointXY> right_wp;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;

public:
  laser_geometry::LaserProjection projector_;
  geometry_msgs::TransformStamped map_to_laser;
  PurePursuit()
  : odom_sub_(), vel_odom_sub(), scan_sub_(),
    path_pub_(), marker_pub_(), drive_pub(), target_frame(),
    source_frame(), prev_error(), tf2_listener(tf_buffer),
    safety_radius_(), cur_vel(), leftpath_pub_(), rightpath_pub_()
  {
    odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
    vel_odom_sub = n.subscribe("/vesc/odom", 1, &PurePursuit::vel_callback, this);

    path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
    scan_sub_ = n.subscribe("/scan", 1, &PurePursuit::scanCallback, this);
    leftpath_pub_ = n.advertise<nav_msgs::Path>("/wps_left", 1, true);
    rightpath_pub_ = n.advertise<nav_msgs::Path>("/wps_right", 1, true);
    marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
    target_frame = "car1_30m/laser";
    source_frame = "map";
    prev_error = 0.0f;
    safety_radius_ = 0.15f;
    cur_vel = 0.0f;
  }

  void point_loader(std::vector<std::vector<float>> a)
  {
    for (int i = 0; i < a.size(); i++) {
      for (int j = 0; j < a[i].size(); j++) {
        PointXY p2, p3;
        p2.data[0] = a[i][2];
        p2.data[1] = a[i][3];
        p3.data[0] = a[i][4];
        p3.data[1] = a[i][5];
        right_wp.push_back(p2);
        left_wp.push_back(p3);
      }
    }
    publish_plan(left_wp, leftpath_pub_);
    publish_plan(right_wp, rightpath_pub_);
  }

  void point_loader2(std::vector<std::vector<float>> a)
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
    publish_plan(reference_wp, path_pub_);
  }

  void pointVis(std::pair<float, float> pts)
  {
    visualization_msgs::Marker points;
    points.header.stamp = ros::Time(0);
    points.header.frame_id = target_frame;
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

  void transformer(
    const std::vector<int> indices,
    std::vector<PointXY> pts,
    float * xg,
    float * yg,
    int * index_wp)
  {
    float xoi = 0.0f;
    float yoi = 0.0f;
    int ind = 0;
    geometry_msgs::PoseStamped ptf;
    map_to_laser = tf_buffer.lookupTransform(
      target_frame, source_frame, ros::Time(0), ros::Duration(1.0) );

    for (int i = 0; i < indices.size(); ++i) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = source_frame;
      p.header.stamp = ros::Time(0);
      p.pose.position.x = pts[indices[i]].data[0];
      p.pose.position.y = pts[indices[i]].data[1];
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

  float get_velocity(float angle)
  {
    angle = std::abs(angle * 57.295f);
    float vel = 1.5f - angle / 30.0f;
    if (vel < 0.7f) {
      vel = 0.7;
    }
    return vel;
  }

  void driver(float angle, int stop = 1)
  {
    float kp = 0.33f;
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
    float velocity = get_velocity(angle);
    drive_msg.drive.speed = velocity * stop;   // velocity
    drive_pub.publish(drive_msg);
  }

  void publish_plan(std::vector<PointXY> traj, ros::Publisher path_pub)
  {
    nav_msgs::Path msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i < traj.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = traj[i].data[0];
      pose.pose.position.y = traj[i].data[1];
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    std::cout << "published plan" << msg.poses.size() << std::endl;
    path_pub.publish(msg);
  }


  std::vector<int> ball_query(std::vector<PointXY> refs, float x, float y, float r)
  {
    float rsq = r * r;
    std::vector<int> indices;
    for (int i = 0; i < refs.size(); ++i) {
      float xi = refs[i].data[0];
      float yi = refs[i].data[1];
      float dis = (x - xi) * (x - xi) + (y - yi) * (y - yi);
      if (dis < rsq) {
        indices.push_back(i);
      }
    }
    return indices;
  }

  float nearest_wp(float x, float y)
  {
    float minD = 100.0f;
    for (int i = 0; i < occupancy.size(); ++i) {
      float xi = occupancy[i].data[0];
      float yi = occupancy[i].data[1];
      float dis = (x - xi) * (x - xi) + (y - yi) * (y - yi);
      if (dis < minD) {
        minD = dis;
      }
    }
    return minD;
  }

  bool collision_checker(float xg, float yg)
  {
    float r = safety_radius_;
    float d = r * 0.8f;   // step size for discretization
    float Dist = std::sqrt(xg * xg + yg * yg);
    int iters = std::max(std::min(static_cast<int>(Dist / d), 2), 6);   // number of points to check
    float dd = nearest_wp(xg, yg);
    if (dd < r * r) {return true;} else {
      float slope = std::atan2(yg, xg);
      for (int i = 1; i < iters; ++i) {
        float xn = i * d * std::cos(slope) + 0.1f;
        float yn = i * d * std::sin(slope);
        float dd = nearest_wp(xn, yn);
        if (dd < r * r) {
          std::cout << "xn, yn, dd, r " << xn << " " << yn << " " << dd << " " << r << std::endl;
          return true;
        }
      }
      return false;
    }
  }

  void vel_callback(const nav_msgs::Odometry::ConstPtr & poser)
  {cur_vel = poser->twist.twist.linear.x;}

  void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_msg)
  {
    float x = pose_msg->pose.pose.position.x;
    float y = pose_msg->pose.pose.position.y;
    float r = std::min(std::max(0.7f, 1.6f * cur_vel / 1.8f), 1.6f);
    // float r = 1.2;
    std::vector<int> indices = ball_query(reference_wp, x, y, r);
    float xg, yg;
    int ind;
    future_col_ = false;

    transformer(indices, reference_wp, &xg, &yg, &ind);
    if (collision_checker(xg, yg)) {
      std::cout << "Sed, I am crashing soon, send help!" << std::endl;

      // checking centre traj
      std::vector<int> indices = ball_query(left_wp, x, y, r);
      transformer(indices, left_wp, &xg, &yg, &ind);
      if (collision_checker(xg, yg)) {    // checking left traj
        std::cout << "left crash, send help!" << std::endl;
        std::vector<int> indices = ball_query(right_wp, x, y, r);
        transformer(indices, right_wp, &xg, &yg, &ind);

        if (collision_checker(xg, yg)) {
          std::cout << "all blocked ##" << std::endl;
          future_col_ = true;
        }
      }
    }
    if (future_col_) {driver(0.0, 0);} else {
      pointVis(std::make_pair(xg, yg));

      float L = (xg * xg) + (yg * yg);
      float angle = 0.0f;
      if (L > 0) {
        angle = 2 * yg / L;
        driver(angle);
      }
    }
    occupancy.clear();
  }


  void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_in)
  {
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan_in, cloud);
    int i = 0;
    occupancy.clear();
    occupancy.reserve(400);
    for (int i = 0; i < cloud.points.size(); ++i) {
      if (i > 240 && i < 840 && i % 2 == 0) {
        PointXY p1;
        p1.data[0] = static_cast<float>(cloud.points[i].x);
        p1.data[1] = static_cast<float>(cloud.points[i].y);
        occupancy.push_back(p1);
      }
    }
  }
};


class WaypointServer
{
public:
  std::vector<std::vector<float>> load(std::string waypoint_file, int rows)
  {
    const int rows_num = rows;
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
  std::string fname = "./data/ways.csv";
  std::string fname2 = "./data/traj.csv";
  WaypointServer waypoint_server;
  std::vector<std::vector<float>> datawp = waypoint_server.load(fname, 6);
  std::vector<std::vector<float>> datawp2 = waypoint_server.load(fname2, 3);
  ros::init(argc, argv, "pp_obst");
  PurePursuit pp;
  pp.point_loader(datawp);
  pp.point_loader2(datawp2);

  ros::spin();
  return 0;
}
