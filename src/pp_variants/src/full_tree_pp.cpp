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

using namespace std;
struct PointXY
{
  float data[2];
};

class PurePursuit
{
private:
  bool center_free;
  bool dyn_obs;
  bool future_col_;
  bool left_free;
  bool right_free;
  bool stat_obs;
  float cur_vel;
  float handcrafted_vel;
  ros::NodeHandle n;
  ros::Publisher drive_pub;
  ros::Publisher leftpath_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher rightpath_pub_;
  ros::Subscriber alt_traj_sub;
  ros::Subscriber enum_sub;
  ros::Subscriber odom_sub_;
  ros::Subscriber vel_odom_sub;
  std::string source_frame;
  std::string target_frame;
  std::vector<float> ref_velocities;
  std::vector<PointXY> left_wp;
  std::vector<PointXY> ref_wp;
  std::vector<PointXY> right_wp;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;

public:
  PurePursuit()
  : odom_sub_(), vel_odom_sub(), path_pub_(), marker_pub_(),
    drive_pub(), cur_vel(), leftpath_pub_(), rightpath_pub_(),
    enum_sub(), alt_traj_sub(), handcrafted_vel(), target_frame(),
    source_frame(), tf2_listener(tf_buffer),
    center_free(), stat_obs(), dyn_obs(), left_free(), right_free()
  {
    odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
    vel_odom_sub = n.subscribe("/vesc/odom", 1, &PurePursuit::vel_callback, this);

    path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
    leftpath_pub_ = n.advertise<nav_msgs::Path>("/wps_left", 1, true);
    rightpath_pub_ = n.advertise<nav_msgs::Path>("/wps_right", 1, true);
    marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
    cur_vel = 0.0f;
    enum_sub = n.subscribe("/center_traj_free", 1, &PurePursuit::enum_callback, this);
    alt_traj_sub = n.subscribe("/alt_traj", 1, &PurePursuit::alt_traj_callback, this);
    target_frame = "car1_30m/laser";
    source_frame = "map";

    handcrafted_vel = 1.2f;
    center_free = false;
    stat_obs = false;
    dyn_obs = false;
    left_free = false;
    right_free = false;
    geometry_msgs::TransformStamped map_to_laser;
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
        float v = a[i][2] / 7.5f;
        p1.data[0] = x;
        p1.data[1] = y;
        ref_velocities.push_back(v);
        ref_wp.push_back(p1);
      }
    }
    publish_plan(ref_wp, path_pub_);
  }


  void point_visualize(std::pair<float, float> pts)
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

  void transformer(
    const std::vector<int> indices, std::vector<PointXY> pts, float * xg, float * yg,
    int * index_wp)
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
    float vel = 1.6f - angle / 30.0f;
    if (vel < 0.8f) {
      vel = 0.8;
    }
    return vel;
  }


  void driver(
    float xg, float yg, int stop = 1, int near_ind = 0,
    float maxvel = 2.0, bool refvel = true)
  {
    float L = (xg * xg) + (yg * yg);
    float angle = 0.0f;
    float velocity = 0.0f;
    if (L > 0 && stop == 1) {
      angle = 2 * yg / L;
    } else {stop = 0;}
    float kp = 0.33f;
    float error = angle;
    angle = kp * error;
    if (std::abs(angle) < 0.035f) {
      angle = 0.0f;
    }
    // used for capping velocity
    if (velocity > maxvel) {velocity = maxvel;}
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "";
    drive_msg.drive.steering_angle = angle * stop;
    if (refvel) {
      velocity = ref_velocities[near_ind];
    } else {
      velocity = get_velocity(angle);
    }
    // used for capping velocity
    if (velocity > maxvel) {
      velocity = maxvel;
    }
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

  int nearest_wp(std::vector<int> indices, float x, float y)
  {
    float minD = 1000.0f;
    int idx = 0;
    for (int i = 0; i < indices.size(); ++i) {
      float xi = ref_wp[indices[i]].data[0];
      float yi = ref_wp[indices[i]].data[1];
      float dis = (x - xi) * (x - xi) + (y - yi) * (y - yi);
      if (dis < minD) {
        minD = dis;
        idx = indices[i];
        if (minD < 0.15f) {return idx;}
      }
    }
    return idx;
  }

  void track_given_traj(
    std::vector<PointXY> refs, float x, float y, float maxvel = 2.0f,
    bool refvel = true)
  {
    // dynamic horizon longer when center free and smaller when center block
    float r = std::min(std::max(0.5f, 1.8f * cur_vel / 1.8f), 1.8f);
    if (!refvel) {r = std::min(std::max(0.7f, 1.6f * cur_vel / 1.8f), 1.6f);}
    int near_ind = 0;
    std::vector<int> indices = ball_query(refs, x, y, r);
    if (indices.size() > 0) {
      float xg, yg;
      int ind;
      transformer(indices, refs, &xg, &yg, &ind);
      point_visualize(std::make_pair(refs[indices[ind]].data[0], refs[indices[ind]].data[1]));
      if (refvel) {near_ind = nearest_wp(indices, x, y);}
      driver(xg, yg, 1, near_ind, maxvel, refvel);
    }
  }

  void state_tree(float x, float y)
  {
    if (center_free) {
      track_given_traj(ref_wp, x, y, 2.0f, true);
    } else if (stat_obs) {
      if (left_free) {
        track_given_traj(left_wp, x, y, 2.0f, false);
      } else if (right_free) {
        track_given_traj(right_wp, x, y, 2.0f, false);
      } else {
        std::cout << "all blocked static" << std::endl;
        driver(0.0f, 0.0f, 0, 0);
      }
    } else if (dyn_obs) {  // not considering curved track
      float dynamic_obst_vel = 0.0;     // get this vel

      std::vector<int> indices = ball_query(ref_wp, x, y, 0.5);
      int near_ind = nearest_wp(indices, x, y);
      int ref_velocity = ref_velocities[near_ind];
      if (ref_velocity < 1.5f) {   // curved track
        std::cout << "dynamic curved follow " << std::endl;
        track_given_traj(ref_wp, x, y, dynamic_obst_vel, true);     // cap n follow
      } else {   // straight track
        if (dynamic_obst_vel > handcrafted_vel * cur_vel) {
          std::cout << "dynamic straight follow" << std::endl;
          track_given_traj(ref_wp, x, y, dynamic_obst_vel, true);     // cap n follow
        } else if (left_free) {
          std::cout << "dynamic overtake left free" << std::endl;
          track_given_traj(left_wp, x, y, 2.0f, false);       // overtake
        } else if (right_free) {
          std::cout << "dynamic overtake right free" << std::endl;
          track_given_traj(right_wp, x, y, 2.0f, false);     // overtake
        } else {
          std::cout << "dynamic all block follow" << std::endl;
          track_given_traj(ref_wp, x, y, dynamic_obst_vel, true);      // cap n follow
        }
      }
    } else {driver(0.0f, 0.0f, 0, 0);}
  }

  void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_msg)
  {
    float x = pose_msg->pose.pose.position.x;
    float y = pose_msg->pose.pose.position.y;
    state_tree(x, y);
  }

  void vel_callback(const nav_msgs::Odometry::ConstPtr & poser)
  {
    cur_vel = poser->twist.twist.linear.x;
  }

  void enum_callback(const std_msgs::Int8::ConstPtr & enum_msg)
  {
    center_free = false;
    stat_obs = false;
    dyn_obs = false;
    int inp = enum_msg->data;
    if (inp == 0) {center_free = true;}
    if (inp == 1) {stat_obs = true;}
    if (inp == 2) {dyn_obs = true;}
  }

  void alt_traj_callback(const std_msgs::Int8::ConstPtr & enum_msg)
  {
    left_free = false;
    right_free = false;
    int inp = enum_msg->data;
    if (inp == 1) {left_free = true;}
    if (inp == 2) {right_free = true;}
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
  ros::init(argc, argv, "full_tree_pp");
  PurePursuit pp;
  pp.point_loader(datawp);
  pp.point_loader2(datawp2);
  ros::spin();
  return 0;
}
