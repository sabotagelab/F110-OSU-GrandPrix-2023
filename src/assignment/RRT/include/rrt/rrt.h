// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// ros
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// standard
#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_map>
#include <boost/algorithm/string.hpp>


// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node
{
  double x, y;
  double cost;   // only used for RRT*
  int parent;   // index of parent node in the tree vector
  bool is_root = false;
} Node;


class RRT {
public:
  explicit RRT(ros::NodeHandle & nh);
  virtual ~RRT();

private:
  ros::NodeHandle nh_;

  ros::Publisher drive_pub;
  ros::Publisher marker_pub;
  ros::Publisher grid_pub;
  ros::Subscriber pf_sub_;
  ros::Subscriber scan_sub_;

  // tf stuff
  tf::TransformListener listener;

  float GRID_RESOLUTION;
  int GRID_WIDTH;
  int GRID_HEIGHT;
  int STEER_LIMIT_LEFT;
  int STEER_LIMIT_RIGHT;
  int ITERATIONS;
  int GRID_SIZE;

  geometry_msgs::Pose ORIGIN;
  nav_msgs::OccupancyGrid occ_grid;
  bool init_state;
  std::vector < int > occ_data;

  // random generator, use this
  std::mt19937 gen;
  std::uniform_real_distribution < > x_dist;
  std::uniform_real_distribution < > y_dist;


  // callbacks
  // where rrt actually happens
  void pf_callback(const geometry_msgs::PoseStamped::ConstPtr & pose_msg);
  // updates occupancy grid
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg);

  // RRT methods
  std::vector < double > sample();
  int nearest(std::vector < Node > & tree, std::vector < double > & sampled_point);
  Node steer(Node & nearest_node, std::vector < double > & sampled_point, int & index);
  bool check_collision(Node & nearest_node, Node & new_node);
  bool is_goal(Node & latest_added_node, double goal_x, double goal_y);
  std::vector < Node > find_path(std::vector < Node > &tree, Node & latest_added_node);
  void drive_me(std::vector < Node > & tree);
  void visualizer(std::vector < Node > & tree, bool & isPath);
  // RRT* methods
  double cost(std::vector < Node > & tree, Node & node);
  double line_cost(Node & n1, Node & n2);
  std::vector < int > near(std::vector < Node > &tree, Node & node);
  // int occ_data[];
};
