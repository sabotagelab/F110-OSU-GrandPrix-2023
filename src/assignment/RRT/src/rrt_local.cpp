#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT()
{
  // Do something in here, free up used memory, print message, etc.
  ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle & nh)
: nh_(nh), gen((std::random_device())())
{
  std::string pose_topic, scan_topic;
  pose_topic = "/pf_pose";
  scan_topic = "/scan";
  GRID_RESOLUTION = 0.2;
  GRID_WIDTH = 4;
  GRID_HEIGHT = GRID_WIDTH;
  STEER_LIMIT_LEFT = 180;
  STEER_LIMIT_RIGHT = -180;
  ITERATIONS = 50;
  GRID_SIZE = 1 + static_cast<int>(GRID_WIDTH * GRID_HEIGHT / (GRID_RESOLUTION * GRID_RESOLUTION));
  ORIGIN.position.x = 0;
  ORIGIN.position.y = 0;
  ORIGIN.orientation.w = 1.0;
  init_state = true;
  gen.seed(4294967295);
  std::uniform_real_distribution<double> x_dist(0.0, 0.25);
  std::uniform_real_distribution<double> y_dist(0.0, 0.25);
  // ROS publishers
  drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
  grid_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("shapes", 2);
  pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
  scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

  occ_grid.header.frame_id = "/laser";
  occ_grid.info.resolution = GRID_RESOLUTION;
  occ_grid.info.width = static_cast<int>(GRID_WIDTH / GRID_RESOLUTION);
  occ_grid.info.height = static_cast<int>(GRID_WIDTH / GRID_RESOLUTION);
  occ_grid.info.origin = ORIGIN;
  ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg)
{
  float x = -ORIGIN.position.y / GRID_RESOLUTION;
  float y = ORIGIN.position.x;
  int m[GRID_SIZE];
  int rmax = static_cast<int>(GRID_WIDTH / GRID_RESOLUTION);
  float adjust = 2.0 / GRID_RESOLUTION;

  for (int i = 0; i < rmax; i++) {
    for (int j = 0; j < rmax; j++) {
      float r = sqrt(pow((x - i), 2) + pow((y - j), 2));
      float theta = ((atan2((y - j), (x - i))) * 180 / 3.14) - 90;
      int index = static_cast<int>(fabs(theta * 3));
      m[j + rmax * i] = 90;
      if (index < 1080) {
        float meas_r = (scan_msg->ranges[index]) / GRID_RESOLUTION;
        if (r > (meas_r + adjust)) {m[j + rmax * i] = 90;}
        if (fabs(r - meas_r) < adjust) {m[j + rmax * i] = 90;}
        if (r < meas_r) {m[j + rmax * i] = 10;}}
    }
  }
  std::vector<signed char> a(m, m + GRID_SIZE);
  occ_grid.data = a;
  grid_pub.publish(occ_grid);
}


void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr & pose_msg)
{
  // The pose callback when subscribed to particle filter's inferred pose
  // The RRT main loop happens here
  // Args:
  //    pose_msg (*PoseStamped): pointer to the incoming pose message
  std::vector<Node> tree;
  int counter = 0;
  if (init_state) {
    init_state = false;
    Node new_node;
    new_node.x = -ORIGIN.position.y / GRID_RESOLUTION;
    new_node.y = ORIGIN.position.x;
    new_node.cost = 0;
    new_node.parent = 0;
    new_node.is_root = true;
    tree.push_back(new_node);
    std::cout << "initialized " << std::endl;
  }
  while (counter <= ITERATIONS) {
    std::vector<double> sampled_point = sample();
    int nearest_node_index = nearest(tree, sampled_point);
    Node new_node = steer(tree[nearest_node_index], sampled_point, nearest_node_index);
    if (new_node.cost > 0) {
      counter++;
      tree.push_back(new_node);
      if (tree.size() > 3) {
        drive_me(tree);
        init_state = true;
        break;
      }
    }
  }
}

std::vector<double> RRT::sample()
{
  // This method returns a sampled point from the free space
  // You should restrict so that it only samples a small region
  // of interest around the car's current position
  // FIX THIS FUNCTION generating points out of the range far away
  // Returns:
  //     sampled_point (std::vector<double>): the sampled point in free space
  std::vector<double> sampled_point;
  float threshold = 1.5;
  int rmax = static_cast<int>(floor(GRID_WIDTH / GRID_RESOLUTION));
  double x = x_dist(gen) * rmax;
  double y = y_dist(gen) * rmax;
  if (occ_grid.data[static_cast<int>(y + rmax * x)] > 20){
    sample();
    }
  sampled_point.push_back(x);
  sampled_point.push_back(y);
  return sampled_point;
}


int RRT::nearest(std::vector<Node> & tree, std::vector<double> & sampled_point)
{
  // This method returns the nearest node on the tree to the sampled point
  // Args: TESTED
  //     tree (std::vector<Node>): the current RRT tree
  //     sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //     nearest_node (int): index of nearest node on the tree
  int min = 10000;
  int nearest_node = 0;
  float xi = sampled_point[0];
  float yi = sampled_point[1];
  for (int i = 0; i < tree.size(); i++) {
    float d = fabs(tree[i].x - xi) + fabs(tree[i].y - yi);
    if (d < min) {
      min = d;
      nearest_node = i;
    }
  }
  return nearest_node;
}

Node RRT::steer(Node & nearest_node, std::vector<double> & sampled_point, int & index)
{
  // The function steer:(x,y)->z returns a point such that z is “closer”
  // to y than x is. The point z returned by the function steer will be
  // such that z minimizes ||z−y|| while at the same time maintaining
  // ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

  // basically, expand the tree towards the sample point (within a max dist)

  // Args: Tested
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //    new_node (Node): new node created from steering

  Node new_node;
  new_node.cost = -1;
  float xi = nearest_node.x;
  float yi = nearest_node.y;
  float threshold = 1 / GRID_RESOLUTION;
  if ((fabs(xi - sampled_point[0]) < threshold) && (fabs(yi - sampled_point[1]) < threshold)) {
    new_node.x = sampled_point[0];
    new_node.y = sampled_point[1];
    new_node.parent = index;
    new_node.cost = -1;
    bool col = check_collision(nearest_node, new_node);
    if (!col) {
      new_node.cost = 1;
    }
  }
  return new_node;
}

void RRT::drive_me(std::vector<Node> & tree)
{
  visualizer(tree);
  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = "laser";
  float angle = atan2(tree[1].y, (tree[1].x - (ORIGIN.position.y / GRID_RESOLUTION)));
  std::cout << (angle * 180 / 3.14) << std::endl;
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.speed = 0.5;
  drive_pub.publish(drive_msg);
}

void RRT::visualizer(std::vector<Node> & tree)
{
  std::vector<geometry_msgs::Point> points_list;
  visualization_msgs::Marker points;
  points.header.stamp = ros::Time::now();
  points.header.frame_id = "laser";
  points.ns = "markers";
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = 0.3;
  points.scale.y = 0.3;
  points.color.b = 1.0;
  points.color.a = 1.0;
  for (int i = 1; i < tree.size(); ++i) {
    geometry_msgs::Point pt;
    pt.y = (tree[i].x + ORIGIN.position.y / GRID_RESOLUTION ) * GRID_RESOLUTION;
    pt.x = GRID_RESOLUTION * (tree[i].y);
    pt.z = 0;
    points_list.push_back(pt);
  }
  points.points = points_list;
  marker_pub.publish(points);
}


bool RRT::check_collision(Node & nearest_node, Node & new_node)
{
  // This method returns a boolean indicating if the path between the
  // nearest node and the new node created from steering is collision free
  // Args:
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    new_node (Node): new node created from steering
  // Returns:
  //    collision (bool): true if in collision, false otherwise

  bool collision = false;
  float radius = 1.0;
  float x_new = new_node.x;
  float y_new = new_node.y;
  float xc = nearest_node.x;
  float yc = nearest_node.y;
  float slope = atan2((yc - y_new), (xc - x_new));
  float d = sqrt(pow((xc - x_new), 2) + pow((yc - y_new), 2));
  int rmax = static_cast<int>(GRID_WIDTH / GRID_RESOLUTION);  // max range

  if (occ_grid.data[y_new + rmax * x_new] > 20)
  {collision = true;}
  if (fabs(slope) > 3){
    collision = true;
    } else {
    if (d < 2 * radius) {
      for (int i = 0; i < rmax; i++) {
        for (int j = 0; j < rmax; j++) {
          float d = sqrt(pow((x_new - j), 2) + pow((y_new - i), 2));

          if (occ_grid.data[j + rmax * i] > 20) {collision = true;}
        }
      }
    } else {
      int n = ceil(d / (radius)) + 1;
      for (int k = 1; k < n; k++) {
        float xe = xc + k * radius * cos(slope);
        float ye = yc + k * radius * sin(slope);
        for (int i = 0; i < rmax; i++) {
          for (int j = 0; j < rmax; j++) {
            float d = sqrt(pow((xe - j), 2) + pow((ye - i), 2));
            if (d < 2 * radius) {
              if (occ_grid.data[j + rmax * i] > 10) {collision = true;}
            }
          }
        }
      }
    }
  }


  return collision;
}
