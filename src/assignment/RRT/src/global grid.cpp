#include "rrt/rrt.h"
// Code makes the car rotate continuously in the map using
// RRT as a local planner to avoid obstacles.
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
  std::string pose_topic, scan_topic, frame_occ;
  int ORIGINx, ORIGINy;
  nh_.getParam("pose_topic", pose_topic);
  nh_.getParam("scan_topic", scan_topic);
  nh_.getParam("GRID_RESOLUTION", GRID_RESOLUTION);
  nh_.getParam("GRID_WIDTH", GRID_WIDTH);
  nh_.getParam("GRID_HEIGHT", GRID_HEIGHT);
  nh_.getParam("ITERATIONS", ITERATIONS);
  nh_.getParam("ORIGINx", ORIGINx);
  nh_.getParam("ORIGINy", ORIGINy);
  nh_.getParam("frame", frame_occ);

  STEER_LIMIT_LEFT = 180;
  STEER_LIMIT_RIGHT = -180;
  GRID_SIZE = 1 + static_cast<int>(GRID_WIDTH * GRID_HEIGHT / (GRID_RESOLUTION * GRID_RESOLUTION));
  ORIGIN.position.x = ORIGINx;
  ORIGIN.position.y = ORIGINy;
  ORIGIN.orientation.w = 1.0;
  init_state = true;
  gen.seed(4294967295);
  std::uniform_real_distribution<double> x_dist(0.0, 0.25);
  std::uniform_real_distribution<double> y_dist(0.0, 0.25);

  // ROS publishers
  drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
  grid_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("shapes", 2);
  scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
  occ_grid.header.frame_id = frame_occ;
  occ_grid.info.resolution = GRID_RESOLUTION;
  occ_grid.info.width = static_cast<int>(GRID_WIDTH / GRID_RESOLUTION);
  occ_grid.info.height = static_cast<int>(GRID_HEIGHT / GRID_RESOLUTION);
  occ_grid.info.origin = ORIGIN;
  ROS_INFO("Created new RRT Object.");
  for (int z = 0; z < GRID_SIZE; ++z) {
    occ_data.push_back(50);
  }
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg)
{
  // add a wait for message fun to odom. get x, y ,yaw (quat to eul) add yaw to theta.
  boost::shared_ptr<geometry_msgs::PoseStamped const> poseptr;
  geometry_msgs::PoseStamped curr_pose;
  poseptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pf_pose", nh_);
  if (poseptr != NULL) {curr_pose = *poseptr;}
  float curr_x = curr_pose.pose.position.x;
  float curr_y = curr_pose.pose.position.y;
  tf::Quaternion q(
    curr_pose.pose.orientation.x,
    curr_pose.pose.orientation.y,
    curr_pose.pose.orientation.z,
    curr_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = yaw * 180 / 3.14159;
  float x = (curr_y - ORIGIN.position.y) / GRID_RESOLUTION;
  float y = (curr_x - ORIGIN.position.x) / GRID_RESOLUTION;
  int rmax = 15;
  int rmax_global = static_cast<int>(GRID_WIDTH / GRID_RESOLUTION);
  float adjust = 1.5 / GRID_RESOLUTION;
  for (int i = x - rmax; i < x + rmax; i++) {
    for (int j = y - rmax; j < y + (rmax); j++) {
      int ind_ = j + rmax_global * i;
      if (ind_ < GRID_SIZE && occ_data[ind_] > 10) {
        float r = sqrt(pow((x - i), 2) + pow((y - j), 2));
        float theta = ((atan2((y - j), (x - i))) * 180 / 3.14) - 90 + yaw;
        int index = static_cast<int>(fabs(theta * 3));
        if (index < 1080) {
          float meas_r = (scan_msg->ranges[index]) / GRID_RESOLUTION;
          if (fabs(r - meas_r) < adjust) {occ_data[ind_] = 90;}
          if (r < meas_r) {occ_data[ind_] = 10;}
        }
      }
    }
  }
  int occ_[GRID_SIZE];
  for (int p = 0; p < GRID_SIZE; ++p) {
    occ_[p] = occ_data[p];
  }
  std::vector<signed char> a(occ_, occ_ + GRID_SIZE);
  occ_grid.data = a;
  grid_pub.publish(occ_grid);
}
