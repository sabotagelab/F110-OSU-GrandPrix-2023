#include "rrt/rrt.h"
// Code makes the car rotate continuosly in the map using RRT as a local planner to avoid obstacles.
// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    std::string pose_topic, scan_topic, frame_occ;
    int ORIGINx, ORIGINy;
    nh_.getParam("pose_topic", pose_topic); //goal pose
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
    GRID_SIZE = 1 + int(GRID_WIDTH *  GRID_HEIGHT /  (GRID_RESOLUTION * GRID_RESOLUTION));
    ORIGIN.position.x = ORIGINx;
    ORIGIN.position.y = ORIGINy; //int(-GRID_WIDTH/(2));
    ORIGIN.orientation.w = 1.0;
    init_state = true;
    gen.seed(4294967295);
    std::uniform_real_distribution<double> x_dist (0.0, 1.0);
    std::uniform_real_distribution<double> y_dist (0.0, 1.0);
    // ROS publishers
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
    grid_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("shapes", 2);
    //path_pub = nh_.advertise<>
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    occ_grid.header.frame_id = frame_occ;
    occ_grid.info.resolution = GRID_RESOLUTION;
    occ_grid.info.width = int(GRID_WIDTH/GRID_RESOLUTION);
    occ_grid.info.height = int(GRID_HEIGHT/GRID_RESOLUTION);
    occ_grid.info.origin = ORIGIN;
    ROS_INFO("Created new RRT Object.");
    for (int z = 0; z < GRID_SIZE; ++z)
    {
        occ_data.push_back(50);
    }

}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // add a wait for message fun to odom. get x, y ,yaw (quat to eul) add yaw to theta.
    //int occ_data[GRID_SIZE];
    boost::shared_ptr<geometry_msgs::PoseStamped const> poseptr;
    geometry_msgs::PoseStamped curr_pose;
    poseptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pf_pose", nh_);
    if (poseptr != NULL){curr_pose = *poseptr;}
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
    yaw = yaw*180/3.14159;
    float x = (curr_y - ORIGIN.position.y)/GRID_RESOLUTION; //-3/GRID_RESOLUTION; //10
    float y = (curr_x - ORIGIN.position.x)/GRID_RESOLUTION;
    int rmax = 15; //int(-2*y); //max range
    int rmax_global = int(GRID_WIDTH/GRID_RESOLUTION);
    float adjust = 0.9/GRID_RESOLUTION;
    for (int i = x-rmax; i <  x+rmax; i++)
    {
        for (int j = y-rmax; j <  y+(rmax); j++)
        {
            int ind_ = j+rmax_global*i;
            if( ind_ < GRID_SIZE && occ_data[ind_]>10)
            {
                float r = sqrt(pow((x - i),2) + pow((y - j),2));
                float theta = ((atan2((y-j),(x-i)))*180/3.14) -90 + yaw;
                int index = int(fabs(theta*3));
                if (index < 1080)
                {
                    float meas_r = (scan_msg->ranges[index])/GRID_RESOLUTION ;
                    if (r < meas_r){occ_data[ind_] = 10;}
                    if (fabs(r - meas_r) < adjust){occ_data[ind_] = 90;}
                }
            }
        }
    }
    int occ_[GRID_SIZE];
    for (int p = 0; p < GRID_SIZE; ++p)
    {
        occ_[p] = occ_data[p];
    }
    std::vector<signed char> a(occ_, occ_+GRID_SIZE);
    occ_grid.data = a;
    grid_pub.publish(occ_grid);  
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    init_state = true;
    std::vector<Node> tree;
    int count = 0;
    std::cout<<"goal is set X: "<<pose_msg->pose.position.x << " and Y: "<<pose_msg->pose.position.y <<std::endl;
    boost::shared_ptr<geometry_msgs::PoseStamped const> poseptr;
    geometry_msgs::PoseStamped curr_pose;
    poseptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pf_pose", nh_);
    if (poseptr != NULL){curr_pose = *poseptr;}
    float curr_x = curr_pose.pose.position.x;
    float curr_y = curr_pose.pose.position.y;
    float x = (curr_y - ORIGIN.position.y)/GRID_RESOLUTION;
    float y = (curr_x - ORIGIN.position.x)/GRID_RESOLUTION;
    int rmax = 15; //int(-2*y); //max range
    int rmax_global = int(GRID_WIDTH/GRID_RESOLUTION);
    if (init_state)
    {
        init_state = false;
        Node new_node;
        new_node.x = x;
        new_node.y = y;
        new_node.cost = 0;
        new_node.parent = 0;
        new_node.is_root = true;
        tree.push_back(new_node);
        std::cout<<"initialized "<<std::endl;
    }
    while(count <= ITERATIONS){
        bool sample_bad = true;
        std::vector<double> good_point;
        while(sample_bad){
            std::vector<double> sampled_point = sample();
            float newx = tree[count].x -rmax + sampled_point[0];
            float newy = tree[count].y -rmax + sampled_point[1];
            int index = int(newy + rmax_global*newx);
            if (occ_data[index] < 20) 
            {
                float steer_ang = (atan2((tree[count].y - newy),(tree[count].x - newx)))*180/3.1415;
                if (fabs(steer_ang) < STEER_LIMIT_LEFT){
                    std::cout<<newx <<"  and new y is "<<newy<<std::endl;
                    sample_bad = false;
                    count = count +1;
                    good_point.push_back(newx);
                    good_point.push_back(newy);
                }    
            }
        }
        int nearest_node_index = nearest(tree, good_point);
        Node next_node = steer(tree[nearest_node_index], good_point, nearest_node_index);
        tree.push_back(next_node);
        float gx = (pose_msg->pose.position.y - ORIGIN.position.y)/GRID_RESOLUTION;
        float gy = (pose_msg->pose.position.x - ORIGIN.position.x)/GRID_RESOLUTION;
        bool goal_found = is_goal(next_node, gx, gy);
        if (goal_found){
            std::vector<Node> Path = find_path(tree, next_node);
            std::cout<<"path size "<< Path.size()<<std::endl;
            Node goal_node;
            goal_node.x = gx;
            goal_node.y = gy;
            goal_node.cost = 0;
            goal_node.parent = tree.size()-1;
            tree.push_back(goal_node);
            visualizer(tree, sample_bad);
            bool isPath = true;
            visualizer(Path, isPath);
            break;
        }
    }
}
    
std::vector<double> RRT::sample() {
    std::vector<double> sampled_point;
    float threshold = 1.5;
    int rmax = 30;//int(floor(GRID_WIDTH/GRID_RESOLUTION)); //max range
    double x = x_dist(gen) * rmax;
    double y = y_dist(gen) * rmax;
    sampled_point.push_back(x);
    sampled_point.push_back(y);
    //std::cout<<x<<"  and y is "<<y<<std::endl;
    return sampled_point;
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    int min = 10000;
    int nearest_node = 0;
    float xi = sampled_point[0];
    float yi = sampled_point[1];
    for (int i = 0; i < tree.size(); i++)
    {
        float d = fabs(tree[i].x - xi) + fabs(tree[i].y - yi);
        if (d < min) 
            {min = d;
            nearest_node = i;}
    }
    std::cout<<"index nearest  "<<nearest_node <<std::endl;
    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point, int &index) {
    Node new_node;
    new_node.x = sampled_point[0];
    new_node.y = sampled_point[1];
    new_node.parent = index;
    new_node.cost = -1;
    /*bool col = check_collision(nearest_node, new_node);
    if (!col)
    {
        new_node.cost = 1;
    }*/
    return new_node;
}

void RRT::visualizer(std::vector<Node> &tree, bool &isPath){
    std::vector<geometry_msgs::Point> pointslist;
    visualization_msgs::Marker points;
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "map";
    points.ns = "markers";
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x=0.08;
    points.color.b = 1.0;
    points.color.a = 1.0;
    if (isPath){
        points.ns = "lines";
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.color.b = 0.0;
        points.color.r = 1.0;
        points.id = 1;
           }
    for (int i = 0; i < tree.size(); ++i)
    {
       geometry_msgs::Point pt;
       pt.y = (tree[i].x * GRID_RESOLUTION) + ORIGIN.position.y;
       pt.x = (tree[i].y * GRID_RESOLUTION) + ORIGIN.position.x;
       pt.z = 0;
       pointslist.push_back(pt);
       if(!isPath){
        int b = tree[i].parent;
       pt.y = (tree[b].x * GRID_RESOLUTION) + ORIGIN.position.y;
       pt.x = (tree[b].y * GRID_RESOLUTION) + ORIGIN.position.x;
       pointslist.push_back(pt);
       }
    }
    points.points = pointslist;
    marker_pub.publish(points);
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    float goal_threshold = 2/GRID_RESOLUTION;
    bool close_enough = false;
    float d = fabs(latest_added_node.x - goal_x) + fabs(latest_added_node.y - goal_y);
    if (d < goal_threshold){close_enough = true;}

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    std::vector<Node> found_path;
    found_path.push_back(latest_added_node);
    int i = latest_added_node.parent;
    //assuming zero index is current pose
    while(i > 0)
    {
        if (i == 0){break;}
        found_path.insert(found_path.begin(), tree[i]);
        i = tree[i].parent;
    }

    return found_path;
}

/*




void RRT::drive_me(std::vector<Node> &tree){
    visualizer(tree);
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "laser";
    float angle = atan2(tree[1].y , (tree[1].x - (ORIGIN.position.y/GRID_RESOLUTION)));
    std::cout<<(angle*180/3.14)<<std::endl;
    drive_msg.drive.steering_angle = angle;
    drive_msg.drive.speed = 0.5;
    drive_pub.publish(drive_msg); 
}


bool RRT::check_collision(Node &nearest_node, Node &new_node) {
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
    float d = sqrt(pow((xc - x_new),2) + pow((yc - y_new),2));
    int rmax = int(GRID_WIDTH/GRID_RESOLUTION); //max range
    
    //if (d< 2*radius) //point is too close so checking only one circle
    if (occ_grid.data[y_new + rmax*x_new] > 20)
        {collision = true;}
    if (fabs(slope) > 3) {collision = true;} //#######################################
    else
    {
        if (d < 2*radius)
        {
           for (int i = 0; i <  rmax; i++)
            {
                for (int j = 0; j <  rmax; j++)
                {
                    float d = sqrt(pow((x_new - j),2) + pow((y_new - i),2));
                    
                    if (occ_grid.data[j +  rmax*i] > 20){collision = true;}
                }
            }
        }
        else
        {
            int n = ceil(d / (radius)) + 1;
            for (int k = 1; k < n; k++)
            {
                float xe = xc + k*radius*cos(slope);
                float ye = yc + k*radius*sin(slope);
                for (int i = 0; i <  rmax; i++)
                {
                    for (int j = 0; j <  rmax; j++)
                    {
                        float d = sqrt(pow((xe - j),2) + pow((ye - i),2));
                        if (d < 2*radius)
                        {
                            if (occ_grid.data[j +  rmax*i] > 10){collision = true;}
                        }
                    }
                }
            }
        }
    }
    

    return collision;
}
*/