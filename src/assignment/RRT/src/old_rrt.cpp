#include "rrt/rrt.h"
// Code makes the car rotate continuosly in the map using RRT as a local planner to avoid obstacles.
// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    std::string pose_topic, scan_topic;
    pose_topic = "/pf_pose";
    scan_topic = "/scan"; 
    //nh_.getParam("pose_topic", pose_topic);
    //nh_.getParam("scan_topic", scan_topic);
    //params in cm or degree
    GRID_RESOLUTION = 0.2;
    GRID_WIDTH = 4;
    GRID_HEIGHT = GRID_WIDTH;
    STEER_LIMIT_LEFT = 180;
    STEER_LIMIT_RIGHT = -180;
    ITERATIONS = 1000;
    GRID_SIZE = int(GRID_WIDTH *  GRID_HEIGHT /  (GRID_RESOLUTION * GRID_RESOLUTION));
    ORIGIN.position.x = 0;
    ORIGIN.position.y = int(-GRID_WIDTH/(2));
    ORIGIN.orientation.w = 1.0;
    init_state = true;
    gen.seed(4294967295);
    std::uniform_real_distribution<double> x_dist (0.0, 1.0);
    std::uniform_real_distribution<double> y_dist (0.0, 1.0);
    //x_dist (0.0, double(GRID_WIDTH/GRID_RESOLUTION));
    //y_dist (0.0, double(GRID_WIDTH/GRID_RESOLUTION));
    // ROS publishers
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
    grid_pub = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);
    //path_pub = nh_.advertise<>
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    occ_grid.header.frame_id = "/laser";
    occ_grid.info.resolution = GRID_RESOLUTION;
    occ_grid.info.width = int(GRID_WIDTH/GRID_RESOLUTION);
    occ_grid.info.height = int(GRID_WIDTH/GRID_RESOLUTION);
    occ_grid.info.origin = ORIGIN;
    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
 
    float x =  -ORIGIN.position.y/GRID_RESOLUTION;
    float y =  ORIGIN.position.x;
    int m[GRID_SIZE];
    int rmax = int(floor(GRID_WIDTH/GRID_RESOLUTION)); //max range
    float adjust = 1.5;

    for (int i = 0; i <  rmax; i++)
    {
        for (int j = 0; j <  rmax; j++)
        {
            float r = sqrt(pow((x - i),2) + pow((y - j),2));
            float theta = ((atan2((y-j),(x-i)))*180/3.14) -90;
            int index = int(fabs(theta*3));
            m[j +  rmax*i] = 50;
            if (index < 1080){
                float meas_r = (scan_msg->ranges[index])/GRID_RESOLUTION;
                if (r > (meas_r + adjust)){m[j +  rmax*i] = 50;}
                if (fabs(r - meas_r) < adjust){m[j +  rmax*i] = 90;}
                if (r < meas_r){m[j +  rmax*i] = 10;}}
        }
    }
    std::vector<signed char> a(m, m+GRID_SIZE);
    occ_grid.data = a;
    grid_pub.publish(occ_grid);  
}


void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    std::vector<Node> tree;
    float x = pose_msg->pose.position.x;
    float y = pose_msg->pose.position.y;
    float goal_x = -ORIGIN.position.y/GRID_RESOLUTION;;
    float goal_y = int(GRID_WIDTH/GRID_RESOLUTION);
    bool goal_found = false;
    int counter = 0;
    if (init_state)
    {
        init_state = false;
        Node new_node;
        new_node.x = -ORIGIN.position.y/GRID_RESOLUTION;
        new_node.y = ORIGIN.position.x;
        new_node.cost = 0;
        new_node.parent = 0;
        new_node.is_root = true;
        tree.push_back(new_node);
        std::cout<<"initialized "<<x <<y<<std::endl;
    }
    while(!goal_found)
    {   
        counter++;
        std::vector<double> sampled_point = sample();
        int nearest_node_index = nearest(tree, sampled_point);
        
        //Node new_node = steer(tree[nearest_node_index], sampled_point, nearest_node_index);
        /*if (new_node.cost > 0)
        {
          goal_found = is_goal(new_node, goal_x, goal_y);
          new_node.cost = new_node.cost + tree[new_node.parent].cost;
          tree.push_back(new_node);
          std::cout<<tree.size()<<std::endl;
          if(is_goal(new_node, goal_x, goal_y))
          {
            std::vector<Node> Path = find_path(tree, new_node);
            std::cout<<" ##########################path found  "<< Path.size() << std::endl;
            init_state = true;

          }
        }*/
        if (counter > ITERATIONS) {break;}
    }
    

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // TESTED
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    float rmax = floor(GRID_WIDTH/GRID_RESOLUTION);
    double x = rmax * x_dist(gen);
    double y = rmax * y_dist(gen);
    sampled_point.push_back(x);
    sampled_point.push_back(y);
    //std::cout<<x<<"  and y is "<<y<<std::endl;
    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
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
    for (int i = 0; i < tree.size(); i++)
    {
        float d = fabs(tree[i].x - xi) + fabs(tree[i].y - yi);
        if (d < min) 
            {min = d;
            nearest_node = i;}
    }

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point, int &index) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

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
    float threshold = 1.5/GRID_RESOLUTION;
    if ((fabs(xi - sampled_point[0]) < threshold) && (fabs(yi - sampled_point[1]) < threshold))
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
        new_node.parent = index;
        new_node.cost = -1;
        float cost_ = 1 ;//line_cost(nearest_node, new_node);
        new_node.cost = cost_;
    }
    else
    {
        float theta = atan2((yi - sampled_point[1]), (xi - sampled_point[0]));
        std::vector<double> new_point;
        new_point.push_back(xi + threshold*0.5*cos(theta));
        new_point.push_back(yi + threshold*0.5*sin(theta));
        steer(nearest_node, new_point, index);
    }

    return new_node;
}


bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal
    float goal_threshold = 2/GRID_RESOLUTION;
    bool close_enough = false;
    float d = fabs(latest_added_node.x - goal_x) + fabs(latest_added_node.y - goal_y);
    if (d < goal_threshold){close_enough = true;}

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    found_path.push_back(latest_added_node);
    int i = latest_added_node.parent;
    //assuming zero index is current pose
    while(i > 0)
    {
        found_path.insert(found_path.begin(), tree[i]);
        i = tree[i].parent;
        if (i == 0){break;}
    }

    return found_path;
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
    if (occ_grid.data[x_new + rmax*y_new] > 20)
        {collision = true;}
    /*else
    {
        for (int i = 0; i <  rmax; i++)
    {
        for (int j = 0; j <  rmax; j++)
        {
            float d = sqrt(pow((x_new - j),2) + pow((y_new - i),2));
            if (d < radius)
            {
                if (occ_grid.data[j +  rmax*i] > 10){collision = true;}
            }
        }
    }
    }*/
    /*else
    {
        int n = ceil(d / radius) + 1;
        for (int k = 1; k < n; k++)
        {
            float xe = xc + k*radius*cos(slope);
            float ye = yc + k*radius*sin(slope);
            for (int i = 0; i <  rmax; i++)
            {
                for (int j = 0; j <  rmax; j++)
                {
                    float d = sqrt(pow((xe - j),2) + pow((ye - i),2));
                    if (d < radius)
                    {
                        if (occ_grid.data[j +  rmax*i] > 10){collision = true;}
                    }
                }
            }
        }
    }
    */

    return collision;
}
// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = -1;
    std::vector<double> point;
    point.push_back(node.x);
    point.push_back(node.y);
    int index = nearest(tree, point);
    bool collision = check_collision(tree[index], node);
    if (collision) {return cost;}
    float xc = tree[index].x;
    float yc = tree[index].y;
    cost = fabs(xc - node.x) + fabs(yc - node.y);
    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = -1;
    bool collision = check_collision(n1, n2);
    if (collision) {
        std::cout << cost <<std::endl;
        return cost;}
    cost = fabs(n1.x - n2.x) + fabs(n2.y -  n1.y);
    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    float threshold = 1.5/GRID_RESOLUTION;
    float xi = node.x;
    float yi = node.y;
    for (int i = 0; i < tree.size(); i++)
    {
        if ((fabs(tree[i].x - xi) + fabs(tree[i].y - yi)) < threshold)
            {neighborhood.push_back(i);}
    }
    return neighborhood;
}