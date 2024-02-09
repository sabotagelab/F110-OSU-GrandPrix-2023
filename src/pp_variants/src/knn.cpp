// #include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <nabo/nabo.h>

// class NearestNeighborsNode {
// public:
//     NearestNeighborsNode() : nh_("~") {
//         path_sub_ = nh_.subscribe("/path", 1, &NearestNeighborsNode::pathCallback, this);
//         waypoint_sub_ = nh_.subscribe("/waypoint", 1, &NearestNeighborsNode::waypointCallback, this);
//         nn_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/nearest_neighbors", 1);
//         nh_.param<double>("search_radius", search_radius_, 1.0);
//     }

//     void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
//         // Convert path message to Eigen matrix
//         Eigen::MatrixXd mat(msg->poses.size(), 3);
//         for (size_t i = 0; i < msg->poses.size(); ++i) {
//             mat(i, 0) = msg->poses[i].pose.position.x;
//             mat(i, 1) = msg->poses[i].pose.position.y;
//             mat(i, 2) = msg->poses[i].pose.position.z;
//         }
//         // Build k-d tree from path
//         nabo::NNSearchF* tree = nabo::NNSearchF::createKDTreeLinearHeap(mat);

//         // Find nearest neighbors to waypoint
//         Eigen::Vector3f query(waypoint_.x, waypoint_.y, waypoint_.z);
//         std::vector<int> indices;
//         std::vector<float> distances;
//         tree->kNN(query, 5, indices, distances, search_radius_, Nabo::NNSearchF::ALLOW_SELF_MATCH);

//         // Build marker array of nearest neighbors
//         visualization_msgs::MarkerArray markers;
//         for (size_t i = 0; i < indices.size(); ++i) {
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = msg->header.frame_id;
//             marker.header.stamp = ros::Time::now();
//             marker.type = visualization_msgs::Marker::SPHERE;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.pose.position.x = mat(indices[i], 0);
//             marker.pose.position.y = mat(indices[i], 1);
//             marker.pose.position.z = mat(indices[i], 2);
//             marker.scale.x = 0.5;
//             marker.scale.y = 0.5;
//             marker.scale.z = 0.5;
//             marker.color.r = 1.0;
//             marker.color.g = 0.0;
//             marker.color.b = 0.0;
//             marker.color.a = 1.0;
//             markers.markers.push_back(marker);
//         }
//         nn_pub_.publish(markers);

//         delete tree;
//     }

//     void waypointCallback(const geometry_msgs::Point::ConstPtr& msg) {
//         waypoint_ = *msg;
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nabo/nabo.h>

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    // Convert path to Eigen matrix
    Eigen::MatrixXf points(2, msg->poses.size());
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
        points(0, i) = msg->poses[i].pose.position.x;
        points(1, i) = msg->poses[i].pose.position.y;
    }

    // Build Nabo search structurefloat x = 0.0;
    // float y = 0.0;
    Nabo::NNSearchF* nns = Nabo::NNSearchF::create(points, 2);

    // Set point to search for (change as needed)
    float x = 0.0;
    float y = 0.0;

    // Find nearest neighbor
    Eigen::VectorXf query(2);
    //query << x, y;
    //int nearest_index = nns->knn(query, 1, 0);

    // Print nearest point
    //ROS_INFO("Nearest point: (%f, %f)", points(0, nearest_index), points(1, nearest_index));

    // Clean up Nabo search structure
    delete nns;
}


int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "nearest_neighbor_node");
    ros::NodeHandle nh;

    // Subscribe to path topic
    ros::Subscriber path_sub = nh.subscribe("path_topic", 1, pathCallback);

    // Set up Nabo search parameters
    Nabo::NNSearchF* nns;
    Eigen::MatrixXf points;
    // Nabo::SearchParams params;
    // params.sorted = true;
    // params.dist_p = 2; // Euclidean distance

    // Set point to search for (change as needed)
    // float x = 0.0;
    // float y = 0.0;

    // Spin ROS node
    ros::spin();

    return 0;
}
