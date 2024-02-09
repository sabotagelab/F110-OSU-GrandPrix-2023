#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
using namespace std;
struct PointXY {
  float data[2];
};

class RootNode {
private:
    ros::NodeHandle n;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher center_enum_pub;
    ros::Publisher alt_traj_enum_pub;
    float safety_radius_;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener;
    std::vector<PointXY> refwp;
    std::vector<PointXY> leftwp;
    std::vector<PointXY> rightwp;
    std::vector<PointXY> map_points;
    float dynamic_thresh;
    int min_angle;
    int max_angle;
    float max_horizon_sq;
    float myx;
    float myy;

    
    // TODO: create ROS subscribers and publishers

public:
    //n = ros::NodeHandle();
    laser_geometry::LaserProjection projector_;
    geometry_msgs::TransformStamped laser_to_map;
    RootNode(): odom_sub_(), scan_sub_(), marker_pub_(), 
                   tf2_listener(tf_buffer), min_angle(), max_angle(),
                   safety_radius_(), dynamic_thresh(), max_horizon_sq(),
                   center_enum_pub(), alt_traj_enum_pub(), myx(), myy()
    {
      odom_sub_ = n.subscribe("/pf/pose/odom", 1, &RootNode::pose_callback, this);
      scan_sub_ = n.subscribe("/scan", 1, &RootNode::scanCallback, this);
      marker_pub_ = n.advertise<visualization_msgs::Marker>("/cloud_map", 1);
      alt_traj_enum_pub = n.advertise<std_msgs::Int8>("/alt_traj", 1);
      center_enum_pub = n.advertise<std_msgs::Int8>("/center_traj_free", 1);
      min_angle = 240;
      max_angle = 840;
      max_horizon_sq = 4.0;
      safety_radius_ = 0.12;
      dynamic_thresh = 0.1;
    }

    void point_loader(std::vector<std::vector<float> > a)
    {   
      for (int i = 0; i < a.size(); i++) { 
        for (int j = 0; j < a[i].size(); j++) {
            PointXY  p2, p3;
            p2.data[0] = a[i][2];
            p2.data[1] = a[i][3];
            p3.data[0] = a[i][4];
            p3.data[1] = a[i][5];
            rightwp.push_back(p2);
            leftwp.push_back(p3);
        }
      }
      //std::cout<<refwp[0][0]<<rightwp[0][0]<<leftwp[0][0]<<std::endl;
    }

    void point_loader2(std::vector<std::vector<float> > a)
    {   
      for (int i = 0; i < a.size(); i++) { 
        for (int j = 0; j < a[i].size(); j++) {
            PointXY p1;
            float x = 0.1f * a[i][0];
            float y = 0.1f * a[i][1];
            p1.data[0] = x;
            p1.data[1] = y;
            refwp.push_back(p1);
        }
      }
    }

    void pointVis() 
    {
      visualization_msgs::Marker points;
      points.header.stamp = ros::Time(0);
      std::string targetf = "map";
      points.header.frame_id = targetf;
      points.ns = "cloud_map";
      points.id = 0;
      points.type = visualization_msgs::Marker::POINTS;
      points.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.scale.x = 0.05;
      points.scale.y = 0.05;
      points.color.b = 1.0;
      points.color.a = 0.5;
      std::vector<geometry_msgs::Point> pp;
      for (int i = 0; i < map_points.size(); ++i)
      {
        geometry_msgs::Point p;
        p.x = map_points[i].data[0];
        p.y = map_points[i].data[1];
        p.z = 0;
        pp.push_back(p);

      }
      
      points.points = pp;
      marker_pub_.publish(points);
    }

   std::vector<int> ball_query(std::vector<PointXY> refs ,float x, float y, float r)
    {
      float rsq = r*r;
      std::vector<int> indices;
      for (int i = 0; i < refs.size(); ++i)
      {
        float xi = refs[i].data[0];
        float yi = refs[i].data[1];
        float dis = (x - xi)*(x-xi) + (y - yi)*(y-yi);
        if (dis < rsq)
        {
          indices.push_back(i);
        }
      }
      return indices;
    }


     float min_dist_to_traj(std::vector<PointXY> traj)
    {
        //returns distance between given trajectory and its nearest occupancy point
        float r = 1.0f;
        std::vector<int> indices = ball_query(traj, myx, myy, r); //trajectory
        float minD = 100.0f;
        for (int j = 0; j < indices.size(); ++j)
        {
            float x = traj[indices[j]].data[0];
            float y = traj[indices[j]].data[1];
            for (int i = 0; i < map_points.size(); ++i)
            {
                float xi = map_points[i].data[0];
                float yi = map_points[i].data[1];
                float dis = (x - xi)*(x-xi) + (y - yi)*(y-yi);
                if (dis < minD)
                {
                    minD = dis;
                    if (minD < safety_radius_){return minD;}
                }  
            }
        }
        
        return minD;
    }
   
   
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
   {
      sensor_msgs::PointCloud cloud;
      projector_.projectLaser(*scan_in, cloud);
      int i = 0;
      std::string targetf = "map";
      laser_to_map = tf_buffer.lookupTransform(targetf, scan_in->header.frame_id, ros::Time(0), ros::Duration(1.0) );
      map_points.clear();
      for(int i = 0; i < cloud.points.size(); ++i) 
       {
          if (i > min_angle && i < max_angle && i%3 == 0) 
           {
                geometry_msgs::PoseStamped p;
                p.header = scan_in->header;
                p.pose.position.x = float(cloud.points[i].x);
                p.pose.position.y = float(cloud.points[i].y);
                p.pose.orientation.w = 1.0;
                geometry_msgs::PoseStamped pose_tf;
                try
                {
                    tf2::doTransform(p, pose_tf, laser_to_map);
                    PointXY p1;
                    p1.data[0] = pose_tf.pose.position.x;
                    p1.data[1] = pose_tf.pose.position.y;
                    map_points.push_back(p1);
                }
                catch(tf2::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
		   }
       }
       pointVis();
       publish_enums();
      // ros::Duration(0.2).sleep();

    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
      {
        myx = pose_msg->pose.pose.position.x;
        myy = pose_msg->pose.pose.position.y;
      }

    void publish_enums()
    {
        bool not_free_flag = false;
        float r = 1.0f;
        std_msgs::Int8 k;
        float min_dist_to_center = min_dist_to_traj(refwp); //add a single line to get nearest map point to traj
        if (min_dist_to_center < safety_radius_)
        {
            not_free_flag = true;
            float min_dist_to_left = min_dist_to_traj(leftwp);
            if (min_dist_to_left >= safety_radius_)
            {
                k.data = 1;
                alt_traj_enum_pub.publish(k);
            }
            else
            {
                float min_dist_to_right = min_dist_to_traj(rightwp);
                if (min_dist_to_right >= safety_radius_)
                {
                    k.data = 2;
                    alt_traj_enum_pub.publish(k);
                }
                else
                {
                    std::cout<<"all blocked"<<std::endl;
                    k.data = 3;
                    alt_traj_enum_pub.publish(k);
                }
            }
        }
        if (!not_free_flag)
        {
            k.data = 0;
            center_enum_pub.publish(k);
        }
        else
        {
           k.data = 1;
           center_enum_pub.publish(k);
        }

    }


};


class WaypointServer
{
public:
  
  std::vector<std::vector<float> > load(std::string waypoint_file, int rows)
  {
    const int rows_num = rows; 
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(waypoint_file.c_str());
    std::string line;
    std::vector<std::vector<float> > wps;
    while(ifs.good())
    {
      getline(ifs, line);
      if(line.empty()){ break; }
      tokenizer tokens(line, sep);
      std::vector<float> data;
      tokenizer::iterator it = tokens.begin();
      for(; it != tokens.end() ; ++it){
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


int main(int argc, char ** argv) {
    std::string fname ="/home/nvidia/f1ten-scripts/ways.csv";
    std::string fname2 ="/home/nvidia/f1ten-scripts/traj.csv";
    WaypointServer waypoint_server;
    std::vector<std::vector<float> > datawp = waypoint_server.load(fname, 6);
    std::vector<std::vector<float> > datawp2 = waypoint_server.load(fname2, 3);
    std::cout<<datawp.size()<<std::endl;
    ros::init(argc, argv, "root_node");
    RootNode RN;
    RN.point_loader(datawp);
    RN.point_loader2(datawp2);

    ros::spin();
    return 0;
}