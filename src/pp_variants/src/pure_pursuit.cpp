#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <limits>
//#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
// using namespace std;
using namespace std;
struct PointXY {
  float data[2];
};


class PurePursuit {
private:
    ros::NodeHandle n;
    bool future_col_;
    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;
    // ros::Publisher leftpath_pub_;
    // ros::Publisher rightpath_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher drive_pub;
    std::string targetf;
    std::string sourcef;
    float prev_error ;
    //tf::TransformListener tf_list;
    std::vector<PointXY> refwp;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener; 

    
    // TODO: create ROS subscribers and publishers

public:
    //n = ros::NodeHandle();
    
    geometry_msgs::TransformStamped map_to_laser;
    PurePursuit(): odom_sub_(), path_pub_(), marker_pub_(), drive_pub(), targetf(), sourcef(), prev_error(), tf2_listener(tf_buffer)
    {
      odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
       path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
      // leftpath_pub_ = n.advertise<nav_msgs::Path>("/wps_left", 1, true);
      // rightpath_pub_ = n.advertise<nav_msgs::Path>("/wps_right", 1, true);
      marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
      drive_pub =  n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
      targetf = "car1_30m/laser"; 
      sourcef = "map";    
      prev_error = 0.0f;
    }
    
    
    //ros::Subscriber scan_sub_ = n.subscribe("/scan", 1, &PurePursuit::scanCallback, this);
   
   
    void point_loader(std::vector<std::vector<float> > a)
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

      publishPlan();
      std::cout<<refwp.size()<<std::endl;
    }

    void pointVis(std::pair<float, float> pts) 
    {
      visualization_msgs::Marker points;
      points.header.stamp = ros::Time(0);
      points.header.frame_id = sourcef;
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
      // for (int i = 0; i < pointlist.size(); ++i)
      // {
        geometry_msgs::Point p;
        p.x = pts.first;
        p.y = pts.second;
        p.z = 0;
        pp.push_back(p);

      //}
      
      points.points = pp;
      marker_pub_.publish(points);
    }

    void transformer(const std::vector<int> indices, float* xg, float* yg, int* inde)
    {
      float xoi = 0.0f;
      float yoi = 0.0f;
      int ind = 0;
      geometry_msgs::PoseStamped ptf;
      map_to_laser = tf_buffer.lookupTransform(targetf, sourcef, ros::Time(0), ros::Duration(1.0) );

      for (int i = 0; i < indices.size(); ++i)
      {
          geometry_msgs::PoseStamped p;
          p.header.frame_id = sourcef;
          p.header.stamp = ros::Time(0);
          p.pose.position.x = refwp[indices[i]].data[0];
          p.pose.position.y = refwp[indices[i]].data[1];
          p.pose.orientation.w = 1.0;
          geometry_msgs::PoseStamped pose_tf;
          try
          {
            tf2::doTransform(p, pose_tf, map_to_laser);
          }
          catch(tf2::TransformException &ex)
          {
              ROS_ERROR("%s", ex.what());
              *xg = xoi;
              *yg = yoi;
              *inde = ind;

          }
          
          if (pose_tf.pose.position.x > xoi)
          {
              ind = i;
              xoi = pose_tf.pose.position.x;
              yoi = pose_tf.pose.position.y;
          }
      }
      //return {xoi, yoi, ind};
      *xg = xoi;
      *yg = yoi;
      *inde = ind;
    }

    float Getvelocity(float angle) 
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
      //float kd = -1.25f;
      float error = angle;
      angle = kp * error; //+ kd * (error - prev_error);
      if (std::abs(angle) < 0.033f) {
          angle = 0.0f;
      }
      prev_error = error;
      ackermann_msgs::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = ros::Time::now();
      drive_msg.header.frame_id = "";
      drive_msg.drive.steering_angle = angle;
      velocity = Getvelocity(angle);
      drive_msg.drive.speed = velocity; //velocity
      drive_pub.publish(drive_msg);
    }

    void publishPlan() 
    {
      nav_msgs::Path msg;
      msg.header.frame_id = "/map";
      msg.header.stamp = ros::Time::now();
      for (int i = 0; i < refwp.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = refwp[i].data[0];
        pose.pose.position.y = refwp[i].data[1];
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
      }
      std::cout<<"published plan"<<msg.poses.size()<<std::endl;
      path_pub_.publish(msg);
    }

    
    std::vector<int> ball_query(float x, float y, float r)
    {
      float rsq = r*r;
      std::vector<int> indices;
      for (int i = 0; i < refwp.size(); ++i)
      {
        float xi = refwp[i].data[0];
        float yi = refwp[i].data[1];
        float dis = (x - xi)*(x-xi) + (y - yi)*(y-yi);
        if (dis < rsq)
        {
          indices.push_back(i);
        }
      }
      return indices;
    }
    
    
    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
      {
        float x = pose_msg->pose.pose.position.x;
        float y = pose_msg->pose.pose.position.y;
        float r = 0.9;
        
        std::vector<int> indices = ball_query(x, y, r);
        if (indices.size() > 0)
        {
          //std::vector<std::vector<float> > pts;
          float xg, yg;
          int ind;
          transformer(indices, &xg, &yg, &ind);
          //std::cout<<ind<<std::endl;

          pointVis(std::make_pair(refwp[indices[ind]].data[0], refwp[indices[ind]].data[1]));

          float L = (xg*xg) + (yg*yg);
          float angle = 0.0f;
          if (L > 0)
          {
            angle = 2*yg/L;
            float velocity = 0.0f; //Getvelocity(angle);
            driver(angle, velocity);

          }

        }
        else{driver(0.0f, 0.0f);}
        }
    

};


class WaypointServer
{
public:
  
  std::vector<std::vector<float> > load(std::string waypoint_file)
  {
    const int rows_num = 3; 
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
    std::string fname ="/home/nvidia/f1ten-scripts/traj.csv";
    WaypointServer waypoint_server;
    std::vector<std::vector<float> > datawp = waypoint_server.load(fname);
    std::cout<<datawp.size()<<std::endl;
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    pp.point_loader(datawp);
    ros::spin();
    return 0;
}





