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
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

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
    ros::Subscriber scan_sub_;
    ros::Subscriber vel_odom_sub;
    ros::Publisher path_pub_;
    ros::Publisher leftpath_pub_;
    ros::Publisher rightpath_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher drive_pub;
    std::string targetf;
    std::string sourcef;
    float prev_error ;
    float safety_radius_;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener;
    std::vector<PointXY> refwp;
    std::vector<PointXY> leftwp;
    std::vector<PointXY> rightwp;
    std::vector<PointXY> occupancy;
    float cur_vel;
    
    // TODO: create ROS subscribers and publishers

public:
    //n = ros::NodeHandle();
    laser_geometry::LaserProjection projector_;
    geometry_msgs::TransformStamped map_to_laser;
    PurePursuit(): odom_sub_(), vel_odom_sub(), scan_sub_(), path_pub_(), marker_pub_(), drive_pub(), targetf(), sourcef(), prev_error(), tf2_listener(tf_buffer),
                   safety_radius_(), cur_vel(), leftpath_pub_(), rightpath_pub_()
    {
      odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
      vel_odom_sub = n.subscribe("/vesc/odom", 1, &PurePursuit::vel_callback, this);
       
      path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
      scan_sub_ = n.subscribe("/scan", 1, &PurePursuit::scanCallback, this);
      leftpath_pub_ = n.advertise<nav_msgs::Path>("/wps_left", 1, true);
      rightpath_pub_ = n.advertise<nav_msgs::Path>("/wps_right", 1, true);
      marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
      drive_pub =  n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
      targetf = "car1_30m/laser"; 
      sourcef = "map";    
      prev_error = 0.0f;
      safety_radius_ = 0.15f; //tune this
      cur_vel = 0.0f;
    }
    

    //pico_tree::KdTree<pico_tree::StdTraits<std::reference_wrapper<std::vector<PointXY> >>> tree1(std::vector<PointXY> points, int max_leaf_size = 10); 
        // TODO: create ROS subscribers and publishers
    
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
      publishPlan(leftwp, leftpath_pub_);
      publishPlan(rightwp, rightpath_pub_);
      std::cout<<rightwp.size()<<leftwp.size()<<std::endl;
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

      publishPlan(refwp, path_pub_);
    }

    void pointVis(std::pair<float, float> pts) 
    {
      visualization_msgs::Marker points;
      points.header.stamp = ros::Time(0);
      points.header.frame_id = targetf;
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

   void transformer(const std::vector<int> indices, std::vector<PointXY> pts, float* xg, float* yg, int* inde)
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
          p.pose.position.x = pts[indices[i]].data[0];
          p.pose.position.y = pts[indices[i]].data[1];
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
      float vel = 1.5f - angle / 30.0f;
      if (vel < 0.7f) {
          vel = 0.7;
      }
      return vel;
    }

    void driver(float angle, int stop=1)
    {
      float kp = 0.33f;
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
      float velocity = Getvelocity(angle);
      drive_msg.drive.speed = velocity * stop; //velocity
      drive_pub.publish(drive_msg);
    }

    void publishPlan(std::vector<PointXY> traj, ros::Publisher path_pub) 
    {
      nav_msgs::Path msg;
      msg.header.frame_id = "/map";
      msg.header.stamp = ros::Time::now();
      for (int i = 0; i < traj.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = traj[i].data[0];
        pose.pose.position.y = traj[i].data[1];
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
      }
      std::cout<<"published plan"<<msg.poses.size()<<std::endl;
      path_pub.publish(msg);
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

    float nearest_wp(float x, float y)
    {
        float minD = 100.0f;
        for (int i = 0; i < occupancy.size(); ++i)
        {
            float xi = occupancy[i].data[0];
            float yi = occupancy[i].data[1];
            float dis = (x - xi)*(x-xi) + (y - yi)*(y-yi);
            if (dis < minD)
            {
                minD = dis;
            }  
        }
        return minD;
    }
    
    bool collision_checker(float xg, float yg) 
    {
      float r = safety_radius_;
      float d = r * 0.8f; // step size for discretization
      float Dist = std::sqrt(xg*xg + yg*yg);
      int iters = std::max(std::min(static_cast<int>(Dist / d), 2), 6); // number of points to check
      float dd = nearest_wp(xg, yg);
      if (dd < r*r) { return true; }
      else {
            float slope = std::atan2(yg, xg);
            // if (std::abs(slope) > 0.7f) {
            //     std::cout<<"not dying here, or am I"<<std::endl;
            //     return true; // not possible to turn above 45-50 degrees
            // }
            for (int i = 1; i < iters; ++i) {
                float xn = i * d * std::cos(slope) + 0.1f;
                float yn = i * d * std::sin(slope);
                float dd = nearest_wp(xn, yn);
                if (dd < r*r) {
                  std::cout<< "xn, yn, dd, r "<< xn << " "<< yn <<" "<< dd<< " "<<r<<std::endl;
                    return true;
                }
            }
            //pointVis(pts);
            return false;
        }
  
    }

    void vel_callback(const nav_msgs::Odometry::ConstPtr &poser)
        {cur_vel = poser->twist.twist.linear.x;}
    
    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
      {
        float x = pose_msg->pose.pose.position.x;
        float y = pose_msg->pose.pose.position.y;
        float r = std::min(std::max(0.7f, 1.6f * cur_vel / 1.8f), 1.6f);
        //float r = 1.2;
        std::vector<int> indices = ball_query(refwp, x, y, r);
        float xg, yg;
        int ind;
        future_col_ = false;
       
        transformer(indices,refwp, &xg, &yg, &ind);
        if (collision_checker(xg, yg))
        {std::cout<< "Sed, I am crashing soon, send help!"<<std::endl;
        
         //checking centre traj
          std::vector<int> indices = ball_query(leftwp, x, y, r);
          transformer(indices,leftwp, &xg, &yg, &ind);
           if (collision_checker(xg, yg)) //checking left traj
            {
                std::cout<< "left crash, send help!"<<std::endl;
                std::vector<int> indices = ball_query(rightwp, x, y, r);
                transformer(indices, rightwp, &xg, &yg, &ind);
                
                if (collision_checker(xg, yg)) //checking right traj
                {
                  std::cout<<"all blocked ##"<<std::endl;
                  future_col_ = true;
                  //return;
                }
            }
          }
        
          
          if (future_col_){driver(0.0, 0);}
          else
          {
            pointVis(std::make_pair(xg, yg));

            float L = (xg*xg) + (yg*yg);
            float angle = 0.0f;
            if (L > 0)
            {
              angle = 2*yg/L;
              driver(angle);

            }
          }
          
          occupancy.clear();
      }
      
    
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
   {
      sensor_msgs::PointCloud cloud;
      projector_.projectLaser(*scan_in, cloud);
      int i = 0;
      occupancy.clear();
      occupancy.reserve(400);
      for(int i = 0; i < cloud.points.size(); ++i) {
          if (i > 240 && i < 840 && i%2 == 0) {
            PointXY p1;
            p1.data[0] = float(cloud.points[i].x);
            p1.data[1] = float(cloud.points[i].y);
            occupancy.push_back(p1);
            }
		    }
      // ros::Duration(0.2).sleep();

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
    ros::init(argc, argv, "pp_obst");
    PurePursuit pp;
    pp.point_loader(datawp);
    pp.point_loader2(datawp2);

    ros::spin();
    return 0;
}





