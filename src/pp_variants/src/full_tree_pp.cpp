#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
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
// #include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>
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
    ros::Subscriber enum_sub;
    ros::Subscriber alt_traj_sub;
    ros::Subscriber vel_odom_sub;

    ros::Publisher path_pub_;
    ros::Publisher leftpath_pub_;
    ros::Publisher rightpath_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher drive_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener;

    std::vector<PointXY> refwp;
    std::vector<float> refvels;
    std::vector<PointXY> leftwp;
    std::vector<PointXY> rightwp;
    float handcrafted_vel;
    float cur_vel;
    std::string targetf;
    std::string sourcef;

    //new bools
    bool center_free;
    bool stat_obs;
    bool dyn_obs;
    bool left_free;
    bool right_free;

public:
   PurePursuit(): odom_sub_(), vel_odom_sub(), path_pub_(), marker_pub_(), 
                  drive_pub(), cur_vel(), leftpath_pub_(), rightpath_pub_(),
                  enum_sub(), alt_traj_sub(), handcrafted_vel(), targetf(), 
                  sourcef(), tf2_listener(tf_buffer),
                  center_free(), stat_obs(), dyn_obs(), left_free(), right_free()
    {
      odom_sub_ = n.subscribe("/pf/pose/odom", 1, &PurePursuit::pose_callback, this);
      vel_odom_sub = n.subscribe("/vesc/odom", 1, &PurePursuit::vel_callback, this);
       
      path_pub_ = n.advertise<nav_msgs::Path>("/wps", 1, true);
      leftpath_pub_ = n.advertise<nav_msgs::Path>("/wps_left", 1, true);
      rightpath_pub_ = n.advertise<nav_msgs::Path>("/wps_right", 1, true);
      marker_pub_ = n.advertise<visualization_msgs::Marker>("/nearest", 1);
      drive_pub =  n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
      cur_vel = 0.0f;
      enum_sub = n.subscribe("/center_traj_free", 1, &PurePursuit::enum_callback, this );
      alt_traj_sub = n.subscribe("/alt_traj", 1, &PurePursuit::alt_traj_callback, this );
      targetf = "car1_30m/laser"; 
      sourcef = "map";
      
      handcrafted_vel = 1.2f;
      //new bools
      center_free = false;
      stat_obs = false;
      dyn_obs = false;
      left_free = false;
      right_free = false;
    }
    geometry_msgs::TransformStamped map_to_laser;


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
            float v =  a[i][2]/7.5f;
            p1.data[0] = x;
            p1.data[1] = y;
            refvels.push_back(v);
            refwp.push_back(p1);
        }
      }

      publishPlan(refwp, path_pub_);
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
      float vel = 1.6f - angle / 30.0f;
      if (vel < 0.8f) {
          vel = 0.8;
      }
      return vel;
    }

    
    void driver(float xg, float yg, int stop=1, int near_ind = 0, 
                float maxvel = 2.0, bool refvel = true)
    {
      float L = (xg*xg) + (yg*yg);
      float angle = 0.0f;
      float velocity = 0.0f;
      if (L > 0 && stop == 1)
      {
        angle = 2*yg/L;
      }
      else {stop = 0;}
      float kp = 0.33f;
      float error = angle;
      angle = kp * error;
      if (std::abs(angle) < 0.035f) {
          angle = 0.0f;
      }
      //used for capping velocity
      if (velocity > maxvel){velocity = maxvel;}
      //prev_error = error;
      ackermann_msgs::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = ros::Time::now();
      drive_msg.header.frame_id = "";
      drive_msg.drive.steering_angle = angle * stop;
      if (refvel){velocity = refvels[near_ind];}
      else {velocity = Getvelocity(angle);}
      //used for capping velocity
      if (velocity > maxvel){velocity = maxvel;}
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

    int nearest_wp(std::vector<int> indices, float x, float y)
    {
        float minD = 1000.0f;
        int idx = 0;
        for (int i = 0; i < indices.size(); ++i)
        {
            float xi = refwp[indices[i]].data[0];
            float yi = refwp[indices[i]].data[1];
            float dis = (x - xi)*(x-xi) + (y - yi)*(y-yi);
            if (dis < minD)
            {
                minD = dis;
                idx = indices[i];
                if (minD < 0.15f){return idx;}
            }  
        }
        return idx;
    }

    void track_given_traj(std::vector<PointXY> refs, float x, float y, float maxvel = 2.0f, bool refvel = true)
    {
      //dynamic horizon longer when center free and smaller when center block
      //float r = std::min(std::max(min_horizon, max_horizon * cur_vel / 1.8f), max_horizon);

      float r = std::min(std::max(0.5f, 1.8f * cur_vel / 1.8f), 1.8f);
      if (!refvel){r = std::min(std::max(0.7f, 1.6f * cur_vel / 1.8f), 1.6f);}
      int near_ind = 0;
      std::vector<int> indices = ball_query(refs, x, y, r);
      if (indices.size() > 0)
      {
        float xg, yg;
        int ind;
        transformer(indices, refs, &xg, &yg, &ind);
        pointVis(std::make_pair(refs[indices[ind]].data[0], refs[indices[ind]].data[1]));
        if (refvel) {near_ind = nearest_wp(indices, x, y);}
        driver(xg, yg, 1, near_ind, maxvel, refvel);
        }
    }

    void state_tree(float x, float y)
    {
        if (center_free) //normie pp
        {
            track_given_traj(refwp, x, y, 2.0f, true);
            std::cout<<"free center"<<std::endl;
        }

        else if (stat_obs)
        {
          if (left_free)
          {
              std::cout<<"free left with static obst"<<std::endl;
              track_given_traj(leftwp, x, y, 2.0f, false);
          }
          else if (right_free)
          {
            std::cout<<"free right with static obst"<<std::endl;
            track_given_traj(rightwp, x, y, 2.0f, false);
          }
          else
          {
            std::cout << "all blocked static" << std::endl;
            //try going reverse maybe?
            driver(0.0f, 0.0f, 0, 0);
          }
        }

        else if (dyn_obs)
        { //not considering curved track
          float dynamic_obst_vel = 0.0; //TODO get this vel
            
          std::vector<int> indices = ball_query(refwp, x, y, 0.5);
          int near_ind = nearest_wp(indices, x, y);
          int ref_velocity = refvels[near_ind];
          if (ref_velocity < 1.5f) //curved track
          {
            std::cout<<"dynamic curved follow "<<std::endl;
            track_given_traj(refwp, x, y, dynamic_obst_vel, true); //cap n follow
          }
          else //straight track
          { 
            if (dynamic_obst_vel > handcrafted_vel*cur_vel)
            {
              std::cout<<"dynamic straight follow"<<std::endl;
              track_given_traj(refwp, x, y, dynamic_obst_vel, true); //cap n follow
            }
            else if (left_free)
            {
                std::cout<<"dynamic overtake left free"<<std::endl;
                track_given_traj(leftwp, x, y, 2.0f, false); //overtake
            }
            else if (right_free)
            {
              std::cout<<"dynamic overtake right free"<<std::endl;
              track_given_traj(rightwp, x, y, 2.0f, false); //overtake
            }
            else
            {
              std::cout<<"dynamic all block follow"<<std::endl;
              track_given_traj(refwp, x, y,  dynamic_obst_vel, true); //cap n follow
            }

          }
            
        }



        else{driver(0.0f, 0.0f, 0, 0);}
    }


    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
      {
        float x = pose_msg->pose.pose.position.x;
        float y = pose_msg->pose.pose.position.y;
        state_tree(x, y);
        }

    void vel_callback(const nav_msgs::Odometry::ConstPtr &poser)
        {cur_vel = poser->twist.twist.linear.x;}


    void enum_callback(const std_msgs::Int8::ConstPtr &enum_msg)
    {
      center_free = false;
      stat_obs = false;
      dyn_obs = false;
      int inp = enum_msg -> data;
      if (inp == 0) {center_free = true;}
      if (inp == 1) {stat_obs = true;}
      if (inp == 2) {dyn_obs = true;}
    }

    void alt_traj_callback(const std_msgs::Int8::ConstPtr &enum_msg)
    {
      left_free = false;
      right_free = false;
      int inp = enum_msg -> data;
      if (inp == 1) {left_free = true;}
      if (inp == 2) {right_free = true;}
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
    ros::init(argc, argv, "full_tree_pp");
    PurePursuit pp;
    pp.point_loader(datawp);
    pp.point_loader2(datawp2);

    ros::spin();
    return 0;
}
