/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool append_zone = false;
double init_x , init_y;
double real_x,real_y;
int init_flag = 0;

  nav_msgs::Path ros_path_;
  ros::Publisher state_pub_;
  ros::Publisher xy_pub;
  nav_msgs::Path ros_goal_path_;

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {

  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_DEBUG_THROTTLE(60,"No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty()) {
      if(append_zone) {
        odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = "world";
      }
    } else {
      if(append_zone) {
        odom.header.frame_id = frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = frame_id;
      }
    }

    if(init_flag == 0)
    {
    init_x = easting;
    init_y = northing;
    init_flag = 1;
    }

  real_x = easting - init_x;
  real_y = northing - init_y;
  
    odom.child_frame_id = "gps_link";

    odom.pose.pose.position.x = real_x;
    odom.pose.pose.position.y = real_y;
    odom.pose.pose.position.z = 0;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = real_x;
    pose.pose.position.y = real_y;
    pose.pose.position.z = 0.0;

    ros_path_.poses.push_back(pose);
    //ROS_INFO("X: %f\n",real_x);
    //ROS_INFO("Y: %f\n",real_y);
    odom_pub.publish(odom);
    state_pub_.publish(ros_path_);
    
  }
}

void gps_Callback(const nav_msgs::Path& gps_point)
{
    std::string zone;
    double northing, easting;
    double x_goal,y_goal;
    double z_set;
    if(gps_point.poses.back().pose.position.x==0&&gps_point.poses.back().pose.position.y==0)
    {
        x_goal =0;
        y_goal =0;
        z_set = 1;
    }
    else {
        LLtoUTM(gps_point.poses.back().pose.position.x, gps_point.poses.back().pose.position.y, northing, easting, zone);
               ROS_INFO("TF a point!!");

               if(init_flag == 0) //test
               {
               init_x = easting;
               init_y = northing;
               init_flag = 1;
               }                  //test

               x_goal = easting - init_x;
               y_goal = northing - init_y;
               z_set = 0.0;
    }


    geometry_msgs::PoseStamped pose;

    ros_goal_path_.header.frame_id = "world";
    ros_goal_path_.header.stamp = ros::Time::now();

    pose.header = ros_goal_path_.header;

    pose.pose.position.x = x_goal;
    pose.pose.position.y = y_goal;
    pose.pose.position.z = z_set;

    ros_goal_path_.poses.push_back(pose);
    xy_pub.publish(ros_goal_path_);

}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_tf_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  ROS_INFO("[START]");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<bool>("append_zone", append_zone, false);

 
  odom_pub = node.advertise<nav_msgs::Odometry>("gps_odom", 10);
  ros::Subscriber fix_sub = node.subscribe("/fix", 10, callback);
   state_pub_ = node.advertise<nav_msgs::Path>("gps_path", 10);
   ros::Subscriber gps_sub = node.subscribe("gps_goal_path",100,gps_Callback);
   xy_pub = node.advertise<nav_msgs::Path>("xy_goal_path",100);

  ros::spin();
}

