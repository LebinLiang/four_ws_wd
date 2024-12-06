#pragma once
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"
#include "tf2/transform_datatypes.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/LinkStates.h"
#include "visualization_msgs/Marker.h"
#include "fourws_viz/marker_helper.h"
#include <nav_msgs/Path.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

class Viz{

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh_;
  ros::Subscriber link_states_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber initialpose_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber jointstate_sub;

  ros::Publisher odom_base_link_pub;
  ros::Publisher odom_bl_steering_link_pub;
  ros::Publisher odom_bl_wheel_link_pub;
  ros::Publisher odom_br_steering_link_pub;
  ros::Publisher odom_br_wheel_link_pub;
  ros::Publisher odom_fl_steering_link_pub;
  ros::Publisher odom_fl_wheel_link_pub;
  ros::Publisher odom_fr_steering_link_pub;
  ros::Publisher odom_fr_wheel_link_pub;
  ros::Publisher marker_pub;
  ros::Publisher debug_data_pub;
  ros::Publisher trajectory_base_link_pub;
  ros::Publisher trajectory_bl_steering_link_pub;
  ros::Publisher trajectory_br_steering_link_pub;
  ros::Publisher trajectory_fl_steering_link_pub;
  ros::Publisher trajectory_fr_steering_link_pub;
  tf2_ros::TransformBroadcaster tf_br_;     //!< @brief tf broadcaster

  nav_msgs::Path trajectory_base_link_;
  nav_msgs::Path trajectory_bl_steering_link_;
  nav_msgs::Path trajectory_br_steering_link_;
  nav_msgs::Path trajectory_fl_steering_link_;
  nav_msgs::Path trajectory_fr_steering_link_;

  geometry_msgs::PoseStamped trajectory_pose_tmp_;
  double gazebo_yaw;
  double imu_yaw;
  double path_yaw;
  double abs_cmd_angle_fl;
  double abs_cmd_angle_fr;
  double abs_cmd_angle_bl;
  double abs_cmd_angle_br;




  /* Debug */
  //可配合rqt_multiplot调试的框架
  mutable std_msgs::Float64MultiArray debug_values_;
  enum DBGVAL {
    ABS_CMD_A_FL = 0,
    ABS_CMD_A_FR = 1,
    ABS_CMD_A_BL = 2,
    ABS_CMD_A_BR = 3,
    GAZEBO_YAW = 4,
    IMU_YAW = 5,
    PATH_YAW = 6,
  };
  //这个参数要严格对应枚举值的数量
  static constexpr unsigned int num_debug_values_ = 7;


public:
  void ImuCallback(const sensor_msgs::ImuConstPtr &msg){
    imu_yaw = tf2::getYaw(msg->orientation);
  }
  void JointstateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void LinkStatesCallback(const gazebo_msgs::LinkStatesConstPtr &msg);
  void GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void InitialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void PulishDebugData();
  void pub_tf(geometry_msgs::PoseStamped ps,std::string s);

  Viz();
  ~Viz(){};
};


visualization_msgs::Marker createNextTargetMarker(const geometry_msgs::Point & next_target);
