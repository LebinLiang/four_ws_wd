#include <iostream>
#include "fourws_viz.hpp"

#define NUM_ALL 9

#define num_ground 0
#define num_base 1
#define num_bl_s 2
#define num_bl_w 3
#define num_br_s 4
#define num_br_w 5
#define num_fl_s 6
#define num_fl_w 7
#define num_fr_s 8
#define num_fr_w 9


visualization_msgs::Marker createNextTargetMarker(const geometry_msgs::Point & next_target)
{
  auto marker = createDefaultMarker(
    "map", "next_target", 0, visualization_msgs::Marker::SPHERE, createMarkerScale(0.05, 0.05, 0.05),
    createMarkerColor(0.0, 1.0, 0.0, 1.0));

  marker.pose.position = next_target;
  marker.pose.position.z = 0.2;
  return marker;
}

//弧度制归一化
double normalizeRadian(const double angle)
{
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}
Viz::Viz(): nh(""), pnh_("~"){

  imu_sub = nh.subscribe("/imu",10, &Viz::ImuCallback,this);
  link_states_sub = nh.subscribe("/gazebo/link_states",10, &Viz::LinkStatesCallback,this);
  goal_sub = nh.subscribe("/move_base_simple/goal",10, &Viz::GoalCallback,this);
  initialpose_sub = nh.subscribe("/initialpose",10, &Viz::InitialposeCallback,this);
  jointstate_sub = nh.subscribe("/4ws/joint_states",10, &Viz::JointstateCallback,this);

  odom_base_link_pub = nh.advertise<nav_msgs::Odometry>("/4ws/odom_base", 10);

  odom_bl_steering_link_pub  = nh.advertise<nav_msgs::Odometry>("/4ws/odom_bl_steering", 10);
  odom_bl_wheel_link_pub     = nh.advertise<nav_msgs::Odometry>("/4ws/odom_bl_wheel", 10);
  odom_br_steering_link_pub  = nh.advertise<nav_msgs::Odometry>("/4ws/odom_br_steering", 10);
  odom_br_wheel_link_pub     = nh.advertise<nav_msgs::Odometry>("/4ws/odom_br_wheel", 10);
  odom_fl_steering_link_pub  = nh.advertise<nav_msgs::Odometry>("/4ws/odom_fl_steering", 10);
  odom_fl_wheel_link_pub     = nh.advertise<nav_msgs::Odometry>("/4ws/odom_fl_wheel", 10);
  odom_fr_steering_link_pub  = nh.advertise<nav_msgs::Odometry>("/4ws/odom_fr_steering", 10);
  odom_fr_wheel_link_pub     = nh.advertise<nav_msgs::Odometry>("/4ws/odom_fr_wheel", 10);
  debug_data_pub = nh.advertise<std_msgs::Float64MultiArray>("/fourws_viz/debug_values", 1);

  trajectory_base_link_pub = nh.advertise<nav_msgs::Path>("/trajectory_base_link", 10);
  trajectory_bl_steering_link_pub = nh.advertise<nav_msgs::Path>("/trajectory_bl_steering_link", 10);
  trajectory_br_steering_link_pub = nh.advertise<nav_msgs::Path>("/trajectory_br_steering_link", 10);
  trajectory_fl_steering_link_pub = nh.advertise<nav_msgs::Path>("/trajectory_fl_steering_link", 10);
  trajectory_fr_steering_link_pub = nh.advertise<nav_msgs::Path>("/trajectory_fr_steering_link", 10);

  trajectory_base_link_.header.stamp = ros::Time::now();
  trajectory_base_link_.header.frame_id = "map";
  trajectory_bl_steering_link_.header.stamp = ros::Time::now();
  trajectory_bl_steering_link_.header.frame_id = "map";
  trajectory_br_steering_link_.header.stamp = ros::Time::now();
  trajectory_br_steering_link_.header.frame_id = "map";
  trajectory_fl_steering_link_.header.stamp = ros::Time::now();
  trajectory_fl_steering_link_.header.frame_id = "map";
  trajectory_fr_steering_link_.header.stamp = ros::Time::now();
  trajectory_fr_steering_link_.header.frame_id = "map";

  marker_pub = nh.advertise<visualization_msgs::Marker>("/4ws/center_point", 10);
}

void Viz::pub_tf(geometry_msgs::PoseStamped ps,std::string s){
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = ps.header;
  transformStamped.child_frame_id = s;
  transformStamped.transform.translation.x = ps.pose.position.x;
  transformStamped.transform.translation.y = ps.pose.position.y;
  transformStamped.transform.translation.z = ps.pose.position.z;

  transformStamped.transform.rotation.x = ps.pose.orientation.x;
  transformStamped.transform.rotation.y = ps.pose.orientation.y;
  transformStamped.transform.rotation.z = ps.pose.orientation.z;
  transformStamped.transform.rotation.w = ps.pose.orientation.w;

  tf_br_.sendTransform(transformStamped);
}

void Viz::JointstateCallback(const sensor_msgs::JointStateConstPtr &msg){
   abs_cmd_angle_bl = normalizeRadian(msg->position.at(0));
   abs_cmd_angle_br = normalizeRadian(msg->position.at(2));
   abs_cmd_angle_fl = normalizeRadian(msg->position.at(4));
   abs_cmd_angle_fr = normalizeRadian(msg->position.at(6));

}


//发布车身和每个轮子的可视化位姿信息，以odom类型显示
//发布map到base_link的TF变换
void Viz::LinkStatesCallback(const gazebo_msgs::LinkStatesConstPtr &msg)
{
  //std::cout<<*msg<<std::endl;
  if(msg->pose.empty()||msg==nullptr||msg->pose.size()<NUM_ALL)
    return;
  std::vector<nav_msgs::Odometry> odom_list;
//  odom_bl_steering,odom_bl_wheel,odom_br_steering,odom_br_wheel,
//                     odom_fl_steering,odom_fl_wheel,odom_fr_steering,odom_fr_wheel;
  for (size_t i=0;i<msg->pose.size();i++) {
    nav_msgs::Odometry odom_tmp;
    odom_tmp.header.frame_id = "map";
    odom_tmp.header.stamp  = ros::Time::now();
    odom_tmp.pose.pose = msg->pose.at(i);
    odom_tmp.twist.twist = msg->twist.at(i);
//    odom_tmp.child_frame_id
    odom_list.push_back(odom_tmp);
//    std::cout<<i<<"   "<<odom_tmp<<std::endl;

  }
  //z轴方向用不到。这里直接算成合速度
  odom_list.at(num_base).twist.twist.linear.z =
      std::hypot(odom_list.at(num_base).twist.twist.linear.x,odom_list.at(num_base).twist.twist.linear.y);
  odom_base_link_pub.publish(odom_list.at(num_base));
  odom_bl_steering_link_pub.publish(odom_list.at(num_bl_s));
  odom_bl_wheel_link_pub.publish(odom_list.at(num_bl_w));
  odom_br_steering_link_pub.publish(odom_list.at(num_br_s));
  odom_br_wheel_link_pub.publish(odom_list.at(num_br_w));
  odom_fl_steering_link_pub.publish(odom_list.at(num_fl_s));
  odom_fl_wheel_link_pub.publish(odom_list.at(num_fl_w));
  odom_fr_steering_link_pub.publish(odom_list.at(num_fr_s));
  odom_fr_wheel_link_pub.publish(odom_list.at(num_fr_w));

  geometry_msgs::PoseStamped ps;
  ps.pose = odom_list.at(num_base).pose.pose;
  ps.header.frame_id = "map";
  ps.header.stamp  = ros::Time::now();
  pub_tf(ps,"base_link");
  pub_tf(ps,"base_footprint");
  gazebo_yaw = tf2::getYaw(msg->pose.at(num_base).orientation);


  trajectory_pose_tmp_.pose = odom_list.at(num_base).pose.pose;
  trajectory_pose_tmp_.header.stamp = ros::Time::now();
  trajectory_pose_tmp_.header.frame_id = "map";
  trajectory_base_link_.poses.push_back(trajectory_pose_tmp_);
  trajectory_base_link_pub.publish(trajectory_base_link_);

  trajectory_pose_tmp_.pose = odom_list.at(num_bl_s).pose.pose;
  trajectory_pose_tmp_.header.stamp = ros::Time::now();
  trajectory_pose_tmp_.header.frame_id = "map";
  trajectory_bl_steering_link_.poses.push_back(trajectory_pose_tmp_);
  trajectory_bl_steering_link_pub.publish(trajectory_bl_steering_link_);

  trajectory_pose_tmp_.pose = odom_list.at(num_br_s).pose.pose;
  trajectory_pose_tmp_.header.stamp = ros::Time::now();
  trajectory_pose_tmp_.header.frame_id = "map";
  trajectory_br_steering_link_.poses.push_back(trajectory_pose_tmp_);
  trajectory_br_steering_link_pub.publish(trajectory_br_steering_link_);

  trajectory_pose_tmp_.pose = odom_list.at(num_fl_s).pose.pose;
  trajectory_pose_tmp_.header.stamp = ros::Time::now();
  trajectory_pose_tmp_.header.frame_id = "map";
  trajectory_fl_steering_link_.poses.push_back(trajectory_pose_tmp_);
  trajectory_fl_steering_link_pub.publish(trajectory_fl_steering_link_);

  trajectory_pose_tmp_.pose = odom_list.at(num_fr_s).pose.pose;
  trajectory_pose_tmp_.header.stamp = ros::Time::now();
  trajectory_pose_tmp_.header.frame_id = "map";
  trajectory_fr_steering_link_.poses.push_back(trajectory_pose_tmp_);
  trajectory_fr_steering_link_pub.publish(trajectory_fr_steering_link_);

}

//接收/move_base_simple/goal话题并可视化为一个点（不选用/clicked_point话题是因为它只能在有物体的地方发布点）
void Viz::GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg){
  visualization_msgs::Marker marker;
  marker = createNextTargetMarker(msg->pose.position);
  marker_pub.publish(marker);
  trajectory_base_link_.poses.clear();
  trajectory_bl_steering_link_.poses.clear();
  trajectory_br_steering_link_.poses.clear();
  trajectory_fl_steering_link_.poses.clear();
  trajectory_fr_steering_link_.poses.clear();
}

void Viz::InitialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker_pub.publish(marker);
}

void Viz::PulishDebugData(){
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);
  debug_values_.data.at(ABS_CMD_A_FL) = abs_cmd_angle_fl;
  debug_values_.data.at(ABS_CMD_A_FR) = abs_cmd_angle_fr;
  debug_values_.data.at(ABS_CMD_A_BL) = abs_cmd_angle_bl;
  debug_values_.data.at(ABS_CMD_A_BR) = abs_cmd_angle_br;
  debug_values_.data.at(GAZEBO_YAW) = gazebo_yaw;
  debug_values_.data.at(IMU_YAW) = imu_yaw;

  debug_data_pub.publish(debug_values_);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fourws_viz");
  Viz v;
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    v.PulishDebugData();
    loop_rate.sleep();
  }
}

