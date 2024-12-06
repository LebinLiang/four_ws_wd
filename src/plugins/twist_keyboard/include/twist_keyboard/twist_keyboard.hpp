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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Float64.h"
#include "tf2/transform_datatypes.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include "geometry_msgs/Twist.h"

class Keyboard{

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_vel_pub;

  geometry_msgs::Twist cmd_vel_;
  public:

  void WriteData(double x,double y,double z){
    cmd_vel_.linear.x = x;
    cmd_vel_.linear.y = y;
    cmd_vel_.angular.z = z;
  }
  void PublishCmd(){
    cmd_vel_pub.publish(cmd_vel_);
}

  Keyboard();
  ~Keyboard(){
  };
};


