#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <thread>
// #include "conio.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>     
#include <iostream>

namespace KeyboardInput {
  int scanKeyboard()
  {
    system("stty -echo");
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    system("stty echo");
    return in;
  }
}

void Command();
char command = '0';

class FourwsControl {
  public:
    FourwsControl();
    ~FourwsControl() = default;
    void VelocitySolve();
  private:
    double vx_;
    double vy_;
    double vw_;
    ros::Publisher fourws_bl_position_cmd_;
    ros::Publisher fourws_br_position_cmd_;
    ros::Publisher fourws_fl_position_cmd_;
    ros::Publisher fourws_fr_position_cmd_;
    ros::Publisher fourws_bl_velocity_cmd_;
    ros::Publisher fourws_br_velocity_cmd_;
    ros::Publisher fourws_fl_velocity_cmd_;
    ros::Publisher fourws_fr_velocity_cmd_;
};

FourwsControl::FourwsControl()
{
  ros::NodeHandle nh;
  fourws_bl_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/bl_steering_position_controller/command", 10);
  fourws_br_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/br_steering_position_controller/command", 10);
  fourws_fl_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fl_steering_position_controller/command", 10);
  fourws_fr_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fr_steering_position_controller/command", 10);
  
  fourws_bl_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/bl_wheel_velocity_controller/command", 10);
  fourws_br_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/br_wheel_velocity_controller/command", 10);
  fourws_fl_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fl_wheel_velocity_controller/command", 10);
  fourws_fr_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fr_wheel_velocity_controller/command", 10);

  vx_ = 0.0;
  vy_ = 0.0;
  vw_ = 0.0;
  
  auto command_thread= std::thread(Command);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    switch (command) {
      case 'i':
        vx_ = 10.0;
        vy_ = 0.0;
        vw_ = 0.0;
        break;
      case ',':
        vx_ = -10.0;
        vy_ = 0.0;
        vw_ = 0.0;
        break;
      case 'j':
        vx_ = 0.000001;
        vy_ = -10.0;
        vw_ = 0.0;
        break;
      case 'l':
        vx_ = 0.000001;
        vy_ = 10.0;
        vw_ = 0.0;
        break;
      case 'u':
        vx_ = 10.0;
        vy_ = -10.0;
        vw_ = 0.0;
        break;
      case 'o':
        vx_ = 10.0;
        vy_ = 10.0;
        vw_ = 0.0;
        break;
      case 'm':
        vx_ = -10.0;
        vy_ = -10.0;
        vw_ = 0.0;
        break;
      case '.':
        vx_ = -10.0;
        vy_ = 10.0;
        vw_ = 0.0;
        break;
      case 'c':
        vx_ = 0.0;
        vy_ = 0.0;
        vw_ = -10.0;
        break;
      case 'v':
        vx_ = 0.0;
        vy_ = 0.0;
        vw_ = 10.0;
        break;
      case 'k':
        vx_ = 0.0;
        vy_ = 0.0;
        vw_ = 0.0;
        break;
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return;
      default:
        break;
    }

    VelocitySolve();

    loop_rate.sleep();
  }
}

void FourwsControl::VelocitySolve()
{
  std_msgs::Float64 cmd;
  if (vx_ == 0 && vy_ == 0 && vw_ == 0) {
    cmd.data = 0;
    fourws_bl_velocity_cmd_.publish(cmd);
    fourws_br_velocity_cmd_.publish(cmd);
    fourws_fl_velocity_cmd_.publish(cmd);
    fourws_fr_velocity_cmd_.publish(cmd);
  } else if (vw_ == 10.0) {
    cmd.data = -0.785;
    fourws_bl_position_cmd_.publish(cmd);
    cmd.data = 0.785;
    fourws_br_position_cmd_.publish(cmd);
    cmd.data = 0.785;
    fourws_fl_position_cmd_.publish(cmd);
    cmd.data = -0.785;
    fourws_fr_position_cmd_.publish(cmd);

    cmd.data = 10;
    fourws_bl_velocity_cmd_.publish(cmd);
    cmd.data = -10;
    fourws_br_velocity_cmd_.publish(cmd);
    cmd.data = 10;
    fourws_fl_velocity_cmd_.publish(cmd);
    cmd.data = -10;
    fourws_fr_velocity_cmd_.publish(cmd);
  } else if (vw_ == -10.0) {
    cmd.data = -0.785;
    fourws_bl_position_cmd_.publish(cmd);
    cmd.data = 0.785;
    fourws_br_position_cmd_.publish(cmd);
    cmd.data = 0.785;
    fourws_fl_position_cmd_.publish(cmd);
    cmd.data = -0.785;
    fourws_fr_position_cmd_.publish(cmd);

    cmd.data = -10;
    fourws_bl_velocity_cmd_.publish(cmd);
    cmd.data = 10;
    fourws_br_velocity_cmd_.publish(cmd);
    cmd.data = -10;
    fourws_fl_velocity_cmd_.publish(cmd);
    cmd.data = 10;
    fourws_fr_velocity_cmd_.publish(cmd);
  } else {
    double angle = atan2(vy_, vx_);
    cmd.data = angle;
    fourws_bl_position_cmd_.publish(cmd);
    fourws_br_position_cmd_.publish(cmd);
    fourws_fl_position_cmd_.publish(cmd);
    fourws_fr_position_cmd_.publish(cmd);

    cmd.data = vx_ * cos(angle) + vy_ * sin(angle);
    fourws_bl_velocity_cmd_.publish(cmd);
    fourws_br_velocity_cmd_.publish(cmd);
    fourws_fl_velocity_cmd_.publish(cmd);
    fourws_fr_velocity_cmd_.publish(cmd);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_teleop");

  FourwsControl fourws_control;
}

void Command() {
  std::cout << "*******************" << std::endl;
  std::cout << "Moving around:" << std::endl
            << "u    i    o" << std::endl
            << "j    k    l" << std::endl
            << "m    ,    ." << std::endl
            << "c: turn left" << std::endl
            << "v: turn right" << std::endl
            << "k: stop" << std::endl
            << "esc: exit program" << std::endl;
  std::cout << "*******************" << std::endl;
  while (command != 27) {
    command = KeyboardInput::scanKeyboard();
  }
}
