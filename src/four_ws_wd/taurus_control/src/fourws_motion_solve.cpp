#include "fourws_motion_solve.h"


double normalizeRadian(const double angle)
{
   double n_angle = std::fmod(angle, 2 * M_PI);
   n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
   return n_angle;
}

void FourwsMotionSolve::JointstateCallback(const sensor_msgs::JointStateConstPtr &msg){
  cur_fl_position =msg->position.at(0);
  cur_fr_position =msg->position.at(2);
  cur_bl_position =msg->position.at(4);
  cur_br_position = msg->position.at(6);
 // printf("bl:%f",cur_bl_position);
 // printf("br:%f",cur_br_position);
 // printf("fl:%f",cur_fl_position);
 // printf("fr:%f",cur_fr_position);
}

FourwsMotionSolve::FourwsMotionSolve()
{
  ros::NodeHandle nh;
  nh.param<double>("/4ws/length_ox", l_ox, 1.37);
  nh.param<double>("/4ws/length_oy", l_oy, 1.6);
  nh.param<double>("/4ws/r_wheel", r_wheel, 0.33);

  nh.param<double>("/4ws/vx_accel_limit", vx_accel_limit, 1.0); //m/s^2
  nh.param<double>("/4ws/vy_accel_limit", vy_accel_limit, 1.0); //m/s^2

  l_r = l_ox*l_ox+l_oy*l_oy;
  fourws_bl_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/rl_steering_position_controller/command", 10);
  fourws_br_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/rr_steering_position_controller/command", 10);
  fourws_fl_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fl_steering_position_controller/command", 10);
  fourws_fr_position_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fr_steering_position_controller/command", 10);
  fourws_bl_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/rl_wheel_velocity_controller/command", 10);
  fourws_br_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/rr_wheel_velocity_controller/command", 10);
  fourws_fl_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fl_wheel_velocity_controller/command", 10);
  fourws_fr_velocity_cmd_ = nh.advertise<std_msgs::Float64>("/4ws/fr_wheel_velocity_controller/command", 10);

  debug_data_pub = nh.advertise<std_msgs::Float64MultiArray>("/4ws/solve_control/debug_values", 1);


  agcar_ctrl_pub_ = nh.advertise<agcar_ctrl_msg::agcar_ctrl>("/base_ctrl",1);

  imu_sub_ = nh.subscribe("/imu", 1, &FourwsMotionSolve::ImuCallback, this);
  cmd_vel_sub_ = nh.subscribe("/cmd_vel", 1, &FourwsMotionSolve::CmdVelCallback, this);
  jointstate_sub = nh.subscribe("/joint_states",10, &FourwsMotionSolve::JointstateCallback,this);

  double VX, VY;
  VX_ = 0.0;
  VY_ = 0.0;
  VW_ = 0.0;

  Lpf1d_VW_.init(0.9);

}

void FourwsMotionSolve::CmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  VX_ = msg->linear.x ;
  VY_ = msg->linear.y ;
  VW_ = msg->angular.z;
}


void FourwsMotionSolve::ImuCallback(const sensor_msgs::Imu &msg)
{
  yaw_ = tf2::getYaw(msg.orientation);
}

double limtDiff(const double set_value , const double prev_value , double diff_lim , const double time){
  double diff = std::max(std::min(set_value - prev_value, diff_lim*time), -diff_lim*time);

  return prev_value + diff;


}
void FourwsMotionSolve::DataProcessor()
{
  //set prev diff_time
  filter_VX_ = limtDiff(VX_,filter_VX_,vx_accel_limit,0.005);
  filter_VY_ = limtDiff(VY_,filter_VY_,vy_accel_limit,0.005);
  filter_VW_ = Lpf1d_VW_.filter(VW_);

  //vx_ = filter_VX_ * cos(yaw_) - filter_VY_ * sin(yaw_);
  //vy_ = filter_VX_ * sin(yaw_) + filter_VY_ * cos(yaw_);
  //vw_ = filter_VW_;

vx_  = -VX_;
vy_  = -VY_;
vw_  =VW_;
}


void FourwsMotionSolve::Solve()
{
    int8_t sign1=1  ;
    int8_t sign2=1 ;
    int8_t sign3=1 ;
    int8_t sign4=1 ;
  v_bl_steering_link_ = sqrt(vy_ * vy_ + vx_ * vx_ + l_r * vw_ * vw_ - 2 * vw_ * (l_ox * vy_ + l_oy * vx_)); //0
  v_fl_steering_link_ = sqrt(vy_ * vy_ + vx_ * vx_ + l_r * vw_ * vw_ - 2 * vw_ * (l_oy * vx_ - l_ox * vy_)); //1
  v_br_steering_link_ = sqrt(vy_ * vy_ + vx_ * vx_ + l_r * vw_ * vw_ + 2 * vw_ * (l_oy * vx_ - l_ox * vy_)); //2
  v_fr_steering_link_ = sqrt(vy_ * vy_ + vx_ * vx_ + l_r * vw_ * vw_ + 2 * vw_ * (l_ox * vy_ + l_oy * vx_)); //3

  a_bl_steering_link_ = atan2((vy_ - vw_ * l_ox), (vx_ - vw_ * l_oy)); //0
  a_fl_steering_link_ = atan2((vy_ + vw_ * l_ox), (vx_ - vw_ * l_oy)); //1
  a_br_steering_link_ = atan2((vy_ - vw_ * l_ox), (vx_ + vw_ * l_oy)); //2
  a_fr_steering_link_ = atan2((vy_ + vw_ * l_ox), (vx_ + vw_ * l_oy)); //3

//printf("a_br1%f\n",a_br_steering_link_); 

  if(a_fl_steering_link_>2.35)
  {
a_fl_steering_link_ = a_fl_steering_link_-3.14;
sign1 = -1;
  }
  else if(a_fl_steering_link_<-2.35)
  {
a_fl_steering_link_ = a_fl_steering_link_+3.14;
  sign1 = -1;
  }
  else
  {
    sign1 = 1;
  }
  
    if(a_bl_steering_link_>2.35)
  {
a_bl_steering_link_ = a_bl_steering_link_-3.14;
sign2 = -1;
  }
  else if(a_bl_steering_link_<-2.35)
  {
a_bl_steering_link_ = a_bl_steering_link_+3.14;
  sign2 = -1;
  }
    else 
  {
    sign2 = 1;
  }
   
    if(a_fr_steering_link_>2.35)
  {
a_fr_steering_link_ = a_fr_steering_link_-3.14;
sign3 = -1;
  }
  else if(a_fr_steering_link_<-2.35)
  {
a_fr_steering_link_ = a_fr_steering_link_+3.14;
  sign3 = -1;
  }
    else 
  {
    sign3 = 1;
  }

    if(a_br_steering_link_>2.35)
  {
a_br_steering_link_ = a_br_steering_link_-3.14;
sign4 = -1;
  }
  else if(a_br_steering_link_<-2.35)
  {
a_br_steering_link_ = a_br_steering_link_+3.14;
  sign4 = -1;
  }
    else 
  {
    sign4 = 1;
  }

//printf("a_br2%f\n",a_br_steering_link_); 
//printf("cur_br%f\n",cur_br_position); 
/*
if(fabs(a_fr_steering_link_-cur_fr_position)>0.329 || fabs(a_br_steering_link_-cur_br_position)>0.329||fabs(a_bl_steering_link_-cur_bl_position)>0.329||fabs(a_fl_steering_link_-cur_fl_position)>0.329)
{
sign1= 0 ;
sign2= 0 ;
sign3= 0 ;
sign4 =0 ;
//printf("stop1\n");
}
*/
v_fl_steering_link_ = sign1*v_fl_steering_link_;
v_bl_steering_link_ = sign2*v_bl_steering_link_;
v_fr_steering_link_ = sign3*v_fr_steering_link_;
v_br_steering_link_ = sign4*v_br_steering_link_;

  PubControlCmd();
}

void FourwsMotionSolve::DebugDataPublish()
{
  //配合rqt_multiplot使用的调试代码输出
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);
  debug_values_.data.at(FILTER_VX) = filter_VX_;
  debug_values_.data.at(FILTER_VY) = filter_VY_;
  debug_values_.data.at(FILTER_VW) = filter_VW_;

  debug_data_pub.publish(debug_values_);
}


void FourwsMotionSolve::PubControlCmd()
{
  std_msgs::Float64 cmd;

  // std::cout << a_bl_steering_link_ << " " << a_br_steering_link_ << " " << a_fl_steering_link_ << " " << a_fr_steering_link_ << std::endl;
  //cmd.data = cur_bl_position + normalizeRadian(a_bl_steering_link_ - cur_bl_position);
  cmd.data = a_bl_steering_link_ ;
  fourws_bl_position_cmd_.publish(cmd);
  //cmd.data = cur_br_position ;// + normalizeRadian(a_br_steering_link_ - cur_br_position);
    cmd.data = a_br_steering_link_;
  fourws_br_position_cmd_.publish(cmd);
  //cmd.data = cur_fl_position  ;//+ normalizeRadian(a_fl_steering_link_ - cur_fl_position);
   cmd.data = a_fl_steering_link_;
  fourws_fl_position_cmd_.publish(cmd);
  //cmd.data = cur_fr_position ;// + normalizeRadian(a_fr_steering_link_ - cur_fr_position);
     cmd.data = a_fr_steering_link_;
  fourws_fr_position_cmd_.publish(cmd);

  cmd.data = v_bl_steering_link_ / r_wheel;
  fourws_bl_velocity_cmd_.publish(cmd);
  cmd.data = v_br_steering_link_ / r_wheel;
  fourws_br_velocity_cmd_.publish(cmd);
  cmd.data = v_fl_steering_link_ / r_wheel;
  fourws_fl_velocity_cmd_.publish(cmd);
  cmd.data = v_fr_steering_link_ / r_wheel;
  fourws_fr_velocity_cmd_.publish(cmd);

  base_ctrl_.wheel_speed[0] = v_bl_steering_link_ ;
  base_ctrl_.wheel_speed[1] = v_br_steering_link_ ;
  base_ctrl_.wheel_speed[2] = v_fl_steering_link_ ;
  base_ctrl_.wheel_speed[3] = v_fr_steering_link_ ;

  base_ctrl_.wheel_angle[0] = a_bl_steering_link_;
  base_ctrl_.wheel_angle[1] = a_br_steering_link_;
  base_ctrl_.wheel_angle[2] = a_fl_steering_link_;
  base_ctrl_.wheel_angle[3] = a_fr_steering_link_;
  base_ctrl_.brake = 0;

  agcar_ctrl_pub_.publish(base_ctrl_);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fourws_motion_solve");

  FourwsMotionSolve fourws_motion_solve;
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    fourws_motion_solve.DataProcessor();
    fourws_motion_solve.Solve();
    fourws_motion_solve.DebugDataPublish();
    loop_rate.sleep();
  }
}
