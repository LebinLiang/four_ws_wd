#ifndef FOURWS_MOTION_SOLVE_H
#define FOURWS_MOTION_SOLVE_H

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf2/utils.h>
#include <iostream>
#include"geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "lowpass_filter.h"
#include "std_msgs/Float64MultiArray.h"
#include <agcar_ctrl_msg/agcar_ctrl.h>


class FourwsMotionSolve {
  public:
    FourwsMotionSolve();
    ~FourwsMotionSolve() = default;
    void ImuCallback(const sensor_msgs::Imu &msg);
    void CmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
    void JointstateCallback(const sensor_msgs::JointStateConstPtr &msg);

    void Solve();
    void DataProcessor();

    void PubControlCmd();
    void DebugDataPublish();

  private:
    ros::Publisher debug_data_pub;
    ros::Publisher fourws_bl_position_cmd_;
    ros::Publisher fourws_br_position_cmd_;
    ros::Publisher fourws_fl_position_cmd_;
    ros::Publisher fourws_fr_position_cmd_;
    ros::Publisher fourws_bl_velocity_cmd_;
    ros::Publisher fourws_br_velocity_cmd_;
    ros::Publisher fourws_fl_velocity_cmd_;
    ros::Publisher fourws_fr_velocity_cmd_;
    
    ros::Publisher agcar_ctrl_pub_;
    
    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber jointstate_sub;
    
   agcar_ctrl_msg::agcar_ctrl base_ctrl_;
    
    double l_ox,l_oy,l_r,r_wheel; //机器人参数

    double VX_;   //设定的全局坐标系线速度
    double VY_;   //设定的全局坐标系线速度
    double VW_;

    double filter_VX_; //平滑后全局坐标系X方向线速度
    double filter_VY_; //平滑后全局坐标系Y方向线速度
    double filter_VW_; //平滑后角速度

    //低通滤波
    Lpf1d Lpf1d_VW_;

    double vx_;   // 车身坐标系x方向线速度
    double vy_;   // 车身坐标系y方向线速度
    double vw_;   // 角速度

    double yaw_;  //机器人航向角

    double a_base_link_;
    double v_base_link_;
    double w_base_link_;
    double r_base_link_;

    double a_bl_steering_link_; //控制角度 弧度制
    double v_bl_steering_link_; //控制速度 rad/s
    double w_bl_steering_link_;
    double r_bl_steering_link_;

    double a_br_steering_link_;
    double v_br_steering_link_;
    double w_br_steering_link_;
    double r_br_steering_link_;

    double a_fl_steering_link_;
    double v_fl_steering_link_;
    double w_fl_steering_link_;
    double r_fl_steering_link_;

    double a_fr_steering_link_;
    double v_fr_steering_link_;
    double w_fr_steering_link_;
    double r_fr_steering_link_;


    double cur_fl_position,cur_fr_position,cur_bl_position,cur_br_position; //转向电机位置反馈，用于处理gazebo控制问题

    double vx_accel_limit,vy_accel_limit,vw_accel_limit;

    /* Debug */
    //可配合rqt_multiplot调试的框架
    mutable std_msgs::Float64MultiArray debug_values_;
    enum DBGVAL {
      FILTER_VX = 0,
      FILTER_VY = 1,
      FILTER_VW = 2,
    };
    //这个参数要严格对应枚举值的数量
    static constexpr unsigned int num_debug_values_ = 3;
};

#endif // FOURWS_MOTION_SOLVE_H
