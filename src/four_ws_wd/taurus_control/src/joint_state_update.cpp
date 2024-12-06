#include<iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <agcar_ctrl_msg/agcar_ctrl.h>
#include <agcar_ctrl_msg/agcar_odom.h>
#include <sensor_msgs/Imu.h>
#include "tf/transform_datatypes.h"
#include <tf/tf.h>


using namespace std;

float w_speed[4] ;
float w_angle[4] ;
double roll,pitch,car_yaw;

void base_odom_Callback(const agcar_ctrl_msg::agcar_odom::ConstPtr& msg){
w_angle[0] = msg->S1_angle;
w_angle[1] = msg->S2_angle;
w_angle[2] = msg->S3_angle;
w_angle[3] = msg->S4_angle;
}

void base_ctrl_Callback(const agcar_ctrl_msg::agcar_ctrl::ConstPtr& msg){
w_speed[0] = msg->wheel_speed[0];
w_speed[1] = msg->wheel_speed[1];
w_speed[2] = msg->wheel_speed[2];
w_speed[3] = msg->wheel_speed[3];
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg){

tf::Quaternion Q1;
tf::quaternionMsgToTF(msg->orientation,Q1);
tf::Matrix3x3(Q1).getRPY(roll,pitch,car_yaw);

}


int main(int argc, char** argv)
 {
    ros::init(argc, argv, "joint_state_update"); 
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);    
    ros::Subscriber joint_odom = n.subscribe<agcar_ctrl_msg::agcar_odom>("/base_odom",1,&base_odom_Callback);
    ros::Subscriber joint_ctrl = n.subscribe<agcar_ctrl_msg::agcar_ctrl>("/base_ctrl",1,&base_ctrl_Callback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data",1,&imu_Callback);
    ros::Rate loop_rate(10);    
    const double degree = M_PI/180;
    // robot state
    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(8);
        joint_state.position.resize(8);
        joint_state.velocity.resize(8);
	joint_state.name[0]="fl_steering_joint";
        joint_state.position[0] = w_angle[0]*degree-car_yaw;
	joint_state.name[1] ="fl_wheel_joint";
        joint_state.velocity[1] = w_speed[0] ;
	joint_state.name[2] ="fr_steering_joint";
        joint_state.position[2] = w_angle[1]*degree-car_yaw;
        joint_state.name[3] ="fr_wheel_joint";
        joint_state.velocity[3] = w_speed[1] ;
	joint_state.name[4] ="rl_steering_joint";
        joint_state.position[4] = w_angle[2]*degree-car_yaw;
	joint_state.name[5] ="rl_wheel_joint";
        joint_state.velocity[5] = w_speed[2] ;
        joint_state.name[6] ="rr_steering_joint";
        joint_state.position[6] = w_angle[3]*degree-car_yaw;
        joint_state.name[7] ="rr_wheel_joint";
        joint_state.velocity[7] = w_speed[3] ;
        ros::spinOnce();
        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}

