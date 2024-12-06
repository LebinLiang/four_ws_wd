//
// Created by charmyoung on 19-7-17.
//

#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include "serial_device.h"
#include "protocol.h"
#include "crc.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <thread>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <agcar_ctrl_msg/agcar_ctrl.h>
#include <agcar_ctrl_msg/agcar_odom.h>
#include <agcar_ctrl_msg/pump_ctrl.h>

/**
 * @brief Robot Base Node
 *        Main Process is
 *        1. RECEIVING:
 *           Virtual Serial Comm -> Unpack and Get Protocol Data
 *           -> Convert to ROS Data -> ROS Publish
 *        2. SENDING:
 *           ROS Subscribe -> Get ROS Data -> Convert to Protocol Data
 *           -> Convert to Protocol Data and Pack -> Virtual Serial Comm
 */
                double x = 0.0;
                double y = 0.0;
                double th = 0.0;
                float tolerant =20;
                float last_angle[4] ;
                float angle_error[4] ;
                ros::Time current_time, last_time;

namespace robomaster {
class Robot {
 public:
  Robot(std::string device_path = "/dev/ttyUSB0"):
      device_path_(device_path) {

    if( !( ROSInit() && CommInit() ) ){
      ros::shutdown();
    };

  }
  ~Robot() {
    if(recv_thread_.joinable()){
      recv_thread_.join();
    }
  }


  void ChassisCtrlCallback(const agcar_ctrl_msg::agcar_ctrl::ConstPtr& msg){         
    //发送ChassisCtrl_msg 回调函数

    chassis_ctrl_info_.wheel_speed[0] = msg->wheel_speed[0];
    chassis_ctrl_info_.wheel_speed[1] = msg->wheel_speed[1];
    chassis_ctrl_info_.wheel_speed[2] = msg->wheel_speed[2];
    chassis_ctrl_info_.wheel_speed[3] = msg->wheel_speed[3];
    
    chassis_ctrl_info_.wheel_angle[0] = msg->wheel_angle[0];
    chassis_ctrl_info_.wheel_angle[1] = msg->wheel_angle[1];
    chassis_ctrl_info_.wheel_angle[2] = msg->wheel_angle[2];
    chassis_ctrl_info_.wheel_angle[3] = msg->wheel_angle[3];
    chassis_ctrl_info_.brake = msg->brake;


    uint16_t send_length = SenderPackSolve((uint8_t*)&chassis_ctrl_info_,sizeof(chassis_ctrl_info_t),
                                           CHASSIS_CTRL_CMD_ID,send_buff_.get());
    device_ptr_->Write(send_buff_.get(),send_length);

  }
  
  void PumpCtrlCallback(const agcar_ctrl_msg::pump_ctrl::ConstPtr& msg){
	
   pump_ctrl_info.pump1_en = msg->pump1_en;
   pump_ctrl_info.pump2_en = msg->pump2_en;

uint16_t send_length = SenderPackSolve((uint8_t*)&pump_ctrl_info,sizeof(pump_ctrl_info_t),
                                           CHASSIS_PUMP_CMD_ID,send_buff_.get());
    device_ptr_->Write(send_buff_.get(),send_length);

}


 private:
  bool ROSInit() {
    ros::NodeHandle nh;
    chassis_ctrl_sub_ = nh.subscribe("base_ctrl", 1, &Robot::ChassisCtrlCallback, this);
    pump_ctrl_sub_ = nh.subscribe("pump_ctrl",1,&Robot::PumpCtrlCallback,this);
    chassis_odom_pub_ = nh.advertise<agcar_ctrl_msg::agcar_odom>("base_odom",1);
   
    //odom_.header.frame_id = "odom";
    //odom_.child_frame_id = "base_link";

    //odom_tf_.header.frame_id = "odom";
    //odom_tf_.child_frame_id = "base_link";
    return true;
  }
  bool CommInit() {

    device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200);    //比特率921600

    if (!device_ptr_->Init()) {
      return false;
    }

    recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
    send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);

    memset(&frame_receive_header_, 0, sizeof(frame_header_struct_t));
    memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));

    /** Specific Protocol Data Initialize here**/
    //memset(&summer_camp_info_, 0, sizeof(summer_camp_info_t));

    //Start recv thread
    recv_thread_ = std::thread(&Robot::RecvThread, this);
    return true;
  }

  void RecvThread() {

      int a = 0;
      int flag = 0;

      uint8_t  last_len = 0;

    while (ros::ok()){
      //uint16_t read_length = device_ptr_->Read(recv_buff_.get(),BUFF_LENGTH);

    last_len  = device_ptr_->ReadUntil2(recv_buff_.get(),END1_SOF,END2_SOF,128);

      while (flag == 0 && last_len ==1)
      {
          if((recv_buff_[a] == END1_SOF)&&(recv_buff_[a+1]==END2_SOF))
              {
                flag =1;
                //printf("%x  ",recv_buff_[a]);
                 //printf("%x  ",recv_buff_[a+1]);
                //printf("\n");
                //printf("------------------------------------------\n");
                SearchFrameSOF(recv_buff_.get(), a);
              }
          //printf("%x  ",recv_buff_[a]);
          a++;
      }
      flag = 0;
      a=0;
     /*
     if (flag ==0 )
      {
        memcpy(p,p2,read_length);
        last_len = read_length;
        flag = 1;
      }
      else if(flag == 1)
      {
          memcpy(p+last_len,p2,read_length);
          last_last_len = read_length;
        flag = 2;
      }
      else 
      {
        memcpy(p+last_len+last_last_len,p2,read_length);
        flag = 0;
      */
       // printf("len:%d\n",read_length+last_len+last_last_len);
        //printf("len1:%d\n",read_length);
    //for( a = 0;a<read_length+last_len+last_last_len;a++){
      //printf("%x  ",Recv_Buf[a]);
      // }
      //printf("\n");
     //printf("------------------------------------------\n");
        usleep(1);
        
      }
    }

  void SearchFrameSOF(uint8_t *frame, uint16_t total_len) {
    uint16_t i;
    uint16_t index=0;
  int a =0;

for (i=0;i<total_len;)
{
if (*frame == HEADER_SOF) {
      //for(a=0;a<21;a++)
       // {
        //  printf("%x  ",*(frame+a));
       //}
       //printf("\n");
      ReceiveDataSolve(frame);
      i = total_len;
}
else
  {
    frame++;
    i++;
  }
}
/*    
    for (i = 0; i < total_len;) {
        
      if (*frame == HEADER_SOF) {
        printf("%d\n ",total_len);
  for(int  a = 0; a<total_len; a++){
      printf("%x  ",*(frame+a));
       }
      printf("\n");
        index = ReceiveDataSolve(frame);
        i += index;
        frame += index;
      } else {
        i++;
        frame++;
      }
    }
    
*/
  }

float ABS(float input)
{
   if(input>0)
	return input;
else 
	return -input;

}

float FilterDate( float input , float* eeror, float * last_data ,float tolate) 
{
    *eeror  = input - *last_data;
    if (ABS(*eeror) < tolate)
    {
        *last_data = input;
        return input;
    }
    else 
    {
      return *last_data;
    }
}


  uint16_t ReceiveDataSolve(uint8_t *frame) {
    uint8_t index = 0;
    uint16_t cmd_id = 0;

    if (*frame != HEADER_SOF) {
      return 0;
    }   

    memcpy(&frame_receive_header_, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
              
           //printf("CRC8: %d\n",Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t)));
          // printf("CRC16: %d\n",Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9));
          //printf("data length : %d \n",frame_receive_header_.data_length);

    if ((!Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t)))  || (!Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9)))
         {
                 ROS_ERROR("CRC  EEROR!");
      return 0;
    } else {
      memcpy(&cmd_id, frame + index, sizeof(uint16_t));
      index += sizeof(uint16_t);

      switch (cmd_id) {

        /** Write Your code here to get different types of data and publish them using ROS interface
         *
         *  Example:
         *
         *   case XXXX_CMD_ID:{
         *
         *    memcpy(&xxxx_info, frame + index, sizeof(xxxx_info_t))
         *    break;
         *
         *   }
         *
         */
        /*case CHASSIS_ODOM_CMD_ID: {
          
            //printf("get chassis_odom_msg");
          memcpy(&chassis_odom_info_, frame + index, sizeof(chassis_odom_info_t));

         
            ROS_INFO("Chassis Odom Info!");
           current_time = ros::Time::now();
          //odom_.header.stamp = current_time;
          //odom_.pose.pose.position.x = chassis_odom_info_.x;
          //odom_.pose.pose.position.y = chassis_odom_info_.y;
          //odom_.pose.pose.position.z = 0.0;
          //geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_odom_info_.gyro_yaw);
          //odom_.pose.pose.orientation = q;
          odom_.twist.twist.linear.x = chassis_odom_info_.vx;
          odom_.twist.twist.linear.y = chassis_odom_info_.vy;
          odom_.twist.twist.angular.z = chassis_odom_info_.vw;
        
        //double dt = 0.01;
          double dt = (current_time - last_time).toSec();
        double delta_x = (chassis_odom_info_.vx * cos(th) - chassis_odom_info_.vy * sin(th)) * dt;
        double delta_y = (chassis_odom_info_.vx * sin(th) + chassis_odom_info_.vy * cos(th)) * dt;
        double delta_th = chassis_odom_info_.vw * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th; 
        
        odom_.header.stamp = current_time;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        odom_.pose.pose.position.x = x;
        odom_.pose.pose.position.y = y;
        odom_.pose.pose.position.z = 0.0;
        odom_.pose.pose.orientation = odom_quat;

          chassis_odom_pub_.publish(odom_) ;
          
          odom_tf_.header.stamp = current_time;
          
          odom_tf_.transform.translation.x = x;
          odom_tf_.transform.translation.y = y;
          odom_tf_.transform.rotation = odom_quat;
          odom_tf_.transform.translation.z = 0.0;
          //odom_tf_.transform.rotation = q;
          tf_broadcaster_.sendTransform(odom_tf_);

            last_time = current_time;

          break;
        }
	*/
        case CHASSIS_ODOM_CMD_ID: {
          ROS_INFO("Base_Odom info!");
	  memcpy(&chassis_odom_info_, frame + index, sizeof(chassis_odom_info_));
		
      //base_odom_.S1_angle = FilterDate(chassis_odom_info_.S1_angle,&angle_error[0],&last_angle[0],tolerant);
      //base_odom_.S2_angle = FilterDate(chassis_odom_info_.S2_angle,&angle_error[1],&last_angle[1],tolerant);
      //base_odom_.S3_angle = FilterDate(chassis_odom_info_.S3_angle,&angle_error[2],&last_angle[2],tolerant);
      //base_odom_.S4_angle = FilterDate(chassis_odom_info_.S4_angle,&angle_error[3],&last_angle[3],tolerant);
       base_odom_.S1_angle = chassis_odom_info_.S1_angle;
       base_odom_.S2_angle = chassis_odom_info_.S2_angle;
       base_odom_.S3_angle = chassis_odom_info_.S3_angle;
       base_odom_.S4_angle = chassis_odom_info_.S4_angle;
	

	   chassis_odom_pub_.publish(base_odom_);
     
          break;
        }
        default: 
          break;
      }

      index += frame_receive_header_.data_length + 2;

      return index;

    }
  }



  uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
                           uint16_t cmd_id , uint8_t *send_buf) {

    uint8_t index = 0;
    frame_send_header_.SOF = HEADER_SOF;
    frame_send_header_.data_length = data_length;
    frame_send_header_.seq++;

    Append_CRC8_Check_Sum((uint8_t * ) & frame_send_header_, sizeof(frame_header_struct_t));


    memcpy(send_buf, &frame_send_header_, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));

    index += sizeof(uint16_t);

    memcpy(send_buf + index, data, data_length);

    Append_CRC16_Check_Sum(send_buf, data_length + 9);

    return data_length + 9;
  }

 private:

  //! VCOM Data Receiving Thread (Tips: VCOM Sending part is in Each ROS Data Callback)
  std::thread recv_thread_;

  //! Device Information and Buffer Allocation
  std::string device_path_;
  std::shared_ptr<SerialDevice> device_ptr_;
  std::unique_ptr<uint8_t[]> recv_buff_;
  std::unique_ptr<uint8_t[]> send_buff_;
  const unsigned int BUFF_LENGTH = 512;

  //! Frame Information
  frame_header_struct_t frame_receive_header_;
  frame_header_struct_t frame_send_header_;

  /** @brief specific protocol data are defined here
   *         xxxx_info_t is defined in protocol.h
   */

  //! Receive from VCOM
  //summer_camp_info_t summer_camp_info_;


  chassis_odom_info_t chassis_odom_info_;

  agcar_ctrl_msg::agcar_odom base_odom_;
  //geometry_msgs::TransformStamped odom_tf_;//! ros chassis odometry tf
  //nav_msgs::Odometry odom_;//! ros odometry message
   
  //! Send to VCOM
  chassis_ctrl_info_t chassis_ctrl_info_;
  pump_ctrl_info_t pump_ctrl_info;


  /** @brief ROS data corresponding to specific protocol data are defined here
   *         You can use ROS provided message data type or create your own one
   *         More information please refer to
   *               http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
   */


  /** @brief ROS Subscription and Publication
   */
  ros::Subscriber chassis_ctrl_sub_; //! ros subscriber for chassis velocity control
  ros::Subscriber pump_ctrl_sub_;
  ros::Publisher chassis_odom_pub_; //! ros publisher for odometry information
  tf::TransformBroadcaster tf_broadcaster_;//! ros chassis odometry tf broadcaster

};
}

#endif //ROBOMASTER_ROBOT_H
