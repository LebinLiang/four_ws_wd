/*=====================================================================
 
 MAVCONN Micro Air Vehicle Flying Robotics Toolkit
 
 (c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>
 
 This file is part of the MAVCONN project
 
 MAVCONN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 MAVCONN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.
 
 ======================================================================*/

/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device to ROS
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
 *
 */

#include "ros/ros.h"


#include "mavlink_ros/Mavlink.h"
//#include "common.h"
#include "mavlink.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include <nav_msgs/Odometry.h>
//#include "sensor_msgs/MagneticField.h"
//#include "sensor_msgs/Temperature.h"
//#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

#include <agcar_ctrl_msg/pump_ctrl.h>
#include  <agcar_ctrl_msg/mavlink_msgid.h>


// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>


// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;


uint8_t system_id =1;
uint8_t component_id=1;
uint8_t sys_state = 1; //0 ready 1guiding

ros::Subscriber mavlink_sub;
ros::Publisher mavlink_pub;
ros::Subscriber fix_sub;
ros::Subscriber yaw_sub;
ros::Subscriber msg_sub;


double Imu_yaw = 0 ;
uint8_t Mission_Count = 0;
uint8_t SEQ = 0;
uint8_t reqflag = 0;
uint8_t test_count = 0;

void mavlinkPubmsg(mavlink_message_t* msg)
{
  mavlink_ros::Mavlink rosmavlink_msg;
  rosmavlink_msg.len = msg->len;
  rosmavlink_msg.seq = msg->seq;
  rosmavlink_msg.sysid = msg->sysid;
  rosmavlink_msg.compid = msg->compid;
  rosmavlink_msg.msgid = msg->msgid;


  for (int i = 0; i < rosmavlink_msg.len; i++)
  {
    (rosmavlink_msg.payload64).push_back(msg->payload64[i]);

  }

  //mavlink_mission_item_t mavlink_mission_item;
// mavlink_msg_mission_item_decode(msg,&mavlink_mission_item);
 // ROS_INFO("MISSION_ITEM-SEQ:%d",mavlink_mission_item.seq);

  mavlink_pub.publish(rosmavlink_msg);

}

void mavlinkHeart()
{
    uint8_t mav_mode_flag ;
    if(sys_state==0) {
    mav_mode_flag =MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ;
    }
    else {
    mav_mode_flag =MAV_MODE_FLAG_AUTO_ENABLED;
    }
    mavlink_message_t mavlink_msg;
    mavlink_msg_heartbeat_pack(system_id,component_id,&mavlink_msg,MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_GENERIC,mav_mode_flag,3,MAV_STATE_STANDBY);
    mavlinkPubmsg(&mavlink_msg);
}

void mavlinkMissionReqList()
{
    mavlink_message_t mavlink_msg;
    mavlink_msg_mission_request_list_pack(system_id,component_id,&mavlink_msg,system_id,component_id);
    mavlinkPubmsg(&mavlink_msg);
}

void mavlinkMissionReq(uint16_t seq)
{
    mavlink_message_t mavlink_msg;
    mavlink_msg_mission_request_pack(system_id,component_id,&mavlink_msg,system_id,component_id,seq);
    mavlinkPubmsg(&mavlink_msg);
}

void mavlinkMissionAck()
{
    mavlink_message_t mavlink_msg;
    mavlink_msg_mission_ack_pack(system_id,component_id,&mavlink_msg,system_id,component_id,MAV_MISSION_ACCEPTED);
    mavlinkPubmsg(&mavlink_msg);
}

void mavlinkTest()
{
    mavlink_message_t mavlink_msg;
    mavlink_msg_mission_count_pack(system_id,component_id,&mavlink_msg,system_id,component_id,10);
    mavlinkPubmsg(&mavlink_msg);
}

void mavlinkTest2(uint16_t seq,float x,float y)
{
    mavlink_message_t mavlink_msg;
    mavlink_msg_mission_item_pack(system_id,component_id,&mavlink_msg,system_id,component_id,seq,0,0,0,0,0.1,0.1,0.1,0.2,x,y,0.1);
    mavlinkPubmsg(&mavlink_msg);
}

void mavlinkTest3(int x,int y)
{
    mavlink_message_t mavlink_msg;
    mavlink_msg_global_position_int_pack(system_id,component_id,&mavlink_msg,0,x,y,0,0,0,0,0,0);
    mavlinkPubmsg(&mavlink_msg);
}


void imuCallback(const sensor_msgs::Imu& msg)
{
      tf::Quaternion quat; 
      tf::quaternionMsgToTF(msg.orientation, quat);
      double roll, pitch, yaw;//
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//
      Imu_yaw = yaw;
}

void fixCallback(const sensor_msgs::NavSatFixConstPtr& fix)
{
    mavlink_message_t mavlink_msg;
    //uint16_t timestamp = ros::Time::now().toSec();
    //*1e7
    mavlink_msg_global_position_int_pack(system_id,component_id,&mavlink_msg,0,fix->latitude*1e7,fix->longitude*1e7,0,0,0,0,0,Imu_yaw);
    mavlinkPubmsg(&mavlink_msg);

}

void msgCallback(const agcar_ctrl_msg::mavlink_msgid& MSG_id)
{
 if( MSG_id.msg_id ==40)
 {
   Mission_Count = MSG_id.data;
   ROS_INFO("MISSION_COUNT:%d",Mission_Count);
   reqflag =1;
   sys_state = 2;
 }
 if( MSG_id.msg_id ==41)
 {
   reqflag =1;
 }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mavlink_station");
  // SETUP ROS
  ros::NodeHandle mavlink_nh; // always root namespace since I assume it's somewhat "broadcast"
  //mavlink_sub = mavlink_nh.subscribe("/mavlink/from", 1000, mavlinkCallback);
  mavlink_pub = mavlink_nh.advertise<mavlink_ros::Mavlink>("/mavlink/to", 1000);

  yaw_sub = mavlink_nh.subscribe("/imu/imu_yaw",1000,imuCallback);
  fix_sub = mavlink_nh.subscribe("/fix", 10, fixCallback);
  msg_sub = mavlink_nh.subscribe("/mavlink/msgid",1000,msgCallback);
  ros::Rate r(1);
  printf("TEST!\n");

   while (ros::ok())
   {

     mavlinkHeart();
     switch(sys_state)
     {
     case 1:
     {       
       //mavlinkMissionReqList();
     }break;
     case 2:
     {
       if(reqflag ==1)
       {
       if(Mission_Count==0)
       {
         SEQ=0;
         Mission_Count=0;
         mavlinkMissionAck();
         sys_state = 3;
       }
        if(Mission_Count!=0)
        {
          mavlinkMissionReq(SEQ);
          SEQ++;
          Mission_Count--;
        }
        reqflag =0;
       }
     }break;
      case 3:
     {
        mavlinkTest3(23.163384*1e7,113.353087*1e7);
        mavlinkTest3(23.164193*1e7,113.354064*1e7);
       // mavlinkTest3(23.163902*1e7,113.352600*1e7);
        sys_state =1;

     }break;
     default:
     {

     }break;
     }


     ros::spinOnce();
     r.sleep();
   }

  return 0;
}
