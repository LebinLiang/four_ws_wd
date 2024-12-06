#!/usr/bin/python3
#coding=utf-8
# Copyright 2020 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import rospy
import tf
import time
import sys
import math
import string
import copy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Vector3
from geometry_msgs.msg import PoseStamped


import numpy as np

imu_speed = [0.0,0.0] #Vx Vy
imu_lasttime = rospy.Time()
NE_speed = [0.0,0.0] #Vnorth Veast
gpsodom_location = [0.0,0.0] #Dnorth Deast
bag_raw_x = []
bag_raw_y = []
real_yaw  = 0.0 
output_location = [0.0,0.0] #DX DY
fuse_path = Path()
odom = Odometry()
angular_z = 0 

'''
def callback_imu(data):
    global imu_speed
    global imu_lasttime
    now_time = rospy.Time.now()
    dt = now_time.to_sec() - imu_lasttime.to_sec()
    x_acc = data.linear_acceleration.x
    y_acc = -data.linear_acceleration.y  # east
    if math.fabs(x_acc)>0.01:
        imu_speed[0] += x_acc*dt
    else:
        imu_speed[0] = 0
    if math.fabs(y_acc)>0.01:
        imu_speed[1] += y_acc*dt
    else:
        imu_speed[1] = 0
    if math.fabs(imu_speed[0])<0.02:
        imu_speed[0] = 0
    if math.fabs(imu_speed[1])<0.02:
        imu_speed[1] = 0
    imu_lasttime = now_time
    #print "imu_speed_x:%f"%(imu_speed[0])
    #print "imu_speed_y:%f"%(imu_speed[1])
'''

def callback_gpsodom(data):
    global gpsodom_location
    global bag_raw_x,bag_raw_y
    raw_x  = data.pose.pose.position.x - 0.01
    raw_y = data.pose.pose.position.y- 0.03
    bag_raw_x.append(copy.deepcopy(raw_x))
    bag_raw_y.append(copy.deepcopy(raw_y))
    if len(bag_raw_x)>10 :
        del bag_raw_x[0]
        del bag_raw_y[0]
    gpsodom_location[0] = np.mean(bag_raw_x)
    gpsodom_location[1] = np.mean(bag_raw_y)

def callback_vel(data):
    global NE_speed
    NE_speed[0] = data.twist.linear.x
    NE_speed[1] = data.twist.linear.y
    print "N_speed_x:%f"%(NE_speed[0])
    print "E_speed_y:%f"%(NE_speed[1])

def callable_imu_yaw(data):
    global real_yaw
    global angular_z
    imu_Angle = [0.0,0.0,0.0]
    imu_Angle = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    real_yaw =imu_Angle[2] 
    angular_z = data.twist.twist.angular.z 

def  fuselocation_tf():
    global real_yaw
    global NE_speed
    global gpsodom_location
    global imu_speed
    global output_location
    global fuse_path 
    global Path_Pub
    global odom_Pub
    global odom
    global angular_z

    translate_speed = [0.0,0.0]
    fuse_NE_speed = [0.0,0.0]
    odom_NE_distance = [0.0,0.0]
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50)
    last_time = rospy.Time()
    last_gps_location = [0.0,0.0]
    Dgps_distance = [0.0,0.0]
    Dv_distance = [0.0,0.0]
     
    while not rospy.is_shutdown(): 
        now_time = rospy.Time.now()
        Dt =  now_time.to_sec() - last_time.to_sec()
        print(" real_yaw get%f"%(real_yaw))
        Dgps_distance[0] = gpsodom_location[0] - last_gps_location[0]
        Dgps_distance[1] = gpsodom_location[1] - last_gps_location[1]
        Dv_distance[0] = NE_speed[0] *Dt
        Dv_distance[1] = NE_speed[1]*Dt
        translate_speed[0] =  NE_speed[0]*math.cos(real_yaw)+NE_speed[1]*math.sin(real_yaw)
        translate_speed[1] =  NE_speed[1]*math.cos(real_yaw)-NE_speed[0]*math.sin(real_yaw)
        fuse_NE_speed[0] =  NE_speed[0] 
        fuse_NE_speed[1] = NE_speed[1]

        if math.fabs(fuse_NE_speed[0]) >0.1:
            odom_NE_distance[0] += Dv_distance[0]
            if math.fabs(Dv_distance[0] - Dgps_distance[0])<1:
                output_location[0] = odom_NE_distance[0]*0.15 + gpsodom_location[0]*0.85
            else:
                output_location[0] = odom_NE_distance[0]*0.85 + gpsodom_location[0]*0.15
        else:
            fuse_NE_speed[0] = 0
            
        if math.fabs(fuse_NE_speed[1]) >0.1:
            odom_NE_distance[1] += Dv_distance[1]
            if math.fabs(Dv_distance[1] - Dgps_distance[1])<1:
                output_location[1] = odom_NE_distance[1]*0.15 + gpsodom_location[1]*0.85
            else:
                output_location[1] = odom_NE_distance[1]*0.85 + gpsodom_location[1]*0.15
        else:
            fuse_NE_speed[1] = 0
            
            
        fuse_path.header.frame_id = "world"
        fuse_path.header.stamp = rospy.Time.now()
        p = PoseStamped()
        p.pose.position.x =output_location[0]
        p.pose.position.y =output_location[1]
        p.pose.position.z = 0.0
        fuse_path.poses.append(p)
        Path_Pub.publish(fuse_path)

        br.sendTransform((output_location[0],output_location[1],0),tf.transformations.quaternion_from_euler(0,0,real_yaw),rospy.Time.now(),"base_link","world")

        odom.header.frame_id="world"
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = output_location[0]
        odom.pose.pose.position.y = output_location[1]
        odom.pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0,0,real_yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
         
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = translate_speed[0]
        odom.twist.twist.linear.y = translate_speed[1]
        odom.twist.twist.angular.z = angular_z
       
        odom_Pub.publish(odom)

        print ("N_x:%f\n"%(gpsodom_location[0]))
        print ("E_y:%f\n"%(gpsodom_location[1]))
        print ("N_vx:%f\n"%(fuse_NE_speed[0]))
        print ("E_vy:%f\n"%(fuse_NE_speed[1]))

        last_time = now_time
        last_gps_location[0] =  gpsodom_location[0]
        last_gps_location[1] =  gpsodom_location[1] 
        rate.sleep()

#main function
if __name__=="__main__":
    try:
        rospy.init_node('imu_gps_location',anonymous=True)
        rospy.Subscriber("/gps_odom",Odometry,callback_gpsodom)
        rospy.Subscriber("/vel",TwistStamped ,callback_vel)
        rospy.Subscriber("/imu/imu_yaw",Odometry,callable_imu_yaw)
        Path_Pub = rospy.Publisher("/fuse_path",Path,queue_size=10)
        odom_Pub = rospy.Publisher("/odom",Odometry,queue_size=10)
        fuselocation_tf()
        rospy.spin()

    except KeyboardInterrupt:
        
        print("Shutting down")
