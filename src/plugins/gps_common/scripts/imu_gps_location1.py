#!/usr/bin/python2
# coding=gbk
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


def callback_gpsodom(data):
    global gpsodom_location
    global bag_raw_x,bag_raw_y
    raw_x  = data.pose.pose.position.x - 0.01
    raw_y = data.pose.pose.position.y- 0.03
    bag_raw_x.append(raw_x)
    bag_raw_y.append(raw_y)
    if len(bag_raw_x)>10 :
        del bag_raw_x[0]
        del bag_raw_y[0]
    gpsodom_location[0] = np.mean(bag_raw_x)
    gpsodom_location[1] = np.mean(bag_raw_y)

def callback_vel(data):
    global NE_speed
    NE_speed[0] = data.twist.linear.x
    NE_speed[1] = data.twist.linear.y

def  fuselocation_tf():
    global real_yaw
    global NE_speed
    global gpsodom_location
    global imu_speed
    global output_location
    global fuse_path 
    global Path_Pub

    translate_speed = [0.0,0.0]
    fuse_NE_speed = [0.0,0.0]
    odom_NE_distance = [0.0,0.0]
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(50)
    Dt = 0.02 
    while not rospy.is_shutdown(): 
        try:
            (trans,rot) = listener.lookupTransform('/imu_link', '/world', rospy.Time(0))
            real_yaw = rot[2]
            print" real_yaw get%f"%(real_yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print"no real_yaw get"
            continue
        translate_speed[0] =  imu_speed[0]*math.cos(real_yaw)-imu_speed[1]*math.sin(real_yaw)
        translate_speed[1] =  imu_speed[0]*math.sin(real_yaw)+imu_speed[1]*math.cos(real_yaw)
        fuse_NE_speed[0] = translate_speed[0] * 0 + NE_speed[0] *1
        fuse_NE_speed[1] = translate_speed[1] * 0 + NE_speed[1] *1
        if math.fabs(fuse_NE_speed[0]) >0.1:
            odom_NE_distance[0] += fuse_NE_speed[0]*Dt
            output_location[0] = odom_NE_distance[0]*0.15 + gpsodom_location[0]*0.85
        else:
            fuse_NE_speed[0] = 0
            #output_location[0] = gpsodom_location[0]
        if math.fabs(fuse_NE_speed[1]) >0.1:
            odom_NE_distance[1] += fuse_NE_speed[1]*Dt
            output_location[1] = odom_NE_distance[1]*0.15 + gpsodom_location[1]*0.85
        else:
            fuse_NE_speed[1] = 0
            #output_location[1] = gpsodom_location[1]
            
        fuse_path.header.frame_id = "world"
        fuse_path.header.stamp = rospy.Time.now()
        p = PoseStamped()
        p.pose.position.x =output_location[0]
        p.pose.position.y =output_location[1]
        p.pose.position.z = 0.0
        fuse_path.poses.append(p)
        Path_Pub.publish(fuse_path)

        br.sendTransform((output_location[0],output_location[1],0),tf.transformations.quaternion_from_euler(0,0,-real_yaw),rospy.Time.now(),"base_link","world")
        print "N_x:%f\n"%(gpsodom_location[0])
        print "E_y:%f\n"%(gpsodom_location[1])
        print "N_vx:%f\n"%(fuse_NE_speed[0])
        print "E_vy:%f\n"%(fuse_NE_speed[1])
        rate.sleep()

#main function
if __name__=="__main__":
    try:
        rospy.init_node('imu_gps_location',anonymous=True)
        rospy.Subscriber("/imu/data",Imu,callback_imu)
        rospy.Subscriber("/gps_odom",Odometry,callback_gpsodom)
        rospy.Subscriber("/vel",TwistStamped ,callback_vel)
        Path_Pub = rospy.Publisher("/fuse_path",Path,queue_size=10)
        fuselocation_tf()
        rospy.spin()

    except KeyboardInterrupt:
        
        print("Shutting down")
