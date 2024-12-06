#!/usr/bin/python3
##coding=utf-8
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

imu_Angle = [0,0,0]
initpose_Angle =  [0,0,0]
init_flag = 0
last_Angle =[0,0,0]
D_yaw = 0
angular_z = 0

def callback_imu(data):
    global imu_Angle
    global angular_z
    imu_Angle = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    angular_z = data.angular_velocity.z
    #print "imu_yaw:%f\n"%(imu_Angle[2])

def callback_initpose(data):
    global initpose_Angle
    global init_flag
    global D_yaw 
    initpose_Angle = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    init_flag = 1
    D_yaw = initpose_Angle[2] - imu_Angle[2]
    #print "[init_yaw_pose ]: %f\n" %(initpose_Angle[2])

def  imu_update():
    global imu_Angle
    global initpose_Angle
    global init_flag
    global Yaw_Pub
    global D_yaw
    global angular_z

    yaw_data = Odometry()
    yaw_data.header.frame_id = "world"
    yaw_data.child_frame_id =  "imu_link"
    rate = rospy.Rate(50)
    #if init_flag == 0:
    #    initpose_Angle[2] = 0.0174 * input("[INPUT]X axis to North(inº):")
    #    init_flag =1
    #print "[X axis to North ]: %f\n" %(initpose_Angle[2])
    
    while not rospy.is_shutdown(): 

        real_yaw = imu_Angle[2] +D_yaw
        if  real_yaw>3.14:
            real_yaw = -3.14 + (real_yaw - 3.14)
        if real_yaw < -3.14:
            real_yaw = 3.14 +(real_yaw + 3.14 )
        quat = tf.transformations.quaternion_from_euler(0,0,real_yaw)
        yaw_data.header.stamp = rospy.Time.now()
        yaw_data.pose.pose.position.x = 0
        yaw_data.pose.pose.position.y = 0
        yaw_data.pose.pose.position.z = 0 
        yaw_data.pose.pose.orientation.x= quat[0]
        yaw_data.pose.pose.orientation.y= quat[1]
        yaw_data.pose.pose.orientation.z= quat[2]
        yaw_data.pose.pose.orientation.w= quat[3]

        yaw_data.twist.twist.linear.x = 0
        yaw_data.twist.twist.linear.y = 0
        yaw_data.twist.twist.linear.z = 0
        yaw_data.twist.twist.angular.z = angular_z

        Yaw_Pub.publish(yaw_data)
        #print "imu_yaw:%f\n"%(imu_Angle[2])
        #print "real_yaw:%f\n"%(imu_Angle[2]+initpose_Angle[2])
        last_Angle = imu_Angle
        rate.sleep()

#main function
if __name__=="__main__":
    try:
	    rospy.init_node('imu_update',anonymous=True)
	    #rospy.Subscriber("/imu/data",Imu,callback_imu)
	    rospy.Subscriber("/imu",Imu,callback_imu)
	    rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,callback_initpose)
	    Yaw_Pub = rospy.Publisher("/imu/imu_yaw",Odometry,queue_size=1)
	    imu_update()
	    rospy.spin()

    except KeyboardInterrupt:
        
        print("Shutting down")
