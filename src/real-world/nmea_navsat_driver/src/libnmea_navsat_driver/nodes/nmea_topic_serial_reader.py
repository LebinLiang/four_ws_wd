# -*- coding: UTF-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Defines the main method for the nmea_topic_serial_reader executable."""

import serial   #引入串口模块

from nmea_msgs.msg import Sentence  #引入nmea_msgs/Sentence结构
import rospy    #引入ROS的Python接口模块

from libnmea_navsat_driver.driver import RosNMEADriver  #引入driver.py中的RosNMEADriver


def main():
    """Create and run the nmea_topic_serial_reader ROS node.

    Opens a serial device and publishes data from the device as nmea_msgs.msg.Sentence messages.

    ROS parameters:
        ~port (str): Path of the serial device to open.
        ~baud (int): Baud rate to configure the serial device.

    ROS publishers:
        nmea_sentence (nmea_msgs.msg.Sentence): Publishes each line from the open serial device as a new
            message. The header's stamp is set to the rostime when the data is read from the serial device.
    """
    rospy.init_node('nmea_topic_serial_reader')     #初始化ros节点名

    nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size=1)     #定义发布者

    serial_port = rospy.get_param('~port', '/dev/ttyUSB1')      #设置串口设备文件名
    serial_baud = rospy.get_param('~baud', 115200)                #设置波特率

    # Get the frame_id
    frame_id = RosNMEADriver.get_frame_id()                 #设置frame_id

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)      #调用python串口模块Serial
        while not rospy.is_shutdown():
            data = GPS.readline().strip()         #从串口读入数据

            sentence = Sentence()       #创建nmea_msgs/Sentence结构变量sentence
            sentence.header.stamp = rospy.get_rostime()     #设置sentence的时间戳，为当前时间
            sentence.header.frame_id = frame_id             #设置sentence的frame_id
            sentence.sentence = data                        #设置sentence的sentence变量。

            nmea_pub.publish(sentence)                      #发布sentence

    except rospy.ROSInterruptException:                     #异常处理
        GPS.close()  # Close GPS serial port
