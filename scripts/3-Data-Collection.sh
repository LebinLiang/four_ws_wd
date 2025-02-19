#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/four_wd_ws/devel/setup.bash


gnome-terminal -x bash -c "rosbag record -o dataset /odom /target_odom;exec bash"

# gnome-terminal -x bash -c "rosbag record -o dataset /target_path /fuse_path;exec bash"



