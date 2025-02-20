#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/four_wd_ws/devel/setup.bash

gnome-terminal -x bash -c "roslaunch robot_launch launch_robot.launch;exec bash"
sleep 2
gnome-terminal -x bash -c "roslaunch robot_launch launch_location.launch;exec bash"
sleep 3
gnome-terminal -x bash -c "roslaunch mavlink_ros mavlink.launch;exec bash"
sleep 3
gnome-terminal -x bash -c "roslaunch four_ws_navigation four_ws_navigation.launch;exec bash"
sleep 3
gnome-terminal -x bash -c "roslaunch simple_robot robot.launch;exec bash"
sleep 3

