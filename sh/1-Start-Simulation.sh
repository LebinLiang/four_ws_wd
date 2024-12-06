#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/four_wd_ws/devel/setup.bash


gnome-terminal -x bash -c "roslaunch taurus_control simulation.launch;exec bash"
sleep 3
gnome-terminal -x bash -c "roslaunch four_ws_navigation four_ws_navigation.launch;exec bash"
