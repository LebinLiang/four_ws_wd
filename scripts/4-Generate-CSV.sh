#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/four_wd_ws/devel/setup.bash

rostopic echo -b dataset_line.bag -p /odom > odom_pid.csv
# rostopic echo -b dataset_2022-04-30-09-45-57.bag -p /cmd_vel > cmd_vel7_pid.csv
rostopic echo -b dataset_line.bag -p /target_odom > target_odom_pid.csv



