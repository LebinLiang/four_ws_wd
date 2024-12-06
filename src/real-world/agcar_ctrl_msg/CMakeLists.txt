cmake_minimum_required(VERSION 3.0.2)
project(agcar_ctrl_msg)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES 
agcar_ctrl.msg 
pump_ctrl.msg
mavlink_msgid.msg
agcar_odom.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost)
