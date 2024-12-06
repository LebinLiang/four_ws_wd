cmake_minimum_required(VERSION 3.0.2)
project(fourws_viz)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  nav_msgs
  gazebo_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_transfer
  CATKIN_DEPENDS  geometry_msgs roscpp std_msgs std_srvs nav_msgs gazebo_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 include/fourws_viz
  ${catkin_INCLUDE_DIRS}
)

add_executable(fourws_viz src/fourws_viz.cpp)
add_dependencies(fourws_viz ${catkin_EXPORTED_TARGETS})
target_link_libraries(fourws_viz ${catkin_LIBRARIES}  )
