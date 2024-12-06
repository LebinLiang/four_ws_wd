cmake_minimum_required(VERSION 3.0.2)
project(twist_keyboard)

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
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_transfer
  CATKIN_DEPENDS  geometry_msgs roscpp std_msgs std_srvs nav_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 include/twist_keyboard
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_keyboard src/twist_keyboard.cpp)
add_dependencies(twist_keyboard ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_keyboard ${catkin_LIBRARIES}  )
