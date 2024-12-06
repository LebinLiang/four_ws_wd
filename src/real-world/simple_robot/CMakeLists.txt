cmake_minimum_required(VERSION 3.1)
project(simple_robot)

set(CMAKE_CXX_STANDARD 11)

#############################
#Dependencies
#############################
find_package(Threads)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    nav_msgs
    geometry_msgs
    agcar_ctrl_msg
    )
catkin_package(

INCLUDE_DIRS include

)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(simple_robot src/robot_node.cpp src/crc.cpp src/serial_device.cpp)
target_link_libraries(simple_robot
    ${CMAKE_THREAD_LIBS_INIT}
    ${catkin_LIBRARIES})
