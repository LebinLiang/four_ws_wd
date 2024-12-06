cmake_minimum_required(VERSION 2.8.3)

project(4ws_car)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

set(URDF_DIRS robots urdf urdf/base urdf/wheel urdf/suspension )

foreach(dir config launch meshes ${URDF_DIRS})
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
