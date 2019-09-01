cmake_minimum_required(VERSION 2.8.3)

project(robot_arm)

#set(LINK_DIR /usr/lib)

#find_package(catkin REQUIRED COMPONENTS
#	roscpp
#	rospy
#	std_msgs
#	genmsg
#	actionlib_msgs
#	actionlib
#)
find_package(roslaunch)

#generate_messages(
#	DEPENDENCIES actionlib_msgs std_msgs
#)
#catkin_package(
#	CATKIN_DEPENDS actionlib_msgs
#)

#include_directories(
#	include/rob_control
#	${catkin_INCLUDE_DIRS}
#)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#add_executable(robot_arm src/rob_control.cpp src/rob_comm.cpp)
#target_link_libraries(robot_arm ${catkin_LIBRATIES})
