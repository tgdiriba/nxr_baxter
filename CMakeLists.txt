cmake_minimum_required(VERSION 2.8.3)
project(nxr_baxter)

find_package(catkin
  REQUIRED
  rospy 
  roscpp 
  roslib
  actionlib

  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  baxter_core_msgs
  skeletonmsgs_nu
  nxr_baxter_msgs
  
  baxter_interface

  tf
  dynamic_reconfigure
  cv_bridge
	
	ar_track_alvar
)

catkin_python_setup()

# generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs control_msgs trajectory_msgs baxter_core_msgs skeletonmsgs_nu)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  roslib
  actionlib

  sensor_msgs
  std_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  baxter_core_msgs
  skeletonmsgs_nu
  message_runtime
  nxr_baxter_msgs
  
  baxter_interface

  tf
  dynamic_reconfigure
  cv_bridge

	ar_track_alvar
)

# include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(nxr_baxter_demo_package src/moveit_test_0.cpp)
# target_link_libraries(nxr_baxter_demo_package ${catkin_LIBRARIES})
# add_dependencies(nxr_baxter_demo_package ${catkin_EXPORTED_TARGETS})
