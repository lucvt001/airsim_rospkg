cmake_minimum_required(VERSION 3.0.2)
project(airsim_rospkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GPSYaw.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES airsim_rospkg
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs message_runtime
  #  DEPENDS system_lib
  )
  
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

