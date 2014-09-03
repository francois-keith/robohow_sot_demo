cmake_minimum_required(VERSION 2.4.6)
project(robohow_sot_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  robohow_common_msgs
  dynamic_graph_bridge_msgs
  dynamic_graph_actionlib
  robohow_sot
 )

catkin_package(
 CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  robohow_common_msgs
  dynamic_graph_bridge_msgs
  dynamic_graph_actionlib
  robohow_sot
)

