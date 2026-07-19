cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/color_block_segmenter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
