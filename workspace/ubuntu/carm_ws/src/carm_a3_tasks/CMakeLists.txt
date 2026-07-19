cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_tasks)

find_package(catkin REQUIRED COMPONENTS
  carm_a3_motion
  rospy
  roslib
  tf
)

catkin_package(
  CATKIN_DEPENDS carm_a3_motion rospy roslib tf
)

catkin_install_python(PROGRAMS
  scripts/grasp_init.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
