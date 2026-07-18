cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_calibration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs std_srvs tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/aruco_handeye_sampler.py
  scripts/solve_handeye.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
