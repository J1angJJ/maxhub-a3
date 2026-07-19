cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  bind
  launch
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
