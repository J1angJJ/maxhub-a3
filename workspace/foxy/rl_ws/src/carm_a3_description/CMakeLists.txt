cmake_minimum_required(VERSION 3.5)
project(carm_a3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  bind
  docs
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
