cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_vision)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(v4l2_camera_node
  src/v4l2_camera_node.cpp
)

target_link_libraries(v4l2_camera_node
  ${catkin_LIBRARIES}
)

add_dependencies(v4l2_camera_node ${catkin_EXPORTED_TARGETS})
