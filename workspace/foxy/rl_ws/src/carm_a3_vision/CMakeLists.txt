cmake_minimum_required(VERSION 3.5)
project(carm_a3_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(v4l2_camera_node
  src/v4l2_camera_node.cpp
)

ament_target_dependencies(v4l2_camera_node
  camera_info_manager
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS v4l2_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/capture_image_sample
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
