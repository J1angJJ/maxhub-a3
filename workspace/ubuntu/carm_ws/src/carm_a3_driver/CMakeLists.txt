cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)

if(DEFINED ENV{arm_control_sdk_DIR})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{arm_control_sdk_DIR}")
endif()

set(VENDORED_ARM_CONTROL_SDK
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/arm_control_sdk"
)
if(EXISTS "${VENDORED_ARM_CONTROL_SDK}/lib/cmake/arm_control_sdk")
  list(APPEND CMAKE_PREFIX_PATH "${VENDORED_ARM_CONTROL_SDK}")
endif()

find_package(arm_control_sdk REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${arm_control_sdk_INCLUDE_DIRS}
)

add_executable(readonly_state_node
  src/readonly_state_node.cpp
)

target_link_libraries(readonly_state_node
  ${catkin_LIBRARIES}
  ${arm_control_sdk_LIBRARIES}
)

add_dependencies(readonly_state_node ${catkin_EXPORTED_TARGETS})
