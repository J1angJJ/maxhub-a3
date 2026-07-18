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

set(ARM_CONTROL_SDK_LIB_DIR "${arm_control_sdk_ROOT_DIR}/lib")
set(ARM_CONTROL_SDK_POCO_LIB_DIR "${arm_control_sdk_ROOT_DIR}/poco/lib")

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf2_ros
)

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
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoNet.so.71"
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoUtil.so.71"
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoFoundation.so.71"
)

set_target_properties(readonly_state_node PROPERTIES
  BUILD_RPATH "${ARM_CONTROL_SDK_LIB_DIR};${ARM_CONTROL_SDK_POCO_LIB_DIR}"
  INSTALL_RPATH "${ARM_CONTROL_SDK_LIB_DIR};${ARM_CONTROL_SDK_POCO_LIB_DIR}"
)

add_dependencies(readonly_state_node ${catkin_EXPORTED_TARGETS})
