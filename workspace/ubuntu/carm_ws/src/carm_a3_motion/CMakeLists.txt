cmake_minimum_required(VERSION 3.0.2)
project(carm_a3_motion)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

add_service_files(
  FILES
  GetCartesianSnapshot.srv
  GetExtendedState.srv
  GetJointSnapshot.srv
  GetToolInfo.srv
  JogJoint.srv
  MoveFlowPose.srv
  MoveJoint.srv
  MoveLineJoint.srv
  MoveLinePose.srv
  MovePose.srv
  SetCollisionConfig.srv
  SetControlMode.srv
  SetGripper.srv
  SetSpeed.srv
  SetToolIndex.srv
  SolveFKArray.srv
  SolveFK.srv
  SolveIKArray.srv
  SolveIK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
  CATKIN_DEPENDS message_runtime rospy roscpp sensor_msgs std_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${arm_control_sdk_INCLUDE_DIRS}
)

add_executable(safe_motion_node
  src/safe_motion_node.cpp
)

add_executable(official_topic_motion_node
  src/official_topic_motion_node.cpp
)

add_dependencies(safe_motion_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(official_topic_motion_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(safe_motion_node
  ${catkin_LIBRARIES}
  ${arm_control_sdk_LIBRARIES}
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoNet.so.71"
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoUtil.so.71"
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoFoundation.so.71"
)

target_link_libraries(official_topic_motion_node
  ${catkin_LIBRARIES}
  ${arm_control_sdk_LIBRARIES}
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoNet.so.71"
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoUtil.so.71"
  "${ARM_CONTROL_SDK_POCO_LIB_DIR}/libPocoFoundation.so.71"
)

set_target_properties(safe_motion_node PROPERTIES
  BUILD_RPATH "${ARM_CONTROL_SDK_LIB_DIR};${ARM_CONTROL_SDK_POCO_LIB_DIR}"
  INSTALL_RPATH "${ARM_CONTROL_SDK_LIB_DIR};${ARM_CONTROL_SDK_POCO_LIB_DIR}"
)

set_target_properties(official_topic_motion_node PROPERTIES
  BUILD_RPATH "${ARM_CONTROL_SDK_LIB_DIR};${ARM_CONTROL_SDK_POCO_LIB_DIR}"
  INSTALL_RPATH "${ARM_CONTROL_SDK_LIB_DIR};${ARM_CONTROL_SDK_POCO_LIB_DIR}"
)

catkin_install_python(PROGRAMS
  scripts/motion_cli.py
  scripts/py_ws_motion_node.py
  scripts/raw_ws_motion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
