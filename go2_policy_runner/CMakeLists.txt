# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(go2_policy_runner LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(go2_policy_runner_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
include(cmake/onnxruntime.cmake)

# yaml-cpp exports different target names depending on version / distro
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# ---------------------------------------------------------------------------------------------
# Core: policy (IsaacLab env port + ONNX), remote controller, FSM. No ROS dependency.
# ---------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/remote/unitree_remote.cpp
  src/remote/remote_dsl.cpp
  src/policy/observation_manager.cpp
  src/policy/action_manager.cpp
  src/policy/policy_env.cpp
  src/policy/onnx_policy.cpp
  src/fsm/modes.cpp
  src/fsm/mode_manager.cpp
  src/monitor/monitor_model.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen ${YAML_CPP_TARGET}
  PRIVATE onnxruntime::onnxruntime)

# ---------------------------------------------------------------------------------------------
# ROS 2 layer: components policy_controller and remote_teleop
# ---------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_ros SHARED
  src/ros/unitree_lowcmd.cpp
  src/ros/motion_switcher_client.cpp
  src/ros/policy_controller_node.cpp
  src/ros/remote_teleop_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_ros PUBLIC
  rclcpp rclcpp_components ament_index_cpp geometry_msgs unitree_go unitree_api
  go2_policy_runner_interfaces)

rclcpp_components_register_nodes(${PROJECT_NAME}_ros
  "go2_policy_runner::ros::PolicyControllerNode")
rclcpp_components_register_node(${PROJECT_NAME}_ros
  PLUGIN "go2_policy_runner::ros::RemoteTeleopNode"
  EXECUTABLE remote_teleop)

# Standalone controller with safe shutdown (damping before the context goes away)
add_executable(policy_controller src/nodes/policy_controller_main.cpp)
target_link_libraries(policy_controller ${PROJECT_NAME}_ros)

# Terminal UI showing the controller and robot state (ncurses)
add_executable(robot_monitor src/monitor/monitor_main.cpp src/monitor/monitor_node.cpp)
target_include_directories(robot_monitor PRIVATE ${CURSES_INCLUDE_DIRS})
target_link_libraries(robot_monitor ${PROJECT_NAME}_core ${CURSES_LIBRARIES})
ament_target_dependencies(robot_monitor
  rclcpp geometry_msgs diagnostic_msgs unitree_go go2_policy_runner_interfaces)

# ---------------------------------------------------------------------------------------------
# install
# ---------------------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS policy_controller robot_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch policies DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/dds_env.sh scripts/fetch_onnxruntime.sh DESTINATION share/${PROJECT_NAME}/scripts)

if(ONNXRUNTIME_VENDORED)
  # keeps the libonnxruntime.so -> .so.1 -> .so.1.x.y symlinks
  install(DIRECTORY "${ONNXRUNTIME_LIBRARY_DIR}/" DESTINATION lib
    FILES_MATCHING PATTERN "libonnxruntime.so*")
endif()

# ---------------------------------------------------------------------------------------------
# tests (core only, no robot / simulator needed)
# ---------------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_remote test/test_remote.cpp)
  target_link_libraries(test_remote ${PROJECT_NAME}_core)
  ament_add_gtest(test_policy_env test/test_policy_env.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_policy_env ${PROJECT_NAME}_core)
  ament_add_gtest(test_fsm test/test_fsm.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_fsm ${PROJECT_NAME}_core)
  ament_add_gtest(test_monitor test/test_monitor.cpp)
  target_link_libraries(test_monitor ${PROJECT_NAME}_core)
endif()

ament_package()
