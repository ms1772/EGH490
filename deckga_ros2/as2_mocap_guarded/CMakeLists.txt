cmake_minimum_required(VERSION 3.8)
project(as2_mocap_guarded)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(as2_core REQUIRED)
# as2_state_estimator ships plugin_base.hpp under ${prefix}/include/as2_state_estimator
# but its generated Config.cmake exports no include dirs, libraries or dependencies
# (it is a plain ament marker package). The include path is supplied by as2_core,
# which exports ${prefix}/include. Requiring it here is still worthwhile: it makes
# the build fail early and legibly if the base package is missing rather than at
# the #include.
find_package(as2_state_estimator REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(PLUGIN_LIBRARY_NAME mocap_pose_guarded)

# The library name is load-bearing: plugins.xml declares
# <library path="mocap_pose_guarded"> and pluginlib turns that into
# libmocap_pose_guarded.so. Renaming the target breaks plugin loading.
add_library(${PLUGIN_LIBRARY_NAME} SHARED
  src/mocap_pose_guarded.cpp
)

target_include_directories(${PLUGIN_LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PLUGIN_LIBRARY_NAME}
  rclcpp
  pluginlib
  as2_core
  geometry_msgs
  std_msgs
  nav_msgs
  mocap4r2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Registers this package in
#   share/ament_index/resource_index/as2_state_estimator__pluginlib__plugin/
# so the stock as2_state_estimator node's ClassLoader finds mocap_pose_guarded::Plugin
# without any modification to the installed as2_state_estimator package.
pluginlib_export_plugin_description_file(as2_state_estimator plugins.xml)

install(TARGETS ${PLUGIN_LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# `plugins/mocap_pose_guarded/config/plugin_default.yaml` mirrors the directory
# layout the stock state_estimator_launch.py expects, so the same relative path
# works if the plugin is ever vendored into as2_state_estimator itself.
install(DIRECTORY
  launch
  config
  plugins
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PLUGIN_LIBRARY_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  as2_core
  geometry_msgs
  std_msgs
  nav_msgs
  mocap4r2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
