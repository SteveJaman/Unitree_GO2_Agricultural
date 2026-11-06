cmake_minimum_required(VERSION 3.8)
project(semantic_traversability_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_semantic_msgs REQUIRED)

set(height_band_deps
  nav2_costmap_2d
  pluginlib
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
)

set(semantic_deps
  nav2_costmap_2d
  pluginlib
  rclcpp
  go2_semantic_msgs
)

add_library(height_band_layer SHARED
  src/height_band_layer.cpp
)
ament_target_dependencies(height_band_layer ${height_band_deps})
target_include_directories(height_band_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(semantic_costmap_layer SHARED
  src/semantic_costmap_layer.cpp
)
ament_target_dependencies(semantic_costmap_layer ${semantic_deps})
target_include_directories(semantic_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_costmap_2d height_band_plugin.xml)

install(TARGETS height_band_layer semantic_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
