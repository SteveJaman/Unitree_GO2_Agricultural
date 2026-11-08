cmake_minimum_required(VERSION 3.8)
project(legged_mppi_critic)
# Needs nav2_mppi_controller (1.1.20): built in the go2-3d image, not go2-lidar3d.
# See tests/traversability/run_all.sh (MPPI_WS) for the build and how trials run it.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)

add_library(legged_mppi_critic SHARED src/step_over_critic.cpp)
target_include_directories(legged_mppi_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(legged_mppi_critic rclcpp nav2_costmap_2d nav2_mppi_controller pluginlib)

# MPPI loads critics as "mppi::critics::<Name>" through its own plugin loader.
pluginlib_export_plugin_description_file(nav2_mppi_controller plugin.xml)

install(TARGETS legged_mppi_critic
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(legged_mppi_critic)
ament_package()
