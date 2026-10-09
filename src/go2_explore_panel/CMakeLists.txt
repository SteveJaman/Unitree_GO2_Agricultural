cmake_minimum_required(VERSION 3.8)
project(go2_explore_panel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable Qt's meta-object compiler BEFORE add_library
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Tell AUTOMOC to also scan headers in include/
set(CMAKE_AUTOMOC_PATH_PREFIX ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Include the HEADER in the sources list so AUTOMOC sees it
add_library(go2_explore_panel SHARED
  src/explore_panel.cpp
  include/go2_explore_panel/explore_panel.hpp
)

target_include_directories(go2_explore_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(go2_explore_panel
  rviz_common::rviz_common
  Qt5::Core Qt5::Gui Qt5::Widgets
)

ament_target_dependencies(go2_explore_panel
  rclcpp rviz_common pluginlib std_msgs
)

install(TARGETS go2_explore_panel
  EXPORT export_go2_explore_panel
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(go2_explore_panel)
ament_export_targets(export_go2_explore_panel)

ament_package()
