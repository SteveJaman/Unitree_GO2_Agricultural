cmake_minimum_required(VERSION 3.8)
project(go2_integration_pkg)

# -----------------------------------------------------------------------------
# Language standards
# -----------------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Build tool dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# -----------------------------------------------------------------------------
# Message and core ROS 2 dependencies
# -----------------------------------------------------------------------------
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# -----------------------------------------------------------------------------
# Python bridge dependencies
# -----------------------------------------------------------------------------
find_package(cv_bridge REQUIRED)

# -----------------------------------------------------------------------------
# Navigation and mapping dependencies
# -----------------------------------------------------------------------------
find_package(nav2_common REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(slam_toolbox REQUIRED)

# -----------------------------------------------------------------------------
# Python package
# -----------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------------------------------------------------------------
# Python executable nodes
#
# Each script is installed to lib/<package>/ so that it can be launched
# with `ros2 run go2_integration_pkg <script>.py` and referenced from
# launch files by its executable name.
# -----------------------------------------------------------------------------
# using the following configuration so no need to add the python file here 
file(GLOB PYTHON_SCRIPTS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/*.py"
)

install(PROGRAMS
  ${PYTHON_SCRIPTS} # added from the above PYTHON_SCRIPTS
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Launch files
# -----------------------------------------------------------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Optional config folder
# -----------------------------------------------------------------------------
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()