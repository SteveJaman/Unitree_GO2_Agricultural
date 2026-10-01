cmake_minimum_required(VERSION 3.8)
project(go2_integration_pkg)

# ---------- Default to C++17 / C11 ----------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- Find dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---------- Install Python package ----------
# The Python module folder must match the package name.
ament_python_install_package(${PROJECT_NAME})

# ---------- Install executables ----------
# These land in install/go2_integration_pkg/lib/go2_integration_pkg/
install(PROGRAMS
  ${PROJECT_NAME}/cloud_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Install launch files ----------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------- Install resource files ----------
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ---------- Tests ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()