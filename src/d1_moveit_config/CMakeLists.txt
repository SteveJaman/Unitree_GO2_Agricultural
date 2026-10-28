cmake_minimum_required(VERSION 3.8)
project(d1_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/inspect_plant.py DESTINATION lib/${PROJECT_NAME})

ament_package()
