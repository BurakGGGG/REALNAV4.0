cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz maps scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav2_motor_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
