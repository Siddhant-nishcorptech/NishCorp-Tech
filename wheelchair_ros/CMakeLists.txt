cmake_minimum_required(VERSION 3.8)
project(wheelchair)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY wheelchair config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/joint_position_controller.py
  scripts/motor_controller.py
  scripts/joint_controller.py
  scripts/teleop_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
