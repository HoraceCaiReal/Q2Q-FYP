cmake_minimum_required(VERSION 3.8)
project(robot_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/arm_executor.py
  scripts/arm_plan.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
