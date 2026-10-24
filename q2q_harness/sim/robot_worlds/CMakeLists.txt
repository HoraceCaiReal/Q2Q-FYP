cmake_minimum_required(VERSION 3.8)
project(robot_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# maps/ 同时保存历史 SLAM 图与显式标注的 ground_truth 图。
# 当前基础导航默认用 q2q_warehouse_ground_truth.yaml；生成脚本也一起安装。
# 没有这个安装规则，默认参数会指向一个不存在的文件，map_server 起不来。
install(DIRECTORY maps scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
