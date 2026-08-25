cmake_minimum_required(VERSION 3.8)
project(dual_crx_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
