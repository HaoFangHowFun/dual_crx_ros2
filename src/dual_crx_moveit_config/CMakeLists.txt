cmake_minimum_required(VERSION 3.8)
project(dual_crx_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
