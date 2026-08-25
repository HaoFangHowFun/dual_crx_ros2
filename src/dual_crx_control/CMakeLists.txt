cmake_minimum_required(VERSION 3.8)
project(dual_crx_control)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dual_crx_demo src/dual_crx_demo.cpp)
ament_target_dependencies(
  dual_crx_demo
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS dual_crx_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
