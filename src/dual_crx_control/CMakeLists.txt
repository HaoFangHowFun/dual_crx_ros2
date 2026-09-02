cmake_minimum_required(VERSION 3.8)
project(dual_crx_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(dual_crx_demo src/dual_crx_demo.cpp)
ament_target_dependencies(
  dual_crx_demo
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS dual_crx_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dual_crx_control_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_control_core test/test_control_core.py)
  ament_add_pytest_test(test_workcell test/test_workcell.py)
endif()

ament_package()
