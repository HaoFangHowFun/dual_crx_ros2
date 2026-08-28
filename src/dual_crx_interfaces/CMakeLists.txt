cmake_minimum_required(VERSION 3.8)
project(dual_crx_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemState.msg"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  "srv/Heartbeat.srv"
  "srv/JointJog.srv"
  "srv/CartesianJog.srv"
  "srv/SoftwareStop.srv"
  "action/JointPosition.action"
  "action/CartesianPose.action"
  DEPENDENCIES action_msgs builtin_interfaces control_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
