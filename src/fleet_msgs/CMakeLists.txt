cmake_minimum_required(VERSION 3.20)
project(fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/TaskStatus.msg"
  "msg/RobotStatus.msg"
  "msg/Detection.msg"
  "srv/SubmitTask.srv"
  "srv/CancelTask.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
