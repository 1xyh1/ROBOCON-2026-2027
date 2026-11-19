cmake_minimum_required(VERSION 3.8)
project(robocon_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetObservation.msg"
  "msg/StartSignalEvent.msg"
  "msg/LocalizationQuality.msg"
  "msg/MissionStatus.msg"
  "msg/TimeAlignmentStatus.msg"
  "action/ObserveTarget.action"
  "action/Relocalize.action"
  "srv/ReconcileMission.srv"
  DEPENDENCIES action_msgs geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
