cmake_minimum_required(VERSION 3.8)
project(rs_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExplorationBid.msg"
  "msg/ThermalDetection.msg"
  "msg/RgbDetection.msg"
  "msg/PersonMark.msg"
  "msg/PersonMarkArray.msg"
  "msg/CommsPacket.msg"
  "msg/MapLayer.msg"
  "msg/ObsBatch.msg"
  "msg/CommsBeacon.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
