cmake_minimum_required(VERSION 3.8)
project(lv2_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(turtlesim REQUIRED)  

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/Znamenitost.msg"
  DEPENDENCIES turtlesim
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
