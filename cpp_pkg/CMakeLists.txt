cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

#program
add_executable(my_first_node_cpp src/my_first_node_cpp.cpp)
ament_target_dependencies(my_first_node_cpp rclcpp)

install(TARGETS
  my_first_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
