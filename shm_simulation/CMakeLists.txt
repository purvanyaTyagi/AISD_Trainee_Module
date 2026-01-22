cmake_minimum_required(VERSION 3.8)
project(shm_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Energy.msg"
  "srv/ApplyForce.srv"
  "srv/UpdateParameters.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(
  visualization_node
  rclcpp
  std_msgs
  visualization_msgs
)

rosidl_target_interfaces(
  visualization_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

install(TARGETS visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(physics_node src/physics_node.cpp)
ament_target_dependencies(
  physics_node
  rclcpp
  std_msgs
)

rosidl_target_interfaces(
  physics_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

install(TARGETS physics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
