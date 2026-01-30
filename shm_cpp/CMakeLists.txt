cmake_minimum_required(VERSION 3.8)
project(shm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(position_publisher src/position_publisher.cpp)
ament_target_dependencies(position_publisher rclcpp std_msgs)

add_executable(visualize_marker src/visualize_marker.cpp)
ament_target_dependencies(visualize_marker rclcpp std_msgs visualization_msgs geometry_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSpring.srv"
)

rosidl_target_interfaces(position_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  position_publisher
  visualize_marker
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
