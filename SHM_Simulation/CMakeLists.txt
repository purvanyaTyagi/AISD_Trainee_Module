cmake_minimum_required(VERSION 3.8)
project(SHM_Simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node rclcpp std_msgs visualization_msgs)
install(TARGETS
  visualization_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(physics_node src/physics_node.cpp)

ament_target_dependencies(
  physics_node
  rclcpp
  std_msgs
)

install(TARGETS
  physics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
