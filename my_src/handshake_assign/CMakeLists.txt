cmake_minimum_required(VERSION 3.8)
project(handshake_assign)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vcu_node src/vcu_node.cpp)
ament_target_dependencies(vcu_node rclcpp std_msgs)

add_executable(ai_node src/ai_node.cpp)
ament_target_dependencies(ai_node rclcpp std_msgs)

install(TARGETS
  vcu_node
  ai_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
