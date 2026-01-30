cmake_minimum_required(VERSION 3.8)
project(spring_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Interface generation
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSpringParams.srv"
)

# Node
add_executable(tryrviz_node src/try.cpp)

ament_target_dependencies(tryrviz_node
  rclcpp
  std_msgs
  visualization_msgs
)

# 🔴 THIS IS CRITICAL
rosidl_target_interfaces(tryrviz_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install
install(TARGETS
  tryrviz_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export interfaces at runtime
ament_export_dependencies(rosidl_default_runtime)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
