cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate the Service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetVelocity.srv"
)

# 3. Robot Controller Node
add_executable(robot_controller_node src/robot_controller.cpp)
ament_target_dependencies(robot_controller_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs
)
# Link the service typesupport to the node
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(robot_controller_node "${cpp_typesupport_target}")

# 4. Teleop Node
add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node 
  rclcpp 
  geometry_msgs
)
target_link_libraries(teleop_node "${cpp_typesupport_target}")

# 5. Install Instructions (Cleaned up)

# Install directories (URDF, Launch, Worlds, Models) to share/
install(DIRECTORY 
  urdf 
  launch 
  worlds 
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install C++ executables to lib/
install(TARGETS 
  robot_controller_node
  teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts (if you have them)
if(EXISTS "${PROJECT_SOURCE_DIR}/scripts")
  install(PROGRAMS
    scripts/robot_controller.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 6. Environment setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 7. REQUIRED: This must be the very last thing in the file
ament_package()