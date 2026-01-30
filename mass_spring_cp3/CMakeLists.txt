cmake_minimum_required(VERSION 3.8)
project(mass_spring_cp3)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Create executable
add_executable(mass_spring_kb src/mass_spring_kb.cpp)

# Link dependencies
ament_target_dependencies(mass_spring_kb
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

# Install executable
install(
  TARGETS mass_spring_kb
  DESTINATION lib/${PROJECT_NAME}
)

# Required for ROS 2 packages
ament_package()

