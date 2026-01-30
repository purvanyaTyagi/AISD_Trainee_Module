cmake_minimum_required(VERSION 3.8)
project(mass_spring_cp2)

# Enable warnings (good practice, but simple)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Create executable for Checkpoint 2
add_executable(mass_spring_energy_node src/mass_spring_energy_node.cpp)

# Link dependencies
ament_target_dependencies(
  mass_spring_energy_node
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

# Install executable
install(TARGETS
  mass_spring_energy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

