cmake_minimum_required(VERSION 3.8)
project(mass_spring_cpp)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =========================
# Physics Node
# =========================
add_executable(simple_shm_node src/simple_shm_node.cpp)
ament_target_dependencies(simple_shm_node
  rclcpp
  std_msgs
)

# =========================
# Visualizer Node
# =========================
add_executable(visualizer_node src/visualizer_node.cpp)
ament_target_dependencies(visualizer_node
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  simple_shm_node
  visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
