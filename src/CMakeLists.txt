cmake_minimum_required(VERSION 3.5)
project(spring_mass_damper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# --- 1. COMPILE THE CODE ---
# Your screenshot confirms the file is named "spring_mass_sim.cpp"
add_executable(spring_mass_sim src/spring_mass_sim.cpp)

ament_target_dependencies(spring_mass_sim
  rclcpp
  visualization_msgs
  geometry_msgs
  std_srvs
)

# --- 2. INSTALL THE EXECUTABLE (THIS IS WHAT YOU ARE MISSING) ---
install(TARGETS
  spring_mass_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()