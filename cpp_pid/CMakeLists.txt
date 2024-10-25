cmake_minimum_required(VERSION 3.8)
project(cpp_pid)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(dbw_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(include)

# Create executable
add_executable(${PROJECT_NAME}_main src/main.cpp src/server.cpp src/environment.cpp src/controller.cpp)

# Set dependencies for the target
ament_target_dependencies(${PROJECT_NAME}_main rclcpp std_msgs vectornav_msgs dbw_msgs)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
