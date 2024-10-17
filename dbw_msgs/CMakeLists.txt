cmake_minimum_required(VERSION 3.5)
project(dbw_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dbw.msg"
  "msg/ASC1.msg"
  "msg/CCVS.msg"
  "msg/EAC1.msg"
  "msg/EBC1.msg"
  "msg/EEC1.msg"
  "msg/EEC2.msg"
  "msg/ETC2.msg"
  "msg/ETC5.msg"
  "msg/HRW.msg"
  "msg/LC.msg"
  "msg/LD.msg"
  "msg/Shutdown.msg"
  "msg/TCO1.msg"
  "msg/VDC2.msg"
  "msg/VEP.msg"
  "msg/VP2.msg"
 )
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
