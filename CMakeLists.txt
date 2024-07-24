cmake_minimum_required(VERSION 3.5)
project(starling_demos_cpp)

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

# TODO ignore clock skew errors TODO
# set(CMAKE_SKIP_CHECKING_BUILD_SYSTEM TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# executables

# Velocity based square demo
add_executable(square_vel src/square_vel.cpp)
ament_target_dependencies(starling_demos_cpp 
    rclcpp 
    px4_msgs 
    geometry_msgs 
    Eigen3 
    eigen3_cmake_module
)
install(TARGETS square_vel DESTINATION lib/${PROJECT_NAME})

add_executable(liftoff src/liftoff.cpp)
ament_target_dependencies(starling_demos_cpp 
    rclcpp 
    px4_msgs 
    geometry_msgs 
    Eigen3 
    eigen3_cmake_module
)
install(TARGETS liftoff DESTINATION lib/${PROJECT_NAME})

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
