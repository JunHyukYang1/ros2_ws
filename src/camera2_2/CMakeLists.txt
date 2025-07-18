cmake_minimum_required(VERSION 3.16)
project(camera2_2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

#build
add_executable(sub_wsl src/sub_wsl.cpp)
ament_target_dependencies(sub_wsl rclcpp sensor_msgs OpenCV)

# Install
install(TARGETS
  sub_wsl
  DESTINATION lib/${PROJECT_NAME})

ament_package()
