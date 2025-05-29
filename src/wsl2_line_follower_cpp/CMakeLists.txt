cmake_minimum_required(VERSION 3.5)
project(wsl2_line_follower_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(line_error_node src/line_error_node.cpp)
ament_target_dependencies(line_error_node rclcpp sensor_msgs OpenCV)

install(TARGETS
  line_error_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()