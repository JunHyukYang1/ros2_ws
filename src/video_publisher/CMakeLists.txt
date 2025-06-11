cmake_minimum_required(VERSION 3.5)
project(video_publisher)

# 기본 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_publisher_node src/video_publisher_node.cpp)

ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  video_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()