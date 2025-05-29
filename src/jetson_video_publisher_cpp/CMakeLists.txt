cmake_minimum_required(VERSION 3.5)
project(jetson_video_publisher_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher rclcpp sensor_msgs OpenCV)

install(TARGETS
  video_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()