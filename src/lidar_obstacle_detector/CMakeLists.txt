cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_detector)

# 기본 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 실행 파일 생성
add_executable(obstacle_labeler_node src/obstacle_labeler_node.cpp)

# 의존성 연결
ament_target_dependencies(obstacle_labeler_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(obstacle_labeler_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(obstacle_labeler_node
  ${OpenCV_LIBS}
)

install(TARGETS obstacle_labeler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()