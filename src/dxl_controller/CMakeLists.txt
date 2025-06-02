cmake_minimum_required(VERSION 3.5)
project(dxl_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Dynamixel SDK 라이브러리 경로 지정 (예: /usr/local/lib/libdxl_x64_cpp.so)
# 만약 SDK가 /usr/local/lib에 설치되어 있다면 아래처럼 찾기 가능
find_library(DYNAMIXEL_SDK_LIB
  NAMES dxl_x64_cpp dynamixel_sdk
  PATHS /usr/local/lib /usr/lib
  NO_DEFAULT_PATH
)

if(NOT DYNAMIXEL_SDK_LIB)
  message(FATAL_ERROR "Cannot find Dynamixel SDK library")
endif()

include_directories(
  include
  /usr/local/include/dynamixel_sdk
)

add_executable(dxl_controller_node
  src/dxl.cpp
  src/dxl_controller_node.cpp  # 실제 노드 진입점 파일명에 맞게 수정하세요
)

ament_target_dependencies(dxl_controller_node
  rclcpp
  std_msgs
)

target_link_libraries(dxl_controller_node
  ${DYNAMIXEL_SDK_LIB}
)

install(TARGETS dxl_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
