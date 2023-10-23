cmake_minimum_required(VERSION 3.8)
project(image_flipper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_flipper src/image_flipper_node.cpp)
ament_target_dependencies(image_flipper rclcpp sensor_msgs image_transport cv_bridge OpenCV)

install(TARGETS image_flipper DESTINATION lib/${PROJECT_NAME})

ament_package()
