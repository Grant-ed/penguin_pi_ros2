cmake_minimum_required(VERSION 3.8)
project(penguin_pi_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetInt16.srv"
  "srv/GetEncoders.srv"
  DEPENDENCIES # Add packages that above messages depend on
)

install(
  DIRECTORY config rviz urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()