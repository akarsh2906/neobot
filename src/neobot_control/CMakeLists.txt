cmake_minimum_required(VERSION 3.8)
project(neobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(wheel_speed_pub src/wheel_speed_pub.cpp)
ament_target_dependencies(wheel_speed_pub rclcpp geometry_msgs std_msgs)


install(TARGETS
  wheel_speed_pub
  DESTINATION lib/${PROJECT_NAME})


ament_package()
