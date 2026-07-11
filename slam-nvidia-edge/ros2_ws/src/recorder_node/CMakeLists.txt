cmake_minimum_required(VERSION 3.22)
project(recorder_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_interfaces REQUIRED)

add_executable(${PROJECT_NAME} src/recorder_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs slam_interfaces)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
