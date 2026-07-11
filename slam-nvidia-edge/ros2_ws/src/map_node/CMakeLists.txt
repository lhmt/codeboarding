cmake_minimum_required(VERSION 3.22)
project(map_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/map_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
