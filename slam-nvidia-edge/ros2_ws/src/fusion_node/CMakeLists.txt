cmake_minimum_required(VERSION 3.22)
project(fusion_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slam_interfaces REQUIRED)
find_package(slam_core REQUIRED)

add_executable(${PROJECT_NAME} src/fusion_node.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs geometry_msgs nav_msgs slam_interfaces)
target_link_libraries(${PROJECT_NAME} slam_core::slam_core)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
