# ROS-free unit tests for slam_core. Buildable anywhere with a C++20 compiler
# and Eigen: cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
cmake_minimum_required(VERSION 3.22)
project(slam_core_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
enable_testing()

set(SLAM_CORE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/slam_core/include)
set(LIO_NODE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/lio_node/include)

foreach(name lie_group imu_preintegration factor_residuals health_monitor)
  add_executable(test_${name} test_${name}.cpp)
  target_include_directories(test_${name} PRIVATE ${SLAM_CORE_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(test_${name} PRIVATE Eigen3::Eigen)
  add_test(NAME ${name} COMMAND test_${name})
endforeach()

add_executable(test_scan_deskew test_scan_deskew.cpp)
target_include_directories(test_scan_deskew PRIVATE ${SLAM_CORE_INCLUDE} ${LIO_NODE_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_scan_deskew PRIVATE Eigen3::Eigen)
add_test(NAME scan_deskew COMMAND test_scan_deskew)

add_executable(test_point_to_plane_icp test_point_to_plane_icp.cpp)
target_include_directories(test_point_to_plane_icp PRIVATE ${SLAM_CORE_INCLUDE} ${LIO_NODE_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_point_to_plane_icp PRIVATE Eigen3::Eigen)
add_test(NAME point_to_plane_icp COMMAND test_point_to_plane_icp)
