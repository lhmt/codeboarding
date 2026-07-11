cmake_minimum_required(VERSION 3.22)
project(tensor_frontend_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Both default OFF so the package builds on any host. On Jetson:
#   colcon build --cmake-args -DENABLE_CUDA=ON -DENABLE_TENSORRT=ON
option(ENABLE_CUDA "Build with CUDA streams" OFF)
option(ENABLE_TENSORRT "Build the TensorRT engine backend" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/tensor_frontend_node.cpp src/inference_engine.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs std_msgs)

if(ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_CUDA)
  target_link_libraries(${PROJECT_NAME} CUDA::cudart)
endif()

if(ENABLE_TENSORRT)
  # JetPack ships TensorRT headers/libs in the default paths; adjust for SDK installs.
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_TENSORRT)
  target_link_libraries(${PROJECT_NAME} nvinfer)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
