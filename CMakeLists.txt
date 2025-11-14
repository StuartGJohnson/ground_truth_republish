cmake_minimum_required(VERSION 3.8)
project(ground_truth_republish)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -march=native -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ground_truth_republish_node src/ground_truth_republish_node.cpp)
ament_target_dependencies(ground_truth_republish_node
  rclcpp rclcpp_components geometry_msgs nav_msgs tf2 tf2_ros)
target_include_directories(ground_truth_republish_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(ground_truth_republish_node PRIVATE ${EIGEN3_INCLUDE_DIR})
target_compile_definitions(ground_truth_republish_node PRIVATE "EIGEN_NO_DEBUG")

install(TARGETS ground_truth_republish_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME})

ament_package()
