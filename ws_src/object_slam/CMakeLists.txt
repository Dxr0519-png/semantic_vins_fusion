cmake_minimum_required(VERSION 3.8)
project(object_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)   # object_slam_node.cpp 用到 tf2/utils.h
find_package(semantic_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)   # 源码编译安装，见 docs/01 §4

include_directories(include ${EIGEN3_INCLUDE_DIRS})

set(SOURCES
  src/quadric.cpp
  src/data_association.cpp
  src/object_map.cpp
  src/quadric_optimizer.cpp
  src/object_slam_node.cpp
)

add_executable(object_slam_node ${SOURCES})

ament_target_dependencies(object_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 semantic_interfaces)

target_link_libraries(object_slam_node
  g2o_core g2o_stuff g2o_types_slam3d)

install(TARGETS object_slam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
