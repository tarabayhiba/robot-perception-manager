cmake_minimum_required(VERSION 3.8)
project(perception_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(perception_manager src/perception_manager.cpp)
ament_target_dependencies(perception_manager rclcpp rclcpp_action my_interfaces)

add_executable(camera_tf_broadcaster  src/camera_tf_broadcaster.cpp)
ament_target_dependencies(camera_tf_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

install(TARGETS perception_manager camera_tf_broadcaster 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
