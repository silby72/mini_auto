cmake_minimum_required(VERSION 3.8)
project(mini_auto)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ROS非依存の計画ライブラリ
add_library(mini_auto_planner src/grid_planner.cpp)
target_include_directories(mini_auto_planner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

add_executable(omni_drive src/omni_drive_node.cpp)
target_include_directories(omni_drive PRIVATE include)
ament_target_dependencies(omni_drive rclcpp std_msgs geometry_msgs)

add_executable(tracking_odom src/tracking_odom_node.cpp)
target_include_directories(tracking_odom PRIVATE include)
ament_target_dependencies(tracking_odom
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(simple_nav src/simple_nav_node.cpp)
target_link_libraries(simple_nav mini_auto_planner)
ament_target_dependencies(simple_nav
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(depth_obstacle src/depth_obstacle_node.cpp)
ament_target_dependencies(depth_obstacle rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(sim_robot src/sim_robot_node.cpp)
target_include_directories(sim_robot PRIVATE include)
ament_target_dependencies(sim_robot
  rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS omni_drive tracking_odom simple_nav depth_obstacle sim_robot
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_include_directories(test_kinematics PRIVATE include)
  ament_add_gtest(test_grid_planner test/test_grid_planner.cpp)
  target_link_libraries(test_grid_planner mini_auto_planner)
endif()

ament_package()
