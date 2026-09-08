cmake_minimum_required(VERSION 3.8)

project(mycobot_moveit_projects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_core REQUIRED)

#The find_package command is used to find the package and its dependencies.

add_executable(cartesian_path src/cartesian_path.cpp)

ament_target_dependencies(
  cartesian_path
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
) 
add_executable(named_targets
  src/named_targets.cpp
)

ament_target_dependencies(
  named_targets
  rclcpp
  moveit_ros_planning_interface
)

add_executable(pose_target
  src/pose_target.cpp
)

ament_target_dependencies(
  pose_target
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(
  planning_scene_objects
  src/planning_scene_objects.cpp
)

ament_target_dependencies(
  planning_scene_objects
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)

add_executable(
  test_obstacle
  src/test_obstacle.cpp
)

ament_target_dependencies(
  test_obstacle
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)

add_executable(cube_approach src/cube_approach.cpp)

ament_target_dependencies(
  cube_approach
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  shape_msgs
)

install(
  TARGETS
    named_targets
    pose_target
    planning_scene_objects
    test_obstacle 
    cartesian_path
    cube_approach
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()