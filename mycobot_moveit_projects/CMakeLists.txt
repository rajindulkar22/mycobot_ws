cmake_minimum_required(VERSION 3.8)
project(mycobot_moveit_projects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(named_targets src/named_targets.cpp)

ament_target_dependencies(
  named_targets
  rclcpp
  moveit_ros_planning_interface
)

install(
  TARGETS named_targets
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()