cmake_minimum_required(VERSION 3.8)
project(mycobot_280jn_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    urdf
    config
    meshes
  DESTINATION share/${PROJECT_NAME}
)

# Copy the world file (do not symlink the worlds/ dir — Docker/host share one
# install tree and absolute symlinks break across /root vs /home paths).
install(
  FILES worlds/mycobot_table.sdf
  DESTINATION share/${PROJECT_NAME}/worlds
)

# colcon --symlink-install turns FILES into symlinks too; force a real copy so
# Gazebo always loads the installed file from the shared host/Docker tree.
install(
  CODE "file(COPY \"${CMAKE_CURRENT_SOURCE_DIR}/worlds/mycobot_table.sdf\"
        DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/worlds\")"
)

ament_package()
