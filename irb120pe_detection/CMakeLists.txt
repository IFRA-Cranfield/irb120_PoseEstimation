cmake_minimum_required(VERSION 3.8)
project(irb120pe_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# =========================================================== #
# REQUIRED TO EXECUTE .cpp scripts:

install(
  DIRECTORY 
    src
  DESTINATION 
    share/${PROJECT_NAME}
)

# REQUIRED to -> Communicate with ROS2:
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# REQUIRED -> irb120pe_data:
find_package(irb120pe_data REQUIRED)

# REQUIRED -> RobMove ACTION:
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Add executable:
add_executable(
  robmove
  src/RobMove.cpp
)

# Install executable:
install(TARGETS
  robmove
  DESTINATION lib/${PROJECT_NAME}
)

# Add ament dependencies:
ament_target_dependencies(
  robmove
  irb120pe_data
  rclcpp
  std_msgs
  rclcpp_action
  moveit_ros_planning_interface
)

# =========================================================== #
# REQUIRED TO EXECUTE .py scripts:
install(
  DIRECTORY 
    python
  DESTINATION 
    share/${PROJECT_NAME}
)
install(PROGRAMS
  python/yolo.py
  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()