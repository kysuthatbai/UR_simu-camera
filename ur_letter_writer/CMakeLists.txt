cmake_minimum_required(VERSION 3.8)
project(ur_letter_writer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(letter_writer_node src/letter_writer_node.cpp)
ament_target_dependencies(letter_writer_node
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  visualization_msgs
)

install(TARGETS
  letter_writer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
