cmake_minimum_required(VERSION 3.8)
project(ur_llm_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteSkill.srv"
  "msg/DetectedObject.msg"
  "msg/ZoneState.msg"
  "msg/SceneObservation.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(skill_server src/skill_server.cpp)
target_compile_features(skill_server PUBLIC cxx_std_17)
ament_target_dependencies(skill_server
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
)
target_link_libraries(skill_server "${cpp_typesupport_target}")

install(TARGETS skill_server
  DESTINATION lib/${PROJECT_NAME}
)

# Module Python (LLM client, prompt, plan validator, perception) + node Python
ament_python_install_package(llm_planner)
install(PROGRAMS
  scripts/llm_planner_node.py
  scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds urdf srdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_plan_validator test/test_plan_validator.py
    PYTHON_EXECUTABLE python3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_student test/test_student.py
    PYTHON_EXECUTABLE python3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_plan_resolver test/test_plan_resolver.py
    PYTHON_EXECUTABLE python3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_perception test/test_perception.py
    PYTHON_EXECUTABLE python3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
