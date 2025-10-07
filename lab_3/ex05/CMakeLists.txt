cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CleaningTask.action"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for msg1 only
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Server executable
add_executable(cleaning_action_server src/cleaning_action_server.cpp)
ament_target_dependencies(cleaning_action_server
  rclcpp rclcpp_action turtlesim geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(cleaning_action_server "${cpp_typesupport_target}")

# Client executable
add_executable(cleaning_action_client src/cleaning_action_client.cpp)
ament_target_dependencies(cleaning_action_client
  rclcpp rclcpp_action turtlesim geometry_msgs
)
target_link_libraries(cleaning_action_client "${cpp_typesupport_target}")

install(TARGETS
  cleaning_action_server
  cleaning_action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
