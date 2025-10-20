cmake_minimum_required(VERSION 3.8)
project(turtle_time_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(turtle_time_follower src/turtle_time_follower.cpp)
ament_target_dependencies(turtle_time_follower
  rclcpp geometry_msgs turtlesim tf2_ros tf2_geometry_msgs
)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster
  rclcpp geometry_msgs turtlesim tf2_ros
)

add_executable(spawn_turtle src/spawn_turtle.cpp)
ament_target_dependencies(spawn_turtle
  rclcpp turtlesim
)

install(TARGETS
  turtle_time_follower
  turtle_tf2_broadcaster
  spawn_turtle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
