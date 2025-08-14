cmake_minimum_required(VERSION 3.8)
project(my_robot_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

include_directories(include)

add_executable(time_publisher src/time_publisher.cpp)
ament_target_dependencies(time_publisher rclcpp my_robot_interfaces)

add_executable(time_subscriber src/time_subscriber.cpp)
ament_target_dependencies(time_subscriber rclcpp my_robot_interfaces)

add_executable(validate_fiscal_code_server src/validate_fiscal_code_server.cpp)
ament_target_dependencies(validate_fiscal_code_server rclcpp my_robot_interfaces)

add_executable(validate_fiscal_code_client src/validate_fiscal_code_client.cpp)
ament_target_dependencies(validate_fiscal_code_client rclcpp my_robot_interfaces)

add_executable(lifecycle_manager src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager rclcpp lifecycle_msgs)

add_executable(move_robot_server src/move_robot_server.cpp)
ament_target_dependencies(move_robot_server rclcpp rclcpp_action rclcpp_lifecycle my_robot_interfaces)

add_executable(move_robot_client src/move_robot_client.cpp)
ament_target_dependencies(move_robot_client rclcpp rclcpp_action rclcpp_lifecycle my_robot_interfaces)

install(TARGETS
  time_publisher
  time_subscriber
  validate_fiscal_code_server
  validate_fiscal_code_client
  lifecycle_manager
  move_robot_server
  move_robot_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
