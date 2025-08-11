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

add_executable(my_topic_publisher src/my_topic_publisher.cpp)
ament_target_dependencies(my_topic_publisher rclcpp my_robot_interfaces)

add_executable(my_topic_subscriber src/my_topic_subscriber.cpp)
ament_target_dependencies(my_topic_subscriber rclcpp my_robot_interfaces)

add_executable(my_service_server src/my_service_server.cpp)
ament_target_dependencies(my_service_server rclcpp my_robot_interfaces)

add_executable(my_service_client src/my_service_client.cpp)
ament_target_dependencies(my_service_client rclcpp my_robot_interfaces)

add_executable(my_lifecycle_manager src/my_lifecycle_manager.cpp)
ament_target_dependencies(my_lifecycle_manager rclcpp lifecycle_msgs)

add_executable(my_lifecycle_action_server src/my_lifecycle_action_server.cpp)
ament_target_dependencies(my_lifecycle_action_server rclcpp rclcpp_action rclcpp_lifecycle my_robot_interfaces)

add_executable(my_action_client src/my_action_client.cpp)
ament_target_dependencies(my_action_client rclcpp rclcpp_action rclcpp_lifecycle my_robot_interfaces)

install(TARGETS
  my_topic_publisher
  my_topic_subscriber
  my_service_server
  my_service_client
  my_lifecycle_manager
  my_lifecycle_action_server
  my_action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
