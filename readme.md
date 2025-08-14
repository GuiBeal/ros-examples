# ROS2 Examples

## Time Topic Publisher-Subscriber

* `my_robot_nodes`
  * [`time_publisher.hpp`](my_robot_nodes/include/my_robot_nodes/time_publisher.hpp) / [`time_publisher.cpp`](my_robot_nodes/src/time_publisher.cpp)
    * Topic Publisher
  * [`time_subscriber.hpp`](my_robot_nodes/include/my_robot_nodes/time_subscriber.hpp) / [`time_publisher.cpp`](my_robot_nodes/src/time_subscriber.cpp)
    * Topic Subscriber
* `my_robot_bringup`
  * [`time_topic.launch.xml`](my_robot_bringup/launch/time_topic.launch.xml)
    * Launch Arguments
    * Remapping
    * Node Parameters

## Validate Fiscal Code Service Server-Client

* `my_robot_nodes`
  * [`validate_fiscal_code_server.hpp`](my_robot_nodes/include/my_robot_nodes/validate_fiscal_code_server.hpp) / [`validate_fiscal_code_server.cpp`](my_robot_nodes/src/validate_fiscal_code_server.cpp)
    * Service Server
  * [`validate_fiscal_code_client.hpp`](my_robot_nodes/include/my_robot_nodes/validate_fiscal_code_client.hpp) / [`validate_fiscal_code_client.cpp`](my_robot_nodes/src/validate_fiscal_code_client.cpp)
    * Service Client
* `my_robot_bringup`
  * [`validate_fiscal_code_service.launch.xml`](my_robot_bringup/launch/validate_fiscal_code_service.launch.xml)
    * Launch Arguments
    * Remapping

## Move Robot Action Server-Client

* `my_robot_nodes`
  * [`move_robot_server.hpp`](my_robot_nodes/include/my_robot_nodes/move_robot_server.hpp) / [`move_robot_server.cpp`](my_robot_nodes/src/move_robot_server.cpp)
    * Action Server
    * Lifecycle Node
  * [`move_robot_client.hpp`](my_robot_nodes/include/my_robot_nodes/move_robot_client.hpp) / [`move_robot_client.cpp`](my_robot_nodes/src/move_robot_client.cpp)
    * Action Client
  * [`lifecycle_manager.hpp`](my_robot_nodes/include/my_robot_nodes/lifecycle_manager.hpp) / [`lifecycle_manager.cpp`](my_robot_nodes/src/lifecycle_manager.cpp)
    * Service Client
* `my_robot_bringup`
  * [`move_robot_action.launch.xml`](my_robot_bringup/launch/move_robot_action.launch.xml)
    * Laung Arguments
    * Node Parameters from File

## Robot Description

* `my_robot_description`
  * [`my_robot.urdf.xacro`](my_robot_description/urdf/my_robot.urdf.xacro)
    * Joints and Links
    * ROS2 Control
    * Xacro
      * Property
      * Macro
      * Include

## Robot Launcher

* `my_robot_bringup`
  * [my_robot.launch.xml](my_robot_bringup/launch/my_robot.launch.xml) / [my_robot.launch.py](my_robot_bringup/launch/my_robot.launch.py)
    * Launch Arguments
    * Node Parameters
    * Node Parameters from File
    * ROS2 Control
    * RViz2
