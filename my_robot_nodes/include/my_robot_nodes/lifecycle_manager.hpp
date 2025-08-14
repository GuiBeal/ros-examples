#ifndef MY_ROBOT_NODES__LIFECYCLE_MANAGER_HPP
#define MY_ROBOT_NODES__LIFECYCLE_MANAGER_HPP

#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/rclcpp.hpp>

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();

  bool changeState(const lifecycle_msgs::msg::Transition & transition);
  void initialize();

private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr pClient_;

  std::string managedNodeName_;
};

#endif
