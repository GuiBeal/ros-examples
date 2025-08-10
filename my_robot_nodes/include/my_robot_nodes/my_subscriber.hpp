#ifndef MY_ROBOT_NODES__MY_SUBSCRIBER_HPP_
#define MY_ROBOT_NODES__MY_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/msg/time.hpp"

class MySubscriberNode : public rclcpp::Node
{
public:
  MySubscriberNode();

private:
  void cbSubscription(const my_robot_interfaces::msg::Time::SharedPtr pMsg);
  void cbParameters(const std::vector<rclcpp::Parameter> & params);

  rclcpp::Subscription<my_robot_interfaces::msg::Time>::SharedPtr pSubscription_;

  std::string topicName_ = "clock";

  PostSetParametersCallbackHandle::SharedPtr pCallbackParams_;
};

#endif
