#ifndef MY_ROBOT_NODES__TIME_SUBSCRIBER_HPP_
#define MY_ROBOT_NODES__TIME_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/msg/time.hpp"

class TimeSubscriber : public rclcpp::Node
{
public:
  TimeSubscriber();

private:
  void cbSubscription(const my_robot_interfaces::msg::Time::SharedPtr pMsg);

  rclcpp::Subscription<my_robot_interfaces::msg::Time>::SharedPtr pSubscription_;
  const std::string topicName_ = "clock";
};

#endif
