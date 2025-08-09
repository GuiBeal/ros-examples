#ifndef MY_ROBOT_NODES__MY_SUBSCRIBER_HPP_
#define MY_ROBOT_NODES__MY_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class MySubscriberNode : public rclcpp::Node
{
public:
  MySubscriberNode();

private:
  void cbSubscription(const std_msgs::msg::Int64::SharedPtr pMsg);
  void cbParameters(const std::vector<rclcpp::Parameter> &params);

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr pSubscription_;

  std::string topicName_ = "number";

  PostSetParametersCallbackHandle::SharedPtr pCallbackParams_;
};

#endif
