#ifndef MY_ROBOT_NODES__MY_PUBLISHER_HPP_
#define MY_ROBOT_NODES__MY_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/msg/time.hpp"

class MyPublisherNode : public rclcpp::Node
{
public:
  MyPublisherNode();

private:
  void cbPublish();
  void cbParameters(const std::vector<rclcpp::Parameter> & params);

  rclcpp::TimerBase::SharedPtr pTimerPublisher_;
  rclcpp::Publisher<my_robot_interfaces::msg::Time>::SharedPtr pPublisher_;

  const std::string topicName_ = "clock";
  double period_ = 0.1;

  PostSetParametersCallbackHandle::SharedPtr pCallbackParams_;
};

#endif
