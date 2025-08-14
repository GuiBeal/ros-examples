#ifndef MY_ROBOT_NODES__TIME_PUBLISHER_HPP_
#define MY_ROBOT_NODES__TIME_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/msg/time.hpp"

class TimePublisher : public rclcpp::Node
{
public:
  TimePublisher();

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
