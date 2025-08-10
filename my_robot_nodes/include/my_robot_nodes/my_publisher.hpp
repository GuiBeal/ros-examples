#ifndef MY_ROBOT_NODES__MY_PUBLISHER_HPP_
#define MY_ROBOT_NODES__MY_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class MyPublisherNode : public rclcpp::Node
{
public:
  MyPublisherNode();

private:
  void cbPublish();
  void cbParameters(const std::vector<rclcpp::Parameter> & params);

  rclcpp::TimerBase::SharedPtr pTimerPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pPublisher_;

  std::string topicName_ = "number";
  double period_ = 1.0;
  int64_t number_ = 0;

  PostSetParametersCallbackHandle::SharedPtr pCallbackParams_;
};

#endif
