#include "my_robot_nodes/my_topic_publisher.hpp"

#include <chrono>

using namespace std::placeholders;

MyPublisherNode::MyPublisherNode() : Node("my_publisher")
{
  declare_parameter("period", 0.1);

  period_ = get_parameter("period").as_double();

  pCallbackParams_ =
    this->add_post_set_parameters_callback(std::bind(&MyPublisherNode::cbParameters, this, _1));

  pPublisher_ =
    create_publisher<my_robot_interfaces::msg::Time>(topicName_, rclcpp::SystemDefaultsQoS());

  pTimerPublisher_ = create_wall_timer(
    std::chrono::duration<double>(period_), std::bind(&MyPublisherNode::cbPublish, this));

  RCLCPP_INFO(get_logger(), "My publisher started.");
}

void MyPublisherNode::cbPublish()
{
  my_robot_interfaces::msg::Time msg;
  msg.data = get_clock()->now();
  pPublisher_->publish(msg);
}

void MyPublisherNode::cbParameters(const std::vector<rclcpp::Parameter> & params)
{
  bool resetTimer = false;
  for (const auto & param : params) {
    if (param.get_name() == "period") {
      period_ = param.as_double();
      resetTimer = true;
    }
  }

  if (resetTimer) {
    pTimerPublisher_->cancel();
    pTimerPublisher_.reset();
    pTimerPublisher_ = create_wall_timer(
      std::chrono::duration<double>(period_), std::bind(&MyPublisherNode::cbPublish, this));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
