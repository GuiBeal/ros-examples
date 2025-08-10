#include "my_robot_nodes/my_subscriber.hpp"

using namespace std::placeholders;

MySubscriberNode::MySubscriberNode() : Node("my_subscriber")
{
  declare_parameter("topic_name", "clock");

  topicName_ = get_parameter("topic_name").as_string();

  pCallbackParams_ =
    this->add_post_set_parameters_callback(std::bind(&MySubscriberNode::cbParameters, this, _1));

  pSubscription_ = create_subscription<my_robot_interfaces::msg::Time>(
    topicName_, rclcpp::SystemDefaultsQoS(),
    std::bind(&MySubscriberNode::cbSubscription, this, _1));

  RCLCPP_INFO(get_logger(), "My subscriber started.");
}

void MySubscriberNode::cbSubscription(const my_robot_interfaces::msg::Time::SharedPtr pMsg)
{
  // not sure how safe the following conversion is
  auto time = static_cast<std::time_t>(pMsg->data.sec);
  auto pLocalTime = std::localtime(&time);
  char buffer[80];
  assert(std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", pLocalTime) != 0);
  std::string formatted_time = std::string(buffer) + "." + std::to_string(pMsg->data.nanosec);

  RCLCPP_INFO_STREAM(get_logger(), "Current date and time is " << formatted_time << ".");
}

void MySubscriberNode::cbParameters(const std::vector<rclcpp::Parameter> & params)
{
  bool resetSubscription = false;
  for (const auto & param : params) {
    if (param.get_name() == "topic_name") {
      topicName_ = param.as_string();
      resetSubscription = true;
    }
  }

  if (resetSubscription) {
    pSubscription_.reset();
    pSubscription_ = create_subscription<my_robot_interfaces::msg::Time>(
      topicName_, rclcpp::SystemDefaultsQoS(),
      std::bind(&MySubscriberNode::cbSubscription, this, _1));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
