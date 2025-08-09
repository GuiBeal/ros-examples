#include <my_robot_nodes/my_subscriber.hpp>

using namespace std::placeholders;

MySubscriberNode::MySubscriberNode()
    : Node("my_subscriber")
{
  declare_parameter("topic_name", "number");

  topicName_ = get_parameter("topic_name").as_string();

  pCallbackParams_ = this->add_post_set_parameters_callback(
      std::bind(&MySubscriberNode::cbParameters, this, _1));

  pSubscription_ = create_subscription<std_msgs::msg::Int64>(
      topicName_, rclcpp::SystemDefaultsQoS(),
      std::bind(&MySubscriberNode::cbSubscription, this, _1));

  RCLCPP_INFO(get_logger(), "My subscriber started.");
}

void MySubscriberNode::cbSubscription(const std_msgs::msg::Int64::SharedPtr pMsg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received: " << pMsg->data);
}

void MySubscriberNode::cbParameters(const std::vector<rclcpp::Parameter> &params)
{
  bool resetSubscription = false;
  for (const auto &param : params)
  {
    if (param.get_name() == "topic_name")
    {
      topicName_ = param.as_string();
      resetSubscription = true;
    }
  }

  if (resetSubscription)
  {
    pSubscription_.reset();
    pSubscription_ = create_subscription<std_msgs::msg::Int64>(
        topicName_, rclcpp::SystemDefaultsQoS(),
        std::bind(&MySubscriberNode::cbSubscription, this, _1));
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
