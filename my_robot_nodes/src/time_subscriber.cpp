#include "my_robot_nodes/time_subscriber.hpp"

using namespace std::placeholders;

TimeSubscriber::TimeSubscriber() : Node("time_subscriber")
{
  pSubscription_ = create_subscription<my_robot_interfaces::msg::Time>(
    topicName_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TimeSubscriber::cbSubscription, this, _1));

  RCLCPP_INFO(get_logger(), "My subscriber started.");
}

void TimeSubscriber::cbSubscription(const my_robot_interfaces::msg::Time::SharedPtr pMsg)
{
  // not sure how safe the following conversion is
  auto time = static_cast<std::time_t>(pMsg->data.sec);
  auto pLocalTime = std::localtime(&time);
  char buffer[80];
  assert(std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", pLocalTime) != 0);
  std::string formatted_time = std::string(buffer) + "." + std::to_string(pMsg->data.nanosec);

  RCLCPP_INFO_STREAM(get_logger(), "Current date and time is " << formatted_time << ".");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
