#include "my_robot_nodes/lifecycle_manager.hpp"

using namespace std::chrono_literals;

LifecycleManager::LifecycleManager() : Node("lifecycle_manager")
{
  declare_parameter("managed_node_name", rclcpp::ParameterType::PARAMETER_STRING);

  managedNodeName_ = get_parameter("managed_node_name").as_string();
  const std::string serviceName = "/" + managedNodeName_ + "/change_state";

  pClient_ = create_client<lifecycle_msgs::srv::ChangeState>(serviceName, rclcpp::ServicesQoS());

  RCLCPP_INFO(get_logger(), "My lifecycle manager started.");
}

bool LifecycleManager::changeState(const lifecycle_msgs::msg::Transition & transition)
{
  while (!pClient_->wait_for_service(10s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for managed node...");
  }

  auto pRequest = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  pRequest->transition = transition;

  const auto future = pClient_->async_send_request(pRequest);
  const auto futureStatus =
    rclcpp::spin_until_future_complete(get_node_base_interface(), future, 10s);
  return futureStatus == rclcpp::FutureReturnCode::SUCCESS;
}

void LifecycleManager::initialize()
{
  lifecycle_msgs::msg::Transition transition;

  transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  transition.label = "configure";
  if (!changeState(transition)) {
    throw "Failed to configure.";
  }

  transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  transition.label = "activate";
  if (!changeState(transition)) {
    throw "Failed to activate.";
  }

  RCLCPP_INFO(get_logger(), "Initialized %s.", managedNodeName_.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pNode = std::make_shared<LifecycleManager>();
  pNode->initialize();
  rclcpp::shutdown();
  return 0;
}