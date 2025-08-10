#include "my_robot_nodes/my_service_client.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

MyClientNode::MyClientNode() : Node("my_service_client")
{
  declare_parameter("service_name", "validate_fiscal_code");

  serviceName_ = get_parameter("service_name").as_string();

  pCallbackParams_ =
    this->add_post_set_parameters_callback(std::bind(&MyClientNode::cbParameters, this, _1));

  pClient_ = create_client<my_robot_interfaces::srv::ValidateFiscalCode>(
    serviceName_, rclcpp::ServicesQoS());

  RCLCPP_INFO(get_logger(), "My service client started.");
}

bool MyClientNode::validateFiscalCode(const std::string & code)
{
  while (!pClient_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for server...");
  }

  auto pRequest = std::make_shared<my_robot_interfaces::srv::ValidateFiscalCode::Request>();
  pRequest->code = code;

  auto future = pClient_->async_send_request(pRequest);  // could also received a callback

  const auto futureStatus =
    rclcpp::spin_until_future_complete(get_node_base_interface(), future, 10s);
  switch (futureStatus)  // not really a good error handling below
  {
    case rclcpp::FutureReturnCode::SUCCESS: {
      const auto pResponse = future.future.get();
      return pResponse->valid;
    }
    case rclcpp::FutureReturnCode::TIMEOUT: {
      RCLCPP_ERROR(get_logger(), "Request timed out.");
      pClient_->remove_pending_request(future.request_id);
      throw "Request timed out.";
    }
    case rclcpp::FutureReturnCode::INTERRUPTED: {
      RCLCPP_ERROR(get_logger(), "Request interrupted.");
      throw "Request interrupted.";
    }
    default: {
      RCLCPP_ERROR(get_logger(), "Request returned unknown status.");
      throw "Request returned unknown status.";
    }
  }
}

void MyClientNode::cbParameters(const std::vector<rclcpp::Parameter> & params)
{
  bool resetClient = false;
  for (const auto & param : params) {
    if (param.get_name() == "service_name") {
      serviceName_ = param.as_string();
      resetClient = true;
    }
  }

  if (resetClient) {
    pClient_.reset();
    pClient_ = create_client<my_robot_interfaces::srv::ValidateFiscalCode>(
      serviceName_, rclcpp::ServicesQoS());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto pNode = std::make_shared<MyClientNode>();
  for (const auto & code : {"610.420.070-20", "123.456.789-10"}) {
    const bool valid = pNode->validateFiscalCode(code);
    RCLCPP_INFO(pNode->get_logger(), "Fiscal code %s is %s.", code, valid ? "valid" : "invalid");
  }
  rclcpp::spin(pNode);
  rclcpp::shutdown();
  return 0;
}