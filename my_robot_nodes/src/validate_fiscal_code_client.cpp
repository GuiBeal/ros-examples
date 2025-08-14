#include "my_robot_nodes/validate_fiscal_code_client.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

ValidateFiscalCodeClient::ValidateFiscalCodeClient() : Node("validate_fiscal_code_client")
{
  pClient_ = create_client<my_robot_interfaces::srv::ValidateFiscalCode>(
    serviceName_, rclcpp::ServicesQoS());

  RCLCPP_INFO(get_logger(), "My service client started.");
}

bool ValidateFiscalCodeClient::validateFiscalCode(const std::string & code)
{
  while (!pClient_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for server...");
  }

  auto pRequest = std::make_shared<my_robot_interfaces::srv::ValidateFiscalCode::Request>();
  pRequest->code = code;

  auto future = pClient_->async_send_request(pRequest);  // could also receive a callback

  const auto futureStatus =
    rclcpp::spin_until_future_complete(get_node_base_interface(), future, 10s);
  switch (futureStatus)  // not really a good error handling below
  {
    case rclcpp::FutureReturnCode::SUCCESS: {
      const auto pResponse = future.get();
      return pResponse->valid;
    }
    case rclcpp::FutureReturnCode::TIMEOUT: {
      pClient_->remove_pending_request(future.request_id);
      throw "Request timed out.";
    }
    case rclcpp::FutureReturnCode::INTERRUPTED: {
      throw "Request interrupted.";
    }
    default: {
      throw "Request returned unknown status.";
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto pNode = std::make_shared<ValidateFiscalCodeClient>();
  for (const auto & code : {"610.420.070-20", "123.456.789-10"}) {
    const bool valid = pNode->validateFiscalCode(code);
    RCLCPP_INFO(pNode->get_logger(), "Fiscal code %s is %s.", code, valid ? "valid" : "invalid");
  }
  rclcpp::spin(pNode);
  rclcpp::shutdown();
  return 0;
}