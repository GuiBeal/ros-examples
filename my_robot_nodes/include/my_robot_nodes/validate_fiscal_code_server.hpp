#ifndef MY_ROBOT_NODES__VALIDATE_FISCAL_CODE_SERVER_HPP_
#define MY_ROBOT_NODES__VALIDATE_FISCAL_CODE_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/srv/validate_fiscal_code.hpp"

class ValidateFiscalCodeServer : public rclcpp::Node
{
public:
  ValidateFiscalCodeServer();

private:
  void cbService(
    const my_robot_interfaces::srv::ValidateFiscalCode::Request::SharedPtr pRequest,
    const my_robot_interfaces::srv::ValidateFiscalCode::Response::SharedPtr pResponse);

  rclcpp::Service<my_robot_interfaces::srv::ValidateFiscalCode>::SharedPtr pServer_;
  const std::string serviceName_ = "validate_fiscal_code";
};

#endif
