#ifndef MY_ROBOT_NODES__MY_SERVICE_CLIENT_HPP_
#define MY_ROBOT_NODES__MY_SERVICE_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/srv/validate_fiscal_code.hpp"

class MyClientNode : public rclcpp::Node
{
public:
  MyClientNode();

  bool validateFiscalCode(const std::string & code);

private:
  rclcpp::Client<my_robot_interfaces::srv::ValidateFiscalCode>::SharedPtr pClient_;
  const std::string serviceName_ = "validate_fiscal_code";
};

#endif