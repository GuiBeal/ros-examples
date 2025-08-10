#ifndef MY_ROBOT_NODES__MY_SERVICE_SERVER_HPP_
#define MY_ROBOT_NODES__MY_SERVICE_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "my_robot_interfaces/srv/validate_fiscal_code.hpp"

class MyServerNode : public rclcpp::Node
{
public:
  MyServerNode();

private:
  void cbService(
    const my_robot_interfaces::srv::ValidateFiscalCode::Request::SharedPtr pRequest,
    const my_robot_interfaces::srv::ValidateFiscalCode::Response::SharedPtr pResponse);
  void cbParameters(const std::vector<rclcpp::Parameter> & params);

  rclcpp::Service<my_robot_interfaces::srv::ValidateFiscalCode>::SharedPtr pServer_;
  PostSetParametersCallbackHandle::SharedPtr pCallbackParams_;
  std::string serviceName_ = "validate_fiscal_code";
};

#endif
