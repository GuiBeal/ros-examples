#include "my_robot_nodes/validate_fiscal_code_server.hpp"

#include <array>
#include <regex>

using namespace std::placeholders;

ValidateFiscalCodeServer::ValidateFiscalCodeServer() : Node("validate_fiscal_code_server")
{
  pServer_ = create_service<my_robot_interfaces::srv::ValidateFiscalCode>(
    serviceName_, std::bind(&ValidateFiscalCodeServer::cbService, this, _1, _2), rclcpp::ServicesQoS());

  RCLCPP_INFO(get_logger(), "My service server started.");
}

void ValidateFiscalCodeServer::cbService(
  const my_robot_interfaces::srv::ValidateFiscalCode::Request::SharedPtr pRequest,
  const my_robot_interfaces::srv::ValidateFiscalCode::Response::SharedPtr pResponse)
{  // validates brazilian CPF
  // the first 9 digits are the code, the last 2 digits are the verifier, totalling 11 digits
  std::string strCode, strVerifier;
  if (std::regex_match(pRequest->code, std::regex("\\d{11}"))) {  // check for digits only format
    strCode = pRequest->code.substr(0, 9);
    strVerifier = pRequest->code.substr(9, 2);
  } else {  // check for ###.###.###-## format
    std::smatch matches;
    if (!std::regex_search(
          pRequest->code, matches, std::regex("(\\d{3})\\.(\\d{3})\\.(\\d{3})-(\\d{2})"))) {
      pResponse->valid = false;
      return;
    }

    assert(matches.size() == 1 + 4);  // first match is the whole string
    strCode.append(matches[1]);
    strCode.append(matches[2]);
    strCode.append(matches[3]);
    strVerifier.append(matches[4]);
  }

  // convert string to integers
  std::array<u_int, 9> code;
  for (size_t i = 0; i < 9; ++i) {
    code[i] = strCode[i] - '0';  // ASCII digits are ordered starting at '0'
  }
  const u_int verifier = strtod(strVerifier.c_str(), nullptr);

  u_int tmp = 0;
  for (size_t i = 0; i < 9; ++i) {
    tmp += code[i] * (10 - i);
  }
  tmp %= 11;
  const u_int firstVerifier = tmp < 2 ? 0 : 11 - tmp;

  tmp = firstVerifier * 2;
  for (size_t i = 0; i < 9; ++i) {
    tmp += code[i] * (11 - i);
  }
  tmp %= 11;
  const u_int secondVerifier = tmp < 2 ? 0 : 11 - tmp;

  pResponse->valid = 10 * firstVerifier + secondVerifier == verifier;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ValidateFiscalCodeServer>());
  rclcpp::shutdown();
  return 0;
}
