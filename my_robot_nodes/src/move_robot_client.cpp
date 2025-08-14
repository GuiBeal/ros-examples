#include "my_robot_nodes/move_robot_client.hpp"

#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

MoveRobotClient::MoveRobotClient() : Node("move_robot_client")
{
  pCallbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  pClient_ = rclcpp_action::create_client<my_robot_interfaces::action::MoveRobot>(
    this, "move_robot", pCallbackGroup_);

  RCLCPP_INFO(get_logger(), "My action client started.");
}

void MoveRobotClient::sendGoal(const double target, const double velocity)
{
  while (!pClient_->wait_for_action_server(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for server...");
  }

  my_robot_interfaces::action::MoveRobot::Goal goal;
  goal.position = target;
  goal.velocity = velocity;

  rclcpp_action::Client<my_robot_interfaces::action::MoveRobot>::SendGoalOptions options;
  options.goal_response_callback = std::bind(&MoveRobotClient::cbGoalResponse, this, _1);
  options.feedback_callback = std::bind(&MoveRobotClient::cbGoalFeedback, this, _1, _2);
  options.result_callback = std::bind(&MoveRobotClient::cbGoalResult, this, _1);

  RCLCPP_INFO(get_logger(), "Sending goal.");
  pClient_->async_send_goal(goal, options);
  rclcpp::sleep_for(1s);  // artificial delay
}

void MoveRobotClient::cancelGoal()
{
  std::lock_guard<std::mutex> lock(mutexGoalHandle_);
  if (!pGoalHandle_) {
    RCLCPP_WARN(get_logger(), "No goal available for cancelling.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Cancelling goal.");
  pClient_->async_cancel_goal(pGoalHandle_);
  pGoalHandle_.reset();
  rclcpp::sleep_for(1s);  // artificial delay
}

void MoveRobotClient::cbGoalResponse(const GoalHandle::SharedPtr & pGoalHandle)
{
  if (!pGoalHandle) {
    RCLCPP_INFO(get_logger(), "Goal rejected.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    pGoalHandle_ = pGoalHandle;
  }
  RCLCPP_INFO(get_logger(), "Goal accepted.");
}

void MoveRobotClient::cbGoalFeedback(
  const GoalHandle::SharedPtr & /*pGoalHandle*/,
  const my_robot_interfaces::action::MoveRobot::Feedback::ConstSharedPtr pFeedback)
{
  RCLCPP_INFO(get_logger(), "Current position is %f.", pFeedback->position);
}

void MoveRobotClient::cbGoalResult(const GoalHandle::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    pGoalHandle_.reset();
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      RCLCPP_INFO(
        get_logger(), "Goal succeeded with message '%s'.", result.result->message.c_str());
    } break;
    case rclcpp_action::ResultCode::ABORTED: {
      RCLCPP_WARN(get_logger(), "Goal aborted with message '%s'.", result.result->message.c_str());
    } break;
    case rclcpp_action::ResultCode::CANCELED: {
      RCLCPP_INFO(get_logger(), "Goal canceled with message '%s'.", result.result->message.c_str());
    } break;
    default: {
      RCLCPP_ERROR(get_logger(), "Unknown goal result code %d.", static_cast<int>(result.code));
    } break;
  }

  RCLCPP_INFO(get_logger(), "Final position is %f.", result.result->position);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(1s);

  auto pNode = std::make_shared<MoveRobotClient>();

  std::thread tSpin([pNode]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pNode->get_node_base_interface());
    executor.spin();
  });

  std::thread tGoal([pNode]() {
    pNode->sendGoal(70, 5);
    std::this_thread::sleep_for(2s);
    pNode->cancelGoal();
    std::this_thread::sleep_for(1s);
    pNode->sendGoal(70, 2);
    std::this_thread::sleep_for(5s);
    pNode->sendGoal(20, 10);
  });

  tSpin.join();
  tGoal.join();

  rclcpp::shutdown();
  return 0;
}