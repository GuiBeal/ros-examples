#ifndef MY_ROBOT_NODES__MOVE_ROBOT_CLIENT_HPP
#define MY_ROBOT_NODES__MOVE_ROBOT_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_robot_interfaces/action/move_robot.hpp"

using GoalHandle = rclcpp_action::ClientGoalHandle<my_robot_interfaces::action::MoveRobot>;

class MoveRobotClient : public rclcpp::Node
{
public:
  MoveRobotClient();

  void sendGoal(const double target, const double velocity);
  void cancelGoal();

private:
  void cbGoalResponse(const GoalHandle::SharedPtr & pGoalHandle);
  void cbGoalFeedback(
    const GoalHandle::SharedPtr & pGoalHandle,
    const my_robot_interfaces::action::MoveRobot::Feedback::ConstSharedPtr pFeedback);
  void cbGoalResult(const GoalHandle::WrappedResult & result);

  rclcpp::CallbackGroup::SharedPtr pCallbackGroup_;
  rclcpp_action::Client<my_robot_interfaces::action::MoveRobot>::SharedPtr pClient_;

  std::mutex mutexGoalHandle_;
  GoalHandle::SharedPtr pGoalHandle_;
};

#endif
