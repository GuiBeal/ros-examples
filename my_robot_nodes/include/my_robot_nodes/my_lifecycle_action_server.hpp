#ifndef MY_ROBOT_NODES__MY_LIFECYCLE_NODE_HPP
#define MY_ROBOT_NODES__MY_LIFECYCLE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "my_robot_interfaces/action/move_robot.hpp"

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using GoalHandle = rclcpp_action::ServerGoalHandle<my_robot_interfaces::action::MoveRobot>;

class MyLifecycleServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleServer();

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

private:
  void configure();
  void cleanup();
  void activate();
  void deactivate();

  rclcpp_action::GoalResponse cbHandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    my_robot_interfaces::action::MoveRobot::Goal::ConstSharedPtr pGoal);
  rclcpp_action::CancelResponse cbHandleCancel(const std::shared_ptr<GoalHandle> pGoalHandle);
  void cbHandleAccepted(const std::shared_ptr<GoalHandle> pGoalHandle);

  void executeGoal(const std::shared_ptr<GoalHandle> pGoalHandle);

  rclcpp::CallbackGroup::SharedPtr pCallbackGroup_;
  rclcpp_action::Server<my_robot_interfaces::action::MoveRobot>::SharedPtr pServer_;

  std::mutex mutexGoalHandle_;
  std::shared_ptr<GoalHandle> pGoalHandle_;
  std::set<rclcpp_action::GoalUUID> preemptingGoalIDs_;

  double updateRate_ = 1.0;

  std::string robotName_ = "robot";

  double minPosition_ = 0;
  double maxPosition_ = 100;

  double position_ = 50;

  bool isActivated_ = false;
};

#endif