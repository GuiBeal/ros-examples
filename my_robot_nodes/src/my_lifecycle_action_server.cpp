#include "my_robot_nodes/my_lifecycle_action_server.hpp"

using namespace std::placeholders;

MyLifecycleServer::MyLifecycleServer() : LifecycleNode("my_lifecycle_action_server")
{
  RCLCPP_INFO(get_logger(), "My action server started.");
}

LifecycleCallbackReturn MyLifecycleServer::on_configure(const rclcpp_lifecycle::State & state)
{
  configure();
  RCLCPP_INFO(get_logger(), "My action server configured.");
  return rclcpp_lifecycle::LifecycleNode::on_configure(state);
}

LifecycleCallbackReturn MyLifecycleServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  cleanup();
  RCLCPP_INFO(get_logger(), "My action server unconfigured.");
  return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
}

LifecycleCallbackReturn MyLifecycleServer::on_activate(const rclcpp_lifecycle::State & state)
{
  activate();
  RCLCPP_INFO(get_logger(), "My action server activated.");
  return rclcpp_lifecycle::LifecycleNode::on_activate(state);
}

LifecycleCallbackReturn MyLifecycleServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  deactivate();
  RCLCPP_INFO(get_logger(), "My action server deactivated.");
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

LifecycleCallbackReturn MyLifecycleServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  deactivate();
  cleanup();
  RCLCPP_INFO(get_logger(), "My action server shutting down.");
  return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
}

void MyLifecycleServer::configure()
{
  declare_parameter("robot_name", "robot");
  declare_parameter("min_position", 0.0);
  declare_parameter("max_position", 100.0);
  declare_parameter("initial_position", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("update_rate", 1.0);

  robotName_ = this->get_parameter("robot_name").as_string();
  std::transform(robotName_.begin(), robotName_.end(), robotName_.begin(), [](u_char c) {
    return std::tolower(c);
  });
  if (robotName_.empty()) {
    throw rclcpp::exceptions::InvalidParametersException("robot_name must not be empty");
  }

  minPosition_ = get_parameter("min_position").as_double();
  maxPosition_ = get_parameter("max_position").as_double();
  if (minPosition_ >= maxPosition_) {
    throw rclcpp::exceptions::InvalidParametersException(
      "max_position must be greater than min_position");
  }

  double initialPosition;
  if (!get_parameter("initial_position", initialPosition)) {
    initialPosition = 0.5 * (minPosition_ + maxPosition_);
  }
  if (initialPosition < minPosition_ || initialPosition > maxPosition_) {
    throw rclcpp::exceptions::InvalidParametersException(
      "initial_position must be between min_position and max_position");
  }

  updateRate_ = get_parameter("update_rate").as_double();
  if (updateRate_ <= 0) {
    throw rclcpp::exceptions::InvalidParametersException("update_rate must be positive");
  }

  pCallbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  const std::string actionName = "move_" + robotName_;
  pServer_ = rclcpp_action::create_server<my_robot_interfaces::action::MoveRobot>(
    this, actionName, std::bind(&MyLifecycleServer::cbHandleGoal, this, _1, _2),
    std::bind(&MyLifecycleServer::cbHandleCancel, this, _1),
    std::bind(&MyLifecycleServer::cbHandleAccepted, this, _1),
    rcl_action_server_get_default_options(), pCallbackGroup_);

  position_ = initialPosition;
}

void MyLifecycleServer::cleanup()
{
  this->undeclare_parameter("robot_name");
  this->undeclare_parameter("min_position");
  this->undeclare_parameter("max_position");
  this->undeclare_parameter("initial_position");
  pServer_.reset();
  pCallbackGroup_.reset();
}

void MyLifecycleServer::activate() { isActivated_ = true; }

void MyLifecycleServer::deactivate()
{
  isActivated_ = false;
  if (pGoalHandle_ && pGoalHandle_->is_active()) {
    preemptingGoalIDs_.insert(pGoalHandle_->get_goal_id());
  }
}

rclcpp_action::GoalResponse MyLifecycleServer::cbHandleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  my_robot_interfaces::action::MoveRobot::Goal::ConstSharedPtr pGoal)
{
  if (!isActivated_) {
    RCLCPP_INFO(get_logger(), "Goal rejected because server is inactive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (pGoal->position < minPosition_) {
    RCLCPP_INFO(get_logger(), "Goal rejected because target position is below %f.", minPosition_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (pGoal->position > maxPosition_) {
    RCLCPP_INFO(get_logger(), "Goal rejected because target position is above %f.", maxPosition_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (pGoal->velocity <= 0) {
    RCLCPP_INFO(get_logger(), "Goal rejected because velocity is not positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    if (pGoalHandle_ && pGoalHandle_->is_active()) {
      preemptingGoalIDs_.insert(pGoalHandle_->get_goal_id());
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Goal accepted for position %f with velocity %f.", pGoal->position,
    pGoal->velocity);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MyLifecycleServer::cbHandleCancel(
  const std::shared_ptr<GoalHandle> /*pGoalHandle*/)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request accepted.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MyLifecycleServer::cbHandleAccepted(const std::shared_ptr<GoalHandle> pGoalHandle)
{
  {
    std::lock_guard<std::mutex> lock(mutexGoalHandle_);
    pGoalHandle_ = pGoalHandle;
  }
  executeGoal(pGoalHandle);
}

void MyLifecycleServer::executeGoal(const std::shared_ptr<GoalHandle> pGoalHandle)
{
  const auto target = pGoalHandle->get_goal()->position;
  const auto velocity = pGoalHandle->get_goal()->velocity;

  auto pFeedback = std::make_shared<my_robot_interfaces::action::MoveRobot::Feedback>();
  pFeedback->position = position_;
  pGoalHandle->publish_feedback(pFeedback);

  auto pResult = std::make_shared<my_robot_interfaces::action::MoveRobot::Result>();

  rclcpp::Rate loopRate(updateRate_);

  while (position_ != target) {
    loopRate.sleep();

    {
      std::lock_guard<std::mutex> lock(mutexGoalHandle_);
      const auto goalID = pGoalHandle->get_goal_id();
      if (preemptingGoalIDs_.find(goalID) != preemptingGoalIDs_.end()) {
        pResult->position = position_;
        pResult->message = "Aborted";
        pGoalHandle->abort(pResult);
        preemptingGoalIDs_.erase(goalID);
        return;
      }
    }

    if (pGoalHandle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "Cancelling goal.");
      pResult->position = position_;
      pResult->message = "Cancelled";
      pGoalHandle->canceled(pResult);
      return;
    }

    if (target > position_) {
      position_ += velocity / updateRate_;
      if (position_ > target) {
        position_ = target;
      }
    } else if (target < position_) {
      position_ -= velocity / updateRate_;
      if (position_ < target) {
        position_ = target;
      }
    } else {
      assert(false);
    }

    RCLCPP_INFO(get_logger(), "Moved to %f.", position_);

    pFeedback->position = position_;
    pGoalHandle->publish_feedback(pFeedback);
  }

  RCLCPP_INFO(get_logger(), "Goal succeeded.");

  pResult->position = position_;
  pResult->message = "Succeeded";
  pGoalHandle->succeed(pResult);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<MyLifecycleServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pNode->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}