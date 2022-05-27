//Corresponding header
#include "robo_cleaner_gui/external_api/CleanerControllerExternalBridge.h"

//System headers

//Other libraries headers
#include "robo_cleaner_common/defines/RoboCleanerTopics.h"
#include "robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h"
#include "robo_common/defines/RoboCommonDefines.h"
#include "utils/rng/Rng.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

using namespace std::placeholders;

CleanerControllerExternalBridge::CleanerControllerExternalBridge()
    : Node("CleanerControllerExternalBridge") {

}

ErrorCode CleanerControllerExternalBridge::init(
    const CleanerControllerExternalBridgeOutInterface &interface) {
  if (ErrorCode::SUCCESS != initOutInterface(interface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initCommunication()) {
    LOGERR("Error, initCommunication() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void CleanerControllerExternalBridge::publishShutdownController() {
  _shutdownControllerPublisher->publish(Empty());

  const auto f = [this]() {
    _outInterface.systemShutdownCb();
  };
  _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void CleanerControllerExternalBridge::publishFieldMapRevealed() {
//  _outInterface.solutionValidator->fieldMapRevealed();
  _fieldMapReveleadedPublisher->publish(Empty());
}

void CleanerControllerExternalBridge::publishFieldMapCleaned() {
//  _outInterface.solutionValidator->fieldMapRevealed();
  _fieldMapCleanedPublisher->publish(Empty());
}

ErrorCode CleanerControllerExternalBridge::initOutInterface(
    const CleanerControllerExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.invokeActionEventCb) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.robotActCb) {
    LOGERR("Error, nullptr provided for RobotActCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode CleanerControllerExternalBridge::initCommunication() {
  constexpr auto queueSize = 10;
  _shutdownControllerPublisher = create_publisher<Empty>(
      SHUTDOWN_CONTROLLER_TOPIC, queueSize);

  _fieldMapReveleadedPublisher = create_publisher<Empty>(
      FIELD_MAP_REVEALED_TOPIC, queueSize);

  _fieldMapCleanedPublisher = create_publisher<Empty>(FIELD_MAP_CLEANED_TOPIC,
      queueSize);

  _moveActionServer = rclcpp_action::create_server<RobotMove>(this,
      ROBOT_MOVE_ACTION,
      std::bind(&CleanerControllerExternalBridge::handleMoveGoal, this, _1, _2),
      std::bind(&CleanerControllerExternalBridge::handleMoveCancel, this, _1),
      std::bind(&CleanerControllerExternalBridge::handleMoveAccepted, this,
          _1));

  return ErrorCode::SUCCESS;
}

rclcpp_action::GoalResponse CleanerControllerExternalBridge::handleMoveGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const RobotMove::Goal> goal) {
  LOG("Received goal request with moveType: %hhd and uuid: %s",
      goal->robot_move_type.move_type,
      rclcpp_action::to_string(uuid).c_str());

  const auto moveType = getMoveType(goal->robot_move_type.move_type);
  if (MoveType::UNKNOWN == moveType) {
    LOGERR("Error, Rejecting goal because of unsupported MoveType");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CleanerControllerExternalBridge::handleMoveCancel(
    [[maybe_unused]]const std::shared_ptr<GoalHandleRobotMove> goalHandle) {
  LOG("Received request to cancel goal with uuid: %s",
      rclcpp_action::to_string(goalHandle->get_goal_id()).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CleanerControllerExternalBridge::handleMoveAccepted(
    const std::shared_ptr<GoalHandleRobotMove> goalHandle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread { std::bind(&CleanerControllerExternalBridge::executeMove, this,
      _1), goalHandle }.detach();
}

void CleanerControllerExternalBridge::executeMove(
    const std::shared_ptr<GoalHandleRobotMove> goalHandle) {
  LOG("Executing goal");
  const auto goal = goalHandle->get_goal();
  auto feedback = std::make_shared<RobotMove::Feedback>();
  auto result = std::make_shared<RobotMove::Result>();

  constexpr auto maxMoves = 10;
  constexpr auto lowIdx = 2;
  constexpr auto highIdx = maxMoves - 2;
  const auto triggerIdx = Rng::getInstance().getRandomNumber(lowIdx, highIdx);

  for (auto i = 0; i < maxMoves; ++i) {
    if (goalHandle->is_canceling()) {
      result->success = false;
      goalHandle->canceled(result);
      LOG("Goal canceled");
      return;
    }

    feedback->approaching_obstacle = (i <= triggerIdx) ? false : true;
    goalHandle->publish_feedback(feedback);
    LOG("Published feedback idx: %d and approaching_obstacle: %d", i,
        static_cast<int32_t>(feedback->approaching_obstacle));

    using namespace std::literals;
    std::this_thread::sleep_for(100ms);
  }

  result->success = true;
  goalHandle->succeed(result);
  LOG("Goal succeeded");
}

