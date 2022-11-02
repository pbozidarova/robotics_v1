#ifndef CLEANERCONTROLLEREXTERNALBRIDGE_H_
#define CLEANERCONTROLLEREXTERNALBRIDGE_H_

#include <atomic>

#include <deque>
#include <vector>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include <robo_cleaner_interfaces/msg/robot_move_type.hpp>
#include <robo_cleaner_interfaces/msg/user_authenticate.hpp>
#include <robo_cleaner_interfaces/msg/battery_status.hpp>
#include <robo_cleaner_interfaces/msg/initial_robot_state.hpp>

#include <robo_cleaner_interfaces/srv/charge_battery.hpp>
#include <robo_cleaner_interfaces/srv/query_battery_status.hpp>
#include <robo_cleaner_interfaces/srv/query_initial_robot_state.hpp>

#include "robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h"
#include "robo_cleaner_interfaces/action/robot_move.hpp"


class CleanerGuiExternalBridge : public rclcpp::Node {
public:
	CleanerGuiExternalBridge();

  int32_t init();

  void run();

  void shutdown();

private:

  using Empty = std_msgs::msg::Empty;
  using String = std_msgs::msg::String;
  using Int32 = std_msgs::msg::Int32;

  using RobotMoveType = robo_cleaner_interfaces::msg::RobotMoveType;
  using UserAuthenticate = robo_cleaner_interfaces::msg::UserAuthenticate;
  using BatteryStatus = robo_cleaner_interfaces::msg::BatteryStatus;
  using InitialRobotState = robo_cleaner_interfaces::msg::InitialRobotState;

  using ChargeBattery = robo_cleaner_interfaces::srv::ChargeBattery;
  using QueryBatteryStatus = robo_cleaner_interfaces::srv::QueryBatteryStatus;
  using QueryInitialRobotState = robo_cleaner_interfaces::srv::QueryInitialRobotState;

  using RobotMove = robo_cleaner_interfaces::action::RobotMove;
  using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

  void queryInitialRobotState();
  void queryBatteryStatus();
  void chargeBattery(int32_t chargeTurns);
  void onRobotMoveCount(const Int32::SharedPtr msg);
  void sendRobotMoveGoal(MoveType moveType);
  void onRobotMoveGoalResponse(
      std::shared_future<GoalHandleRobotMove::SharedPtr> future);
  void onRobotMoveFeedback(
      const GoalHandleRobotMove::SharedPtr goalHandle,
      const std::shared_ptr<const RobotMove::Feedback> feedback);
  void onRobotMoveResult(
      const GoalHandleRobotMove::WrappedResult &result);


//  RobotPositionResponse robotMove(MoveType moveType);
  void allocateBorders();
//  void populateTiles();
//  void traverseRightUpperMap();
//  void traverseRightLowerMap();
//
//  void traverseLeftLowerMap();
//  void traverseLeftUpperMap();
//
//
  void changeCurrentBotPosition();
  int areReachedAllBorders();
//  void checkIsReachedSectionBorder();
//  void checkIsReachedRightUpperSectionBorder();
//  void checkIsReachedRightLowerSectionBorder();
//
//  void checkIsReachedLeftLowerSectionBorder();
//  void checkIsReachedLeftUpperSectionBorder();
//
//
//  void checkIsReachedLowerSectionBorder();
//
  void checkIsReachedBorder(char tile);
  void initiliseProcessedMatrix();
  void expandProcessedMatrix();
//  bool isMapPopulated();
//  void validateMap();
//  void validateLongestSequence();
//  void activateMining();
//  void goToPosition(FieldPos position);

  std::shared_ptr<rclcpp::Publisher<UserAuthenticate>> _userAuthenticatePublisher;

  rclcpp::Publisher<Empty>::SharedPtr _toggleHelpPagePublisher;
  rclcpp::Publisher<Empty>::SharedPtr _toggleDebugInfoPublisher;
  rclcpp::Publisher<String>::SharedPtr _setDebugMsgPublisher;

  rclcpp_action::Client<RobotMove>::SharedPtr _moveActionClient;

  rclcpp::Client<QueryBatteryStatus>::SharedPtr _batteryStatusClient;
  rclcpp::Client<QueryInitialRobotState>::SharedPtr _initialRobotStateClient;
  rclcpp::Client<ChargeBattery>::SharedPtr _chargeBatteryClient;

  rclcpp::Subscription<Empty>::SharedPtr _shutdownControllerSubscription;
  rclcpp::Subscription<Empty>::SharedPtr _fieldMapReveleadedSubscription;
  rclcpp::Subscription<Empty>::SharedPtr _fieldMapCleanedSubscription;
  rclcpp::Subscription<Int32>::SharedPtr _robotMoveCounterSubscription;

  FieldDescription fieldMap;
  char robotTile;
  int32_t direction;
  int32_t maxMovesOnFullEnergy;
  int32_t movesLeft;
  int32_t robotMoveCounter;

  FieldData dataProcessed;

  FieldData dataCollected;

  int currentRow;
  int currentCol;
  int maxRowIndex = 0;
  int maxColIndex = 0;

  bool isGoalCanceled;

  std::atomic<bool> _isRunning = true;


  bool isReachedLeftBorder;
  bool isReachedRightBorder;
  bool isReachedUpperBorder;
  bool isReachedLowerBorder;

  int areReachedAllBordersFlag = 0;

};

#endif /* CLEANERCONTROLLEREXTERNALBRIDGE_H_ */
