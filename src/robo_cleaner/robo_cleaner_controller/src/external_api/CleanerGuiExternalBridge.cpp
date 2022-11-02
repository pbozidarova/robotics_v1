#include "robo_cleaner_controller/external_api/CleanerGuiExternalBridge.h"
#include <cstdint>
#include <stdint.h>

#include <thread>
#include <iostream>
#include <deque>
#include <vector>
#include <ctype.h>
#include <string>
#include <stack>
#include <sstream>

#include <std_msgs/msg/int32.hpp>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

#include "robo_cleaner_common/defines/RoboCleanerTopics.h"
#include "robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h"

  using Int32 = std_msgs::msg::Int32;

using robo_cleaner_interfaces::msg::RobotMoveType;
using robo_cleaner_interfaces::msg::UserAuthenticate;
using robo_cleaner_interfaces::msg::BatteryStatus;
using robo_cleaner_interfaces::msg::InitialRobotState;

using robo_cleaner_interfaces::srv::ChargeBattery;
using robo_cleaner_interfaces::srv::QueryBatteryStatus;
using robo_cleaner_interfaces::srv::QueryInitialRobotState;

using robo_cleaner_interfaces::action::RobotMove;




namespace {
constexpr auto NODE_NAME = "CleanerControllerExternalBridge";
using namespace std::literals;
using namespace std::placeholders;
constexpr auto PROCESSED_MARKER = '0';

void printMatrix(FieldData  matrix){
	  for (size_t r = 0; r < matrix.size(); ++r) {
	    for (size_t c = 0; c < matrix[r].size(); ++c) {
	    	std::cout << matrix[r][c];
	    }
	    std::cout<<""<< std::endl;
	  }
}
}

struct InitialRobotPos {
  SurroundingTiles surroundingTiles;
  Direction robotDir = Direction::UP;
  uint8_t robotTile = RoboCommonDefines::UNKNOWN_FIELD_MARKER;
};

template <typename T>
void waitForService(const T &client) {
  using namespace std::literals;
  while (!client->wait_for_service(1s)) {
    std::cout << "Service: [" << client->get_service_name()
    << "] not available. Waiting for 1s ..." << std::endl;
  }
}

template <typename T, typename ActionName>
void waitForAction(const T &action, const ActionName &actionName) {
  while (!action->wait_for_action_server(1s)) {
    std::cout << "Action: [" << actionName
    << "] not available. Waiting for 1s ..." << std::endl;
  }
}


CleanerGuiExternalBridge::CleanerGuiExternalBridge()
    : Node(NODE_NAME) {
}

int32_t CleanerGuiExternalBridge::init() {
  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);
  rclcpp::SubscriptionOptions subsriptionOptions;

  _userAuthenticatePublisher = create_publisher<UserAuthenticate>(USER_AUTHENTICATE_TOPIC, qos);

  _initialRobotStateClient = create_client<QueryInitialRobotState>(QUERY_INITIAL_ROBOT_STATE_SERVICE);
  _chargeBatteryClient = create_client<ChargeBattery>(CHARGE_BATTERY_SERVICE);
  _batteryStatusClient = create_client<QueryBatteryStatus>(QUERY_BATTERY_STATUS_SERVICE);

  _robotMoveCounterSubscription = create_subscription<Int32>(
		  ROBOT_MOVE_COUNTER_TOPIC, qos,
	      std::bind(&CleanerGuiExternalBridge::onRobotMoveCount, this,
	          std::placeholders::_1), subsriptionOptions);

  _moveActionClient = rclcpp_action::create_client<RobotMove>(this,
		  ROBOT_MOVE_ACTION);

  waitForService(_initialRobotStateClient);
  waitForService(_chargeBatteryClient);
  waitForService(_batteryStatusClient);

  waitForAction(_moveActionClient, ROBOT_MOVE_ACTION);

  return EXIT_SUCCESS;
}

void CleanerGuiExternalBridge::run() {

	currentRow = 0;
	currentCol = 0;

	dataCollected.push_back({});

	queryInitialRobotState();
	queryBatteryStatus();
	chargeBattery(0);
	initiliseProcessedMatrix();
	allocateBorders();

	std::cout << "isGoalCanceled " << isGoalCanceled << std::endl;

	std::cout << "max col" << maxColIndex << std::endl;
	std::cout << "max row" << maxRowIndex << std::endl;

	printMatrix(dataCollected);

}

void CleanerGuiExternalBridge::shutdown() {
  std::cout << "Initiating shutdown..." << std::endl;
  _isRunning = false;
}

void CleanerGuiExternalBridge::queryInitialRobotState() {
  std::shared_ptr<QueryInitialRobotState::Request> request = std::make_shared<
		  QueryInitialRobotState::Request>();

  auto result = _initialRobotStateClient->async_send_request(request);
  const std::shared_ptr<QueryInitialRobotState::Response> response = result.get();

  const auto &data = response->initial_robot_state;

  robotTile = data.robot_tile;
  direction = data.robot_dir;
  std::cout << "robot_tile " << robotTile << std::endl;
  std::cout << "direction " << direction << std::endl;
}

void CleanerGuiExternalBridge::queryBatteryStatus() {
  std::shared_ptr<QueryBatteryStatus::Request> request = std::make_shared<
		  QueryBatteryStatus::Request>();

  auto result = _batteryStatusClient->async_send_request(request);
  const std::shared_ptr<QueryBatteryStatus::Response> response = result.get();

  const auto &data = response->battery_status;

  maxMovesOnFullEnergy = data.max_moves_on_full_energy;
  movesLeft = data.moves_left;
  std::cout << "maxMovesOnFullEnergy " << maxMovesOnFullEnergy << std::endl;
  std::cout << "movesLeft " << movesLeft << std::endl;
}

void CleanerGuiExternalBridge::chargeBattery(int32_t chargeTurns) {
  std::shared_ptr<ChargeBattery::Request> request = std::make_shared<
		  ChargeBattery::Request>();
  request->charge_turns = chargeTurns;

  auto result = _chargeBatteryClient->async_send_request(request);
  const std::shared_ptr<ChargeBattery::Response> response = result.get();

  const auto &data = response->battery_status;
  maxMovesOnFullEnergy = data.max_moves_on_full_energy;
  movesLeft = data.moves_left;

  std::cout << "charge success " << response->success << std::endl;
  std::cout << "charge error_reason " << response->error_reason << std::endl;
  std::cout << "charge turns_spend_charging " << response->turns_spend_charging << std::endl;
}

void CleanerGuiExternalBridge::onRobotMoveCount(
    const Int32::SharedPtr msg) {

  robotMoveCounter = msg->data;
  std::cout << "robotMoveCounter " << robotMoveCounter << std::endl;
}

void CleanerGuiExternalBridge::sendRobotMoveGoal(MoveType moveType) {
	  isGoalCanceled = false;
	  auto goalMsg = RobotMove::Goal();
	  goalMsg.robot_move_type.move_type = getMoveTypeField(moveType);

	  std::cout << "Sending RobotMove goal with move type: " << getMoveTypeField(moveType)
	            << std::endl;

	  auto sendGoalOptions =
	      rclcpp_action::Client<RobotMove>::SendGoalOptions();
	  sendGoalOptions.goal_response_callback = std::bind(
	      &CleanerGuiExternalBridge::onRobotMoveGoalResponse, this, _1);
	  sendGoalOptions.feedback_callback = std::bind(
	      &CleanerGuiExternalBridge::onRobotMoveFeedback, this, _1, _2);
	  sendGoalOptions.result_callback = std::bind(
	      &CleanerGuiExternalBridge::onRobotMoveResult, this, _1);

	  _moveActionClient->async_send_goal(goalMsg, sendGoalOptions);

}

void CleanerGuiExternalBridge::onRobotMoveGoalResponse(
    std::shared_future<GoalHandleRobotMove::SharedPtr> future) {
  const auto goal_handle = future.get();
  if (!goal_handle) {
    std::cerr << "Goal was rejected by server" << std::endl;
  } else {
    std::cout << "Goal accepted by server, waiting for result" << std::endl;
  }
}

void CleanerGuiExternalBridge::onRobotMoveFeedback(
    [[maybe_unused]]const GoalHandleRobotMove::SharedPtr goalHandle,
    const std::shared_ptr<const RobotMove::Feedback> feedback) {
  std::cout << "approaching_field_marker: " << feedback->approaching_field_marker << '\n'
            << feedback->progress_percent << std::endl;

  checkIsReachedBorder(feedback->approaching_field_marker);

  if(feedback->approaching_field_marker == 'x' || feedback->approaching_field_marker == 'X' || feedback->approaching_field_marker == '#'){
	  //send cancel request
	  _moveActionClient->async_cancel_all_goals();
  }
}

void CleanerGuiExternalBridge::onRobotMoveResult(
    const GoalHandleRobotMove::WrappedResult &wrappedResult) {
  switch (wrappedResult.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
	isGoalCanceled = 0;
    break;
  case rclcpp_action::ResultCode::ABORTED:
	isGoalCanceled = 1;
    std::cerr << "Goal was aborted" << std::endl;
    return;
  case rclcpp_action::ResultCode::CANCELED:
	isGoalCanceled = 1;
    std::cerr << "Goal was aborted" << std::endl;
    return;
  default:
    std::cerr << "Unknown result code: "
              << static_cast<int32_t>(wrappedResult.code) << std::endl;
    return;
  }

  const auto &result = wrappedResult.result;

  std::cout << "Success " << std::boolalpha << result->success
            << ", error_reason: " << result->error_reason << '\n'
            << result->processed_field_marker << std::endl;
}

void CleanerGuiExternalBridge::allocateBorders(){

	if(	areReachedAllBorders() > 0 && _isRunning) {
			return;
	} else {
		sendRobotMoveGoal(MoveType::FORWARD);

	    std::this_thread::sleep_for(5s);

	    if(!isGoalCanceled){
		  changeCurrentBotPosition();
		  printMatrix(dataProcessed);
		} else {
		  sendRobotMoveGoal(MoveType::ROTATE_RIGHT);
		  if( direction + 1 >= 4){
			  direction = 0;
		  }else {
			  direction++;
		  }
		}
	    allocateBorders();
	}
}

void CleanerGuiExternalBridge::changeCurrentBotPosition(){
	switch (direction) {
	case 0: //Direction::UP:
		if(currentRow-1 >= 0){
			currentRow--;
		}else{
			maxRowIndex++;
			expandProcessedMatrix();
		}

		std::cout << "------------------" << std::endl;
		std::cout << "UP currentRow " << currentRow << std::endl;
		std::cout << "UP currentCol " << currentCol << std::endl;

		std::cout << "maxRow " << maxRowIndex << std::endl;
		std::cout << "maxCol " << maxColIndex << std::endl;
		break;
	case 1: //Direction::RIGHT:
		currentCol++;
		if(currentCol > maxColIndex) {
			maxColIndex = currentCol;
			expandProcessedMatrix();
		}
		std::cout << "------------------" << std::endl;
		  std::cout << "RIGHT currentCol " << currentCol << std::endl;
		  std::cout << "RIGHT currentRow " << currentRow << std::endl;

			std::cout << "maxRow " << maxRowIndex << std::endl;
			std::cout << "maxCol " << maxColIndex << std::endl;
		  break;
	case 2: // Direction::DOWN:
		currentRow++;
		if(currentRow > maxRowIndex){
			maxRowIndex = currentRow;
			expandProcessedMatrix();
		}
		std::cout << "------------------" << std::endl;
		  std::cout << "DOWN currentRow " << currentRow << std::endl;
		  std::cout << "DOWN currentCol " << currentCol << std::endl;

			std::cout << "maxRow " << maxRowIndex << std::endl;
			std::cout << "maxCol " << maxColIndex << std::endl;
		  break;
	case 3: //Direction::LEFT:
		if(currentCol-1 >= 0){
			currentCol--;
		}else{
			maxColIndex++;
			expandProcessedMatrix();
		}
		std::cout << "------------------" << std::endl;
		std::cout << "LEFT currentCol " << currentCol << std::endl;
		std::cout << "LEFT currentRow " << currentRow << std::endl;

		std::cout << "maxRow " << maxRowIndex << std::endl;
		std::cout << "maxCol " << maxColIndex << std::endl;
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}
//
void CleanerGuiExternalBridge::checkIsReachedBorder(char tile){
	if(tile != '#'){
	  return;
	}
	switch (direction) {
	case 0: //Direction::UP:
		isReachedUpperBorder = true;
		break;
	case 1: //Direction::RIGHT:
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		isReachedLowerBorder = true;
		break;
	case 3: //Direction::LEFT:
		isReachedLeftBorder = true;
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

int CleanerGuiExternalBridge::areReachedAllBorders(){
	if(isReachedUpperBorder &&
			isReachedRightBorder &&
			isReachedLowerBorder &&
			isReachedLeftBorder) {
		isReachedUpperBorder  = false;
					isReachedRightBorder = false;
					isReachedLowerBorder = false;
					isReachedLeftBorder = false;

		areReachedAllBordersFlag++;
	}

  return areReachedAllBordersFlag;
}

void CleanerGuiExternalBridge::expandProcessedMatrix(){
	switch (direction) {
		case 0: //Direction::UP:
		    dataProcessed.insert(dataProcessed.begin(), {2});
			break;
		case 1: //Direction::RIGHT:
			for (int r = 0; r <= maxRowIndex; ++r) {
			  std::cout<< "row " << r << std::endl;
			  dataProcessed[r].push_back(PROCESSED_MARKER);
			}
  		    break;
		case 2: // Direction::DOWN:
		    dataProcessed.push_back({});
			break;
		case 3: //Direction::LEFT:
			for (int r = 0; r <= maxRowIndex; ++r) {
			    dataProcessed[r].insert(dataProcessed[r].begin(), PROCESSED_MARKER);
			}
			break;
		default:
		  std::cout << "dir " << direction << std::endl;
		  LOGERR("Error, received unsupported Direction: %d", direction);
		}
}

void CleanerGuiExternalBridge::initiliseProcessedMatrix(){
	for (int r = 0; r <= maxRowIndex ; ++r) {
		dataProcessed.push_back({});
	  for (int c = 0; c <= maxColIndex ; ++c) {
		  dataProcessed[r].push_back(PROCESSED_MARKER);
	  }
	}
}


