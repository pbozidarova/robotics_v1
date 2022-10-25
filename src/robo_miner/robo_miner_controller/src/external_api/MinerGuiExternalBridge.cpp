#include "robo_miner_controller/external_api/MinerGuiExternalBridge.h"
#include "robo_miner_controller/helpers/algorithms/FloodFill.h"

#include <cstdint>
#include <stdint.h>

#include <thread>
#include <iostream>
#include <deque>
#include <vector>
#include <ctype.h>
#include <string>
#include <stack>


//Other libraries headers
#include "robo_miner_interfaces/msg/robot_position_response.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

#include "robo_miner_common/defines/RoboMinerTopics.h"
#include "robo_miner_common/message_helpers/RoboMinerMessageHelpers.h"

using robo_miner_interfaces::srv::RobotMove;
using robo_miner_interfaces::srv::FieldMapValidate;
using robo_miner_interfaces::srv::LongestSequenceValidate;
using robo_miner_interfaces::srv::ActivateMiningValidate;

using robo_miner_interfaces::msg::RobotMoveType;
using robo_miner_interfaces::msg::RobotPositionResponse;
using robo_miner_interfaces::msg::UInt8MultiArray;
using robo_miner_interfaces::msg::FieldPoint;


namespace {
constexpr auto NODE_NAME = "MinerControllerExternalBridge";


void fillRobotPositionResponseInfo(const RobotPositionResponse &data) {
	  Direction dir;
	  dir = getRobotDirection(data.robot_dir);
	  std::cout << "moving" << getRobotDirectionField(dir) << std::endl;

	  std::cout << "left" << data.surrounding_tiles[0] << std::endl;
	  std::cout << "forward" << data.surrounding_tiles[1] << std::endl;
	  std::cout << "right" << data.surrounding_tiles[2] << std::endl;
}

constexpr auto PROCESSED_MARKER = '0';

bool isValidMove(char nextTile) {

	const auto validChars = std::string("pgcbr");

  if(validChars.find(nextTile)<validChars.length()){
	  std::cout << nextTile << std::endl;
	  return true;
  }

  return false;
}


bool isValidMove(const FieldData &data, const FieldPos &location) {
  if (0 > location.row) {
    return false;
  }

  if (static_cast<int32_t>(data.size()) <= location.row) {
    return false;
  }

  if (0 > location.col) {
    return false;
  }

  if (!data[0].empty() &&
      static_cast<int32_t>(data[0].size()) <= location.col) {
    return false;
  }
  if (data[location.row][location.col] == 'k') {
    return false;
  }

  return true;
}

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


MinerGuiExternalBridge::MinerGuiExternalBridge()
    : Node(NODE_NAME) {

}

int32_t MinerGuiExternalBridge::init() {
  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);
  _userAuthenticatePublisher = create_publisher<UserAuthenticate>(USER_AUTHENTICATE_TOPIC, qos);

  _queryInitialRobotPositionClient = create_client<QueryInitialRobotPosition>(QUERY_INITIAL_ROBOT_POSITION_SERVICE);
  _robotMoveClient = create_client<RobotMove>(ROBOT_MOVE_SERVICE);
  _fieldMapValidateClient = create_client<FieldMapValidate>(FIELD_MAP_VALIDATE_SERVICE);

  waitForService(_queryInitialRobotPositionClient);
  waitForService(_robotMoveClient);
  waitForService(_fieldMapValidateClient);


  return EXIT_SUCCESS;
}

void MinerGuiExternalBridge::run() {

	currentRow = 0;
	currentCol = 0;

	dataCollected.push_back({});

	const auto &data = queryInitialRobotPosition();
	changeCurrentBotPosition();

	allocateBorders(data);

	std::cout << "max col" << maxColIndex << std::endl;
	std::cout << "max row" << maxRowIndex << std::endl;

	initiliseProcessedMatrix();


	traverseRightUpperMap();
	traverseRightLowerMap();
	traverseLeftLowerMap();

	printMatrix(dataCollected);

	validateMap();
	validateLongestSequence();
}

void MinerGuiExternalBridge::validateMap() {
	  std::shared_ptr<FieldMapValidate::Request> request = std::make_shared<
			  FieldMapValidate::Request>();
	  UInt8MultiArray field_map;
	  field_map.rows = maxRowIndex + 1;
	  field_map.cols = maxColIndex + 1;
	  std::vector<uint8_t> arr;

	  arr.resize( (maxRowIndex + 1) * (maxColIndex + 1) );

	  for (size_t r = 0; r < dataCollected.size(); r++) {
	    for (size_t c = 0; c < dataCollected[r].size(); c++) {
	    	arr[(r * dataCollected[r].size() ) + c] = static_cast<int>(dataCollected[r][c]);
	    }
	  }
	  field_map.data = arr;
	  request->field_map = field_map;

	  auto result = _fieldMapValidateClient->async_send_request(request);
	  const std::shared_ptr<FieldMapValidate::Response> response = result.get();
	  const auto &responseData = response->success;

	  std::cout << "Rows " << request->field_map.rows << std::endl;
	  std::cout << "Cols " << request->field_map.cols << std::endl;

	  auto populatedArr = field_map.data;
	  for (size_t r = 0; r < populatedArr.size(); r++) {
		  std::cout << populatedArr[r];
	  }

	  std::cout << "     is Map validated successfully " << responseData << std::endl;
	  std::cout << "error_reason" << response->error_reason << std::endl;

}

void MinerGuiExternalBridge::validateLongestSequence() {
	  std::shared_ptr<LongestSequenceValidate::Request> request = std::make_shared<
			  LongestSequenceValidate::Request>();
	  std::vector<char> nonCrystalMarkers; //{'r', 'p', 'b', 'g', 'c' };
	  nonCrystalMarkers.push_back('r');
	  nonCrystalMarkers.push_back('p');
	  nonCrystalMarkers.push_back('b');
	  nonCrystalMarkers.push_back('g');
	  nonCrystalMarkers.push_back('c');

	  std::vector<FieldPos> seq;
	  seq = FloodFill::findLongestCrystalSequence(dataCollected, nonCrystalMarkers);

      for (const auto &s : seq) {
    	  FieldPoint point;
    	  point.row = s.row;
    	  point.col = s.col;

    	  request->sequence_points.push_back(point);
      }

	  auto result = _longestSequenceValidateClient->async_send_request(request);
	  const std::shared_ptr<LongestSequenceValidate::Response> response = result.get();
	  const auto &responseData = response->success;
	  std::cout << "is Map validated successfully " << responseData << std::endl;
}

void MinerGuiExternalBridge::activateMining() {
	  std::shared_ptr<ActivateMiningValidate::Request> request = std::make_shared<
			  ActivateMiningValidate::Request>();

	  auto result = _activateMiningValidateClient->async_send_request(request);
	  const std::shared_ptr<ActivateMiningValidate::Response> response = result.get();
	  const auto &responseData = response->success;
	  std::cout << "is Map validated successfully " << responseData << std::endl;
}

RobotPositionResponse MinerGuiExternalBridge::queryInitialRobotPosition() {
  std::shared_ptr<QueryInitialRobotPosition::Request> request = std::make_shared<
	QueryInitialRobotPosition::Request>();

  auto result = _queryInitialRobotPositionClient->async_send_request(request);
  const std::shared_ptr<QueryInitialRobotPosition::Response> response = result.get();
  const auto &robot_initial_tile = response->robot_initial_tile;
  std::cout << robot_initial_tile<< std::endl;
  initialTile = robot_initial_tile;

  const auto &data = response->robot_position_response;

  leftTile = data.surrounding_tiles[0];
  forwardTile = data.surrounding_tiles[1];
  rightTile = data.surrounding_tiles[2];
  direction = data.robot_dir;

  fillRobotPositionResponseInfo(response->robot_position_response);

  return response->robot_position_response;
}

RobotPositionResponse MinerGuiExternalBridge::robotMove(MoveType moveType) {
	  std::shared_ptr<RobotMove::Request> request = std::make_shared<
			  RobotMove::Request>();
	  RobotMoveType msg;
	  msg.move_type = getMoveTypeField(moveType);
	  request->robot_move_type = msg;

	  auto result = _robotMoveClient->async_send_request(request);
	  const std::shared_ptr<RobotMove::Response> response = result.get();
	  const auto &data = response->robot_position_response;

	  leftTile = data.surrounding_tiles[0];
	  forwardTile = data.surrounding_tiles[1];
	  rightTile = data.surrounding_tiles[2];
	  direction = data.robot_dir;

	  std::cout << data.surrounding_tiles[0] << data.surrounding_tiles[1] << data.surrounding_tiles[2] << data.robot_dir << std::endl;

	  if(moveType == MoveType::FORWARD){
		  changeCurrentBotPosition();
	  }

	  if(areReachedAllBorders() > 0){
		populateTiles();
	  }

	  return response->robot_position_response;
}

void MinerGuiExternalBridge::allocateBorders(RobotPositionResponse data){
	const auto neighbours = data.surrounding_tiles;
	checkIsReachedBorder();

	for (int i = 0; i < 3; i++) {
		if(	areReachedAllBorders() > 0) {
			return;
		}else{
			char neighbour = neighbours[i];

			if (!isValidMove(neighbour) || circleCounter == 4) {
				circleCounter = 0;
				continue;
			}
			circleCounter++;
			if(i == 0){
				robotMove(MoveType::ROTATE_LEFT);
			}
			if(i == 2){
				robotMove(MoveType::ROTATE_RIGHT);
			}
			const auto moveData = robotMove(MoveType::FORWARD);
			allocateBorders(moveData);
			if(	areReachedAllBorders() > 0) {
				return;
			}else{
				if(i == 0){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 1){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 2){
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_RIGHT);
				}
			}

		}

	}
}

void MinerGuiExternalBridge::goToPosition(FieldPos destination){

	std::stack<FieldPos> q;
    FieldPos startingPoint = {currentRow, currentCol};

    q.push(startingPoint);

    while (!q.empty()) {
    	FieldPos currentPoint = q.top();

    	if(currentPoint.row == destination.row && currentPoint.col == destination.col){
    		return;
    	}

    	q.pop();
        constexpr auto arrSize = 4;
		const std::array<FieldPos, arrSize> dirs = {
				FieldPos { currentPoint.row, currentPoint.col - 1 },
				FieldPos { currentPoint.row - 1, currentPoint.col },
				FieldPos { currentPoint.row, currentPoint.col + 1 },
				FieldPos { currentPoint.row + 1, currentPoint.col } };

		for (const auto &dir : dirs) {
		  if (isValidMove(dataProcessed, dir)) {
			q.push(dir);

			switch (direction) {
			case 0: //Direction::UP:
				robotMove(MoveType::ROTATE_LEFT);
				break;
			case 1: //Direction::RIGHT:
				robotMove(MoveType::ROTATE_LEFT);
				robotMove(MoveType::ROTATE_LEFT);
				break;
			case 2: // Direction::DOWN:
				robotMove(MoveType::ROTATE_RIGHT);
				break;
			case 3: //Direction::LEFT:
				break;
			default:
			  std::cout << "dir " << direction << std::endl;
			  LOGERR("Error, received unsupported Direction: %d", direction);
			}


	    	while(!isValidMove(forwardTile)){
	    		robotMove(MoveType::ROTATE_LEFT);
	    	}
			robotMove(MoveType::FORWARD);
		  }
		}
    }

}

void MinerGuiExternalBridge::traverseRightUpperMap(){
	std::vector<char> neighbours;
	neighbours.push_back(leftTile);
	neighbours.push_back(forwardTile);
	neighbours.push_back(rightTile);

	std::cout << "traverseRightUpperMap " << std::endl;

	std::cout << "sectionBorderMarker " << sectionBorderMarker << std::endl;
	std::cout << "currentRow " << currentRow << std::endl;


	for (int i = 0; i < 3; i++) {
		if( areReachedAllBorders() > 1	) {
			return;
		}else{
			char neighbour = neighbours[i];

			if (!isValidMove(neighbour) ||
					circleCounter == 4 ||
					(sectionBorderMarker > currentRow + 1 && direction == 2) ||
					(sectionBorderMarker < currentCol && direction == 3)
			) {
				circleCounter = 0;
				continue;
			}
			circleCounter++;

			if(i == 0){
				robotMove(MoveType::ROTATE_LEFT);
			}
			if(i == 2){
				robotMove(MoveType::ROTATE_RIGHT);
			}

			robotMove(MoveType::FORWARD);
			checkIsReachedRightUpperSectionBorder();

			traverseRightUpperMap();

			if( areReachedAllBorders() > 1 ) {
				return;
			}else{
				if(i == 0){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 1){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 2){
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_RIGHT);
				}
			}

		}

	}

}

void MinerGuiExternalBridge::traverseRightLowerMap(){
	std::vector<char> neighbours;
	neighbours.push_back(leftTile);
	neighbours.push_back(forwardTile);
	neighbours.push_back(rightTile);

	std::cout << "traverseRightLowerMap " << std::endl;

	checkIsReachedRightLowerSectionBorder();
	std::cout << "direction " <<  direction << std::endl;

	std::cout << "sectionBorderMarker " << sectionBorderMarker << std::endl;
	std::cout << "currentRow " << currentRow << std::endl;


	for (int i = 0; i < 3; i++) {
		if( areReachedAllBorders() > 2	) {
			return;
		}else{
			char neighbour = neighbours[i];

			if (!isValidMove(neighbour) || circleCounter == 4 ||
					(sectionBorderMarker > currentRow - 1 && direction == 0) ||
					(sectionBorderMarker > currentCol - 1 && direction == 3)
			) {
				circleCounter = 0;
				continue;
			}
			circleCounter++;

			if(i == 0){
				robotMove(MoveType::ROTATE_LEFT);
			}
			if(i == 2){
				robotMove(MoveType::ROTATE_RIGHT);
			}
			robotMove(MoveType::FORWARD);
			checkIsReachedRightLowerSectionBorder();

			traverseRightLowerMap();

			if( areReachedAllBorders() > 2 ) {
				return;
			}else{
				if(i == 0){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 1){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 2){
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_RIGHT);
				}
			}

		}

	}

}

void MinerGuiExternalBridge::traverseLeftLowerMap(){
	if(!isValidMove(leftTile) && !isValidMove(forwardTile) && !isValidMove(rightTile)){
		robotMove(MoveType::ROTATE_RIGHT);
		robotMove(MoveType::ROTATE_RIGHT);
		traverseLeftLowerMap();
	}

	std::vector<char> neighbours;
	neighbours.push_back(leftTile);
	neighbours.push_back(forwardTile);
	neighbours.push_back(rightTile);


	std::cout << "traverseLeftLowerMap " << std::endl;
	std::cout << "direction " <<  direction << std::endl;

	std::cout << "sectionBorderMarker " << sectionBorderMarker << std::endl;
	std::cout << "currentRow " << currentRow << std::endl;

	checkIsReachedLeftLowerSectionBorder();

	for (int i = 0; i < 3; i++) {
		std::cout << "-----------------------------" << std::endl;

		if( areReachedAllBorders() > 3	) {
			return;
		}else{
			char neighbour = neighbours[i];

			if (!isValidMove(neighbour) || circleCounter == 4
//					|| (sectionBorderMarker > currentRow - 1 && direction == 0)
//					|| (sectionBorderMarker > currentCol + 1 && direction == 1 && isReachedRightBorder)
			) {
				circleCounter = 0;
				continue;
			}
			circleCounter++;

			if(i == 0){
				robotMove(MoveType::ROTATE_LEFT);
			}
			if(i == 2){
				robotMove(MoveType::ROTATE_RIGHT);
			}
			robotMove(MoveType::FORWARD);
			checkIsReachedLeftLowerSectionBorder();

			traverseLeftLowerMap();

			if( areReachedAllBorders() > 3 ) {
				return;
			}else{
				if(i == 0){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 1){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 2){
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_RIGHT);
				}
			}

		}

	}

}

void MinerGuiExternalBridge::traverseLeftUpperMap(){
	std::vector<char> neighbours;
	neighbours.push_back(leftTile);
	neighbours.push_back(forwardTile);
	neighbours.push_back(rightTile);

	if(!isValidMove(leftTile) && !isValidMove(forwardTile) && !isValidMove(rightTile)){
		robotMove(MoveType::ROTATE_LEFT);
		traverseLeftLowerMap();
	}

	std::cout << "traverseLeftLowerMap " << std::endl;
	std::cout << "direction " <<  direction << std::endl;

	std::cout << "sectionBorderMarker " << sectionBorderMarker << std::endl;
	std::cout << "currentRow " << currentRow << std::endl;

	checkIsReachedLeftUpperSectionBorder();


	for (int i = 0; i < 3; i++) {
		std::cout << "-----------------------------" << std::endl;

		if( areReachedAllBorders() > 3	) {
			return;
		}else{
			char neighbour = neighbours[i];

			if (!isValidMove(neighbour) || circleCounter == 4 ||
					(sectionBorderMarker > currentRow - 1 && direction == 0) ||
					(sectionBorderMarker > currentCol + 1 && direction == 1)
			) {
				circleCounter = 0;
				continue;
			}
			circleCounter++;

			if(i == 0){
				robotMove(MoveType::ROTATE_LEFT);
			}
			if(i == 2){
				robotMove(MoveType::ROTATE_RIGHT);
			}
			robotMove(MoveType::FORWARD);
			checkIsReachedLeftUpperSectionBorder();

			traverseLeftLowerMap();

			if( areReachedAllBorders() > 3 ) {
				return;
			}else{
				if(i == 0){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 1){
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_LEFT);
					robotMove(MoveType::ROTATE_LEFT);
				}
				if(i == 2){
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::ROTATE_RIGHT);
					robotMove(MoveType::FORWARD);
					robotMove(MoveType::ROTATE_RIGHT);
				}
			}

		}

	}

}


void MinerGuiExternalBridge::populateTiles(){
	switch (direction) {
	case 0: //Direction::UP:
		if(currentRow - 1 >= 0){
			dataCollected[currentRow -1][currentCol] = forwardTile;
		}

		if(currentCol - 1 >=0){
			dataCollected[currentRow][currentCol -1] = leftTile;
		}
		if(currentCol + 1 <= maxColIndex){
			dataCollected[currentRow][currentCol +1] = rightTile;
		}
		break;
	case 1: //Direction::RIGHT:
		if(currentCol + 1 <= maxColIndex){
			dataCollected[currentRow][currentCol+1] = forwardTile;
		}

		if(currentRow-1 >= 0){
			dataCollected[currentRow -1][currentCol] = leftTile;
		}
		if(currentRow+1 <= maxRowIndex){
			dataCollected[currentRow +1][currentCol] = rightTile;
		}

		break;
	case 2: // Direction::DOWN:
		if(currentRow + 1 <= maxRowIndex){
			std::cout << "down " << forwardTile << std::endl;
			dataCollected[currentRow+1][currentCol] = forwardTile;
		}

		if(currentCol - 1 >=0){
			dataCollected[currentRow][currentCol -1] = rightTile;
		}
		if(currentCol + 1 <= maxColIndex){
			dataCollected[currentRow][currentCol +1] = leftTile;
		}
   	    break;
	case 3: //Direction::LEFT:
		if(currentCol - 1 >= 0){
			dataCollected[currentRow][currentCol-1] = forwardTile;
		}

		if(currentRow-1 >= 0){
			dataCollected[currentRow-1][currentCol] = rightTile;
		}
		if(currentRow+1 <= maxRowIndex){
			dataCollected[currentRow+1][currentCol] = leftTile;
		}

		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

void MinerGuiExternalBridge::changeCurrentBotPosition(){
	switch (direction) {
	case 0: //Direction::UP:
		if(currentRow-1 >= 0){
			currentRow--;
		}else{
			maxRowIndex++;
		}

		std::cout << "------------------" << std::endl;
		std::cout << "UP currentRow " << currentRow << std::endl;
		std::cout << "UP currentCol " << currentCol << std::endl;

		std::cout << "UP maxRow " << maxRowIndex << std::endl;
		std::cout << "UP maxCol " << maxColIndex << std::endl;
		break;
	case 1: //Direction::RIGHT:
		currentCol++;
		if(currentCol > maxColIndex) {
			maxColIndex = currentCol;
		}
		std::cout << "------------------" << std::endl;
		  std::cout << "RIGHT currentCol " << currentCol << std::endl;
		  std::cout << "RIGHT currentRow " << currentRow << std::endl;

			std::cout << "UP maxRow " << maxRowIndex << std::endl;
			std::cout << "UP maxCol " << maxColIndex << std::endl;
		  break;
	case 2: // Direction::DOWN:
		currentRow++;
		if(currentRow > maxRowIndex){
			maxRowIndex = currentRow;
		}
		std::cout << "------------------" << std::endl;
		  std::cout << "DOWN currentRow " << currentRow << std::endl;
		  std::cout << "DOWN currentCol " << currentCol << std::endl;

			std::cout << "UP maxRow " << maxRowIndex << std::endl;
			std::cout << "UP maxCol " << maxColIndex << std::endl;
		  break;
	case 3: //Direction::LEFT:
		if(currentCol-1 >= 0){
			currentCol--;
		}else{
			maxColIndex++;
		}

		std::cout << "------------------" << std::endl;
		std::cout << "LEFT currentCol " << currentCol << std::endl;
		std::cout << "LEFT currentRow " << currentRow << std::endl;

		std::cout << "UP maxRow " << maxRowIndex << std::endl;
		std::cout << "UP maxCol " << maxColIndex << std::endl;
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}


void MinerGuiExternalBridge::checkIsReachedBorder(){
	if(forwardTile != '#'){
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

void MinerGuiExternalBridge::checkIsReachedSectionBorder(){
	switch (direction) {
	case 0: //Direction::UP:
		if(forwardTile != '#'){
		  return;
		}
		isReachedUpperBorder = true;
		break;
	case 1: //Direction::RIGHT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		if(sectionBorderMarker == currentRow){
			isReachedLowerBorder = true;
		}
		break;
	case 3: //Direction::LEFT:
		if(forwardTile != '#'){
			isReachedLeftBorder = true;
		}
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

void MinerGuiExternalBridge::checkIsReachedRightUpperSectionBorder(){
	switch (direction) {
	case 0: //Direction::UP:
		if(forwardTile != '#'){
		  return;
		}
		isReachedUpperBorder = true;
		break;
	case 1: //Direction::RIGHT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		if(sectionBorderMarker == currentRow){
			isReachedLowerBorder = true;
		}
		break;
	case 3: //Direction::LEFT:
		if(sectionBorderMarker == currentCol){
			isReachedLeftBorder = true;
		}
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

void MinerGuiExternalBridge::checkIsReachedRightLowerSectionBorder(){
	std::cout << "isReachedLeftBorder" << isReachedLeftBorder << std::endl;
	std::cout << "isReachedRightBorder" << isReachedRightBorder << std::endl;
	std::cout << "isReachedUpperBorder" << isReachedUpperBorder << std::endl;
	std::cout << "isReachedLowerBorder" << isReachedLowerBorder << std::endl;
    std::cout << "areReachedAllBordersFlag " << areReachedAllBordersFlag << std::endl;
	if(sectionBorderMarker == currentRow){
		isReachedUpperBorder = true;
	}

	switch (direction) {
	case 0: //Direction::UP:

		break;
	case 1: //Direction::RIGHT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		if(forwardTile != '#'){
		  return;
		}
		isReachedLowerBorder = true;
		break;
	case 3: //Direction::LEFT:
		if(sectionBorderMarker == currentCol){
			isReachedLeftBorder = true;
		}
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

void MinerGuiExternalBridge::checkIsReachedLeftLowerSectionBorder(){
	std::cout << "isReachedLeftBorder" << isReachedLeftBorder << std::endl;
	std::cout << "isReachedRightBorder" << isReachedRightBorder << std::endl;
	std::cout << "isReachedUpperBorder" << isReachedUpperBorder << std::endl;
	std::cout << "isReachedLowerBorder" << isReachedLowerBorder << std::endl;
    std::cout << "areReachedAllBordersFlag " << areReachedAllBordersFlag << std::endl;

	switch (direction) {
	case 0: //Direction::UP:
		if(sectionBorderMarker == currentRow){
			isReachedUpperBorder = true;
		}
		break;
	case 1: //Direction::RIGHT:
//		if(sectionBorderMarker == currentCol){
//			isReachedRightBorder = true;
//		}
		if(forwardTile != '#'){
		  return;
		}
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		if(forwardTile != '#'){
		  return;
		}
		isReachedLowerBorder = true;
		break;
	case 3: //Direction::LEFT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedLeftBorder = true;
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

void MinerGuiExternalBridge::checkIsReachedLeftUpperSectionBorder(){
	switch (direction) {
	case 0: //Direction::UP:
		if(forwardTile != '#'){
		  return;
		}
		isReachedUpperBorder = true;
		break;
	case 1: //Direction::RIGHT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		if(sectionBorderMarker == currentRow){
			isReachedLowerBorder = true;
		}
		break;
	case 3: //Direction::LEFT:
		if(sectionBorderMarker == currentCol){
			isReachedLeftBorder = true;
		}
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}


void MinerGuiExternalBridge::checkIsReachedLowerSectionBorder(){


	switch (direction) {
	case 0: //Direction::UP:
		if(sectionBorderMarker == currentRow){
			isReachedUpperBorder = true;
		}
		break;
	case 1: //Direction::RIGHT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedRightBorder = true;
		break;
	case 2: // Direction::DOWN:
		if(forwardTile != '#'){
		  return;
		}
		isReachedLowerBorder = true;
		break;
	case 3: //Direction::LEFT:
		if(forwardTile != '#'){
		  return;
		}
		isReachedLeftBorder = true;
		break;
	default:
	  std::cout << "dir " << direction << std::endl;
	  LOGERR("Error, received unsupported Direction: %d", direction);
	}
}

int MinerGuiExternalBridge::areReachedAllBorders(){
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

void MinerGuiExternalBridge::initiliseProcessedMatrix(){
	for (int r = 0; r <= maxRowIndex ; ++r) {
		dataCollected.push_back({});
	  for (int c = 0; c <= maxColIndex ; ++c) {
			dataCollected[r].push_back(PROCESSED_MARKER);
	  }
	}

	for (int r = 0; r <= maxRowIndex ; ++r) {
		dataProcessed.push_back({});
	  for (int c = 0; c <= maxColIndex ; ++c) {
		  dataProcessed[r].push_back(PROCESSED_MARKER);
	  }
	}
}

bool MinerGuiExternalBridge::isMapPopulated(){
	for (int r = 0; r <= maxRowIndex; ++r) {
	  for (int c = 0; c <= maxColIndex; ++c) {
		if(dataCollected[r][c] == (PROCESSED_MARKER)){
			return false;
		}
	  }
	}
	return true;
}

