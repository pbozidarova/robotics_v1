#ifndef MINERCONTROLLEREXTERNALBRIDGE_H_
#define MINERCONTROLLEREXTERNALBRIDGE_H_

#include <deque>
#include <vector>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <robo_miner_interfaces/msg/robot_move_type.hpp>
#include <robo_miner_interfaces/msg/user_authenticate.hpp>
#include <robo_miner_interfaces/msg/robot_position_response.hpp>
#include <robo_miner_interfaces/srv/activate_mining_validate.hpp>
#include <robo_miner_interfaces/srv/field_map_validate.hpp>
#include <robo_miner_interfaces/srv/longest_sequence_validate.hpp>
#include <robo_miner_interfaces/srv/query_initial_robot_position.hpp>
#include <robo_miner_interfaces/srv/robot_move.hpp>
#include "robo_miner_common/message_helpers/RoboMinerMessageHelpers.h"


class MinerGuiExternalBridge : public rclcpp::Node {
public:
	MinerGuiExternalBridge();

  int32_t init();

  void run();

private:
  using UserAuthenticate = robo_miner_interfaces::msg::UserAuthenticate;
  using RobotMoveType = robo_miner_interfaces::msg::RobotMoveType;
  using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
  using RobotMove = robo_miner_interfaces::srv::RobotMove;
  using UInt8MultiArray = robo_miner_interfaces::msg::UInt8MultiArray;
  using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
  using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
  using ActivateMiningValidate = robo_miner_interfaces::srv::ActivateMiningValidate;

  using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;

  RobotPositionResponse queryInitialRobotPosition();
  RobotPositionResponse robotMove(MoveType moveType);
  void allocateBorders(RobotPositionResponse data);
  void populateTiles();
  void traverseRightUpperMap();
  void traverseRightLowerMap();

  void traverseLeftLowerMap();
  void traverseLeftUpperMap();


  void changeCurrentBotPosition();
  int areReachedAllBorders();
  void checkIsReachedSectionBorder();
  void checkIsReachedRightUpperSectionBorder();
  void checkIsReachedRightLowerSectionBorder();

  void checkIsReachedLeftLowerSectionBorder();
  void checkIsReachedLeftUpperSectionBorder();


  void checkIsReachedLowerSectionBorder();

  void checkIsReachedBorder();
  void initiliseProcessedMatrix();
  bool isMapPopulated();
  void validateMap();
  void validateLongestSequence();
  void activateMining();
  void goToPosition(FieldPos position);

  std::shared_ptr<rclcpp::Publisher<UserAuthenticate>> _userAuthenticatePublisher;
  std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> _queryInitialRobotPositionClient;
  std::shared_ptr<rclcpp::Client<RobotMove>> _robotMoveClient;
  std::shared_ptr<rclcpp::Client<FieldMapValidate>> _fieldMapValidateClient;
  std::shared_ptr<rclcpp::Client<LongestSequenceValidate>> _longestSequenceValidateClient;
  std::shared_ptr<rclcpp::Client<ActivateMiningValidate>> _activateMiningValidateClient;


  FieldDescription fieldMap;
  char initialTile;
  char leftTile;
  char forwardTile;
  char rightTile;
  int32_t direction;

  FieldData dataProcessed;
  FieldData dataCollected;

  int currentRow;
  int currentCol;
  int maxRowIndex;
  int maxColIndex;

  int circleCounter = 0;
  int sectionBorderMarker = 2;


  bool isReachedLeftBorder;
  bool isReachedRightBorder;
  bool isReachedUpperBorder;
  bool isReachedLowerBorder;

//  bool isReachedSectionBorder;

  int areReachedAllBordersFlag = 0;

};

#endif /* MINERCONTROLLEREXTERNALBRIDGE_H_ */
