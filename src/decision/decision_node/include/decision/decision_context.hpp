#pragma once

#include <optional>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "decision/types.hpp"

namespace decision
{

class DecisionContext
{
public:
  void setRobotStatus(const decision_interfaces::msg::RobotStatus & msg, double stamp_sec);
  void setGameStatus(const decision_interfaces::msg::GameStatus & msg, double stamp_sec);

  std::optional<int> effectiveCurrentHp() const;
  std::optional<decision_interfaces::msg::RobotStatus> robotStatus() const;
  std::optional<double> robotStatusStampSec() const;
  std::optional<decision_interfaces::msg::GameStatus> gameStatus() const;

private:
  struct TimedRobotStatus
  {
    decision_interfaces::msg::RobotStatus msg;
    double stamp_sec{0.0};
  };

  struct TimedGameStatus
  {
    decision_interfaces::msg::GameStatus msg;
    double stamp_sec{0.0};
  };

  std::optional<TimedRobotStatus> robot_status_;
  std::optional<TimedGameStatus> game_status_;
};

}  // namespace decision
