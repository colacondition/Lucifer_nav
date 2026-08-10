#pragma once

#include <optional>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "decision/decision_config.hpp"
#include "decision/types.hpp"

namespace decision
{

class DecisionContext
{
public:
  explicit DecisionContext(CombatConfig combat_config = CombatConfig{});

  void setRobotStatus(const decision_interfaces::msg::RobotStatus & msg, double stamp_sec);
  void setGameStatus(const decision_interfaces::msg::GameStatus & msg, double stamp_sec);

  std::optional<int> effectiveCurrentHp() const;
  std::optional<decision_interfaces::msg::RobotStatus> robotStatus() const;
  std::optional<double> robotStatusStampSec() const;
  std::optional<decision_interfaces::msg::GameStatus> gameStatus() const;

  // 依据 is_attacked / shooter_heat 直接推断当前交战态势，不做掉血率推断。
  CombatAssessment combatAssessment() const;

private:
  CombatConfig combat_config_;

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
