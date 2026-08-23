#pragma once

#include <optional>
#include <string>

#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>

#include "decision/decision_config.hpp"
#include "decision/types.hpp"

namespace decision
{

// fast_location 的 localization_status 是 "<STATE> <reason>"，例如 "LOST waiting"、"OK accepted"。
// 只认第一个 token。空串不算丢：mapping_nav / 测试不起 fast_location。
inline bool integrityStateIsLost(const std::string & payload)
{
  if (payload.empty()) {
    return false;
  }
  const auto space = payload.find(' ');
  const std::string state =
    space == std::string::npos ? payload : payload.substr(0, space);
  return state == "LOST";
}

class DecisionContext
{
public:
  explicit DecisionContext(CombatConfig combat_config = CombatConfig{});

  void setRobotStatus(const decision_interfaces::msg::RobotStatus & msg, double stamp_sec);
  void setGameStatus(const decision_interfaces::msg::GameStatus & msg, double stamp_sec);
  void setLocalizationStatus(const std::string & payload);

  std::optional<int> effectiveCurrentHp() const;
  std::optional<decision_interfaces::msg::RobotStatus> robotStatus() const;
  std::optional<double> robotStatusStampSec() const;
  std::optional<decision_interfaces::msg::GameStatus> gameStatus() const;

  // 从没收到过 localization_status 时 false；只有第一段是 LOST 才 true。
  bool hasLocalizationStatus() const;
  bool localizationLost() const;
  std::string localizationStatusPayload() const;

  // 依据 is_attacked / shooter_heat 直接推断当前交战态势，不做掉血率推断。
  CombatAssessment combatAssessment() const;

private:
  CombatConfig combat_config_;
  std::optional<std::string> localization_status_payload_;

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
