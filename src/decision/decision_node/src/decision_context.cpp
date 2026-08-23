#include "decision/decision_context.hpp"

#include <string>
#include <utility>

namespace decision
{

DecisionContext::DecisionContext(CombatConfig combat_config)
: combat_config_(std::move(combat_config)) {}

void DecisionContext::setRobotStatus(
  const decision_interfaces::msg::RobotStatus & msg, double stamp_sec)
{
  robot_status_ = TimedRobotStatus{msg, stamp_sec};
}

void DecisionContext::setGameStatus(
  const decision_interfaces::msg::GameStatus & msg, double stamp_sec)
{
  game_status_ = TimedGameStatus{msg, stamp_sec};
}

void DecisionContext::setLocalizationStatus(const std::string & payload)
{
  localization_status_payload_ = payload;
}

std::optional<int> DecisionContext::effectiveCurrentHp() const
{
  if (!robot_status_.has_value()) {
    return std::nullopt;
  }

  return static_cast<int>(robot_status_->msg.current_hp);
}

std::optional<decision_interfaces::msg::RobotStatus> DecisionContext::robotStatus() const
{
  if (!robot_status_.has_value()) {
    return std::nullopt;
  }

  return robot_status_->msg;
}

std::optional<double> DecisionContext::robotStatusStampSec() const
{
  if (!robot_status_.has_value()) {
    return std::nullopt;
  }

  return robot_status_->stamp_sec;
}

std::optional<decision_interfaces::msg::GameStatus> DecisionContext::gameStatus() const
{
  if (!game_status_.has_value()) {
    return std::nullopt;
  }

  return game_status_->msg;
}

bool DecisionContext::hasLocalizationStatus() const
{
  return localization_status_payload_.has_value();
}

bool DecisionContext::localizationLost() const
{
  return localization_status_payload_.has_value() &&
    integrityStateIsLost(*localization_status_payload_);
}

std::string DecisionContext::localizationStatusPayload() const
{
  return localization_status_payload_.value_or(std::string{});
}

CombatAssessment DecisionContext::combatAssessment() const
{
  CombatAssessment out;
  if (!robot_status_.has_value()) {
    return out;  // valid = false
  }

  const auto & msg = robot_status_->msg;
  out.firing = static_cast<int>(msg.shooter_heat) >= combat_config_.firing_heat_threshold;
  out.hit = msg.is_attacked;
  out.valid = true;

  // 直接按信号判定，优先级：开火 > 挨打 > 平静。
  if (out.firing) {
    // 热量高（正在开火/最近射击）：正面交战，站定输出。
    out.state = EngagementState::Engaging;
  } else if (out.hit) {
    // 正在挨打但没开火：被压制/被侧后偷，自瞄没锁到打我的人 → 换位脱离。
    out.state = EngagementState::Suppressed;
  } else {
    out.state = EngagementState::Calm;
  }

  return out;
}

}  // namespace decision
