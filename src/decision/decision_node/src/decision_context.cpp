#include "decision/decision_context.hpp"

#include <string>
#include <utility>

namespace decision
{

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

}  // namespace decision
