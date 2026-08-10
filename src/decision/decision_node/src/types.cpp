#include "decision/types.hpp"

namespace decision
{

std::string toString(const TargetName target)
{
  switch (target) {
    case TargetName::WaitHome:
      return "wait_home";
    case TargetName::WaitHp:
      return "wait_hp";
    case TargetName::Home:
      return "home";
    case TargetName::Patrol:
      return "patrol";
    case TargetName::Center:
      return "center";
    case TargetName::WaitCenter:
      return "wait_center";
    case TargetName::Unknown:
      return "unknown";
  }

  return "unknown";
}

std::string toString(const EngagementState state)
{
  switch (state) {
    case EngagementState::Calm:
      return "calm";
    case EngagementState::Engaging:
      return "engaging";
    case EngagementState::Suppressed:
      return "suppressed";
  }

  return "calm";
}

TargetName targetNameFromString(const std::string & value)
{
  if (value == "wait_home") {
    return TargetName::WaitHome;
  }
  if (value == "wait_hp") {
    return TargetName::WaitHp;
  }
  if (value == "home") {
    return TargetName::Home;
  }
  if (value == "patrol") {
    return TargetName::Patrol;
  }
  if (value == "center") {
    return TargetName::Center;
  }
  if (value == "wait_center") {
    return TargetName::WaitCenter;
  }
  return TargetName::Unknown;
}

}  // namespace decision
