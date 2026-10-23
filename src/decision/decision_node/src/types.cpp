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
    case TargetName::HomeTunnel:
      return "home_tunnel";
    case TargetName::HomeSafe:
      return "home_safe";
    case TargetName::Patrol:
      return "patrol";
    case TargetName::Center:
      return "center";
    case TargetName::WaitCenter:
      return "wait_center";
    case TargetName::Chase:
      return "chase";
    case TargetName::Zone:
      return "zone";
    case TargetName::Unknown:
      return "unknown";
  }

  return "unknown";
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
  if (value == "home_tunnel") {
    return TargetName::HomeTunnel;
  }
  if (value == "home_safe") {
    return TargetName::HomeSafe;
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
  if (value == "chase") {
    return TargetName::Chase;
  }
  if (value == "zone") {
    return TargetName::Zone;
  }
  return TargetName::Unknown;
}

}  // namespace decision
