#include "decision/decision_state.hpp"

#include <type_traits>

namespace decision
{

std::string toString(const DecisionState & state)
{
  return std::visit([](const auto & value) {
    using T = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<T, HomeState>) {
      return value.substate == HomeSubstate::WaitHome ? "HOME.WAIT_HOME" : "HOME.WAIT_HP";
    } else if constexpr (std::is_same_v<T, MoveState>) {
      return value.substate == MoveSubstate::GoHome ? "MOVE.GO_HOME" : "MOVE.GO_CENTER";
    } else {
      return value.substate == CenterSubstate::WaitCenter ? "CENTER.WAIT_CENTER" : "CENTER.PATROL";
    }
  }, state);
}

TargetName targetForState(const DecisionState & state)
{
  return std::visit([](const auto & value) {
    using T = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<T, HomeState>) {
      return value.substate == HomeSubstate::WaitHome ? TargetName::WaitHome : TargetName::WaitHp;
    } else if constexpr (std::is_same_v<T, MoveState>) {
      return value.substate == MoveSubstate::GoHome ? TargetName::Home : TargetName::Center;
    } else {
      return value.substate == CenterSubstate::WaitCenter ? TargetName::WaitCenter : TargetName::Patrol;
    }
  }, state);
}

rclcpp::QoS decisionStateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

std_msgs::msg::String decisionStateMessage(const DecisionState & state)
{
  std_msgs::msg::String message;
  message.data = toString(state);
  return message;
}

}  // namespace decision
