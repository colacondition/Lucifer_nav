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
      switch (value.substate) {
        case CenterSubstate::Hold:
          return "CENTER.HOLD";
        case CenterSubstate::Engage:
          return "CENTER.ENGAGE";
        case CenterSubstate::Reposition:
          return "CENTER.REPOSITION";
      }
      return "CENTER.HOLD";
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
      // 中心区子状态的静态映射仅作占位，实际下发由 node 层战术点池决定：
      // Hold/Reposition 从 patrol.csv 的点里挑单点，Engage 不下发站定输出。
      // Reposition 特意不再映射到 Patrol，避免任何路径把它当成跑完整巡逻航线。
      switch (value.substate) {
        case CenterSubstate::Hold:
          return TargetName::WaitCenter;
        case CenterSubstate::Engage:
          return TargetName::Center;
        case CenterSubstate::Reposition:
          return TargetName::WaitCenter;
      }
      return TargetName::WaitCenter;
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
