#pragma once

#include <string>
#include <variant>

#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>

#include "decision/types.hpp"

namespace decision
{

enum class HomeSubstate { WaitHome, WaitHp };
enum class MoveSubstate { GoHome, GoCenter };
// 中心区交战驱动子状态：
//   Hold       平静占区，守视野点并周期性换位（破静止）
//   Engage     正面交火，站定当前位专注输出
//   Reposition 被压制/被偷，换位脱离重找视野
enum class CenterSubstate { Hold, Engage, Reposition };

struct HomeState {
  HomeSubstate substate;
  friend bool operator==(const HomeState & a, const HomeState & b) { return a.substate == b.substate; }
};
struct MoveState {
  MoveSubstate substate;
  friend bool operator==(const MoveState & a, const MoveState & b) { return a.substate == b.substate; }
};
struct CenterState {
  CenterSubstate substate;
  friend bool operator==(const CenterState & a, const CenterState & b) { return a.substate == b.substate; }
};

using DecisionState = std::variant<HomeState, MoveState, CenterState>;

std::string toString(const DecisionState & state);
TargetName targetForState(const DecisionState & state);
rclcpp::QoS decisionStateQos();
std_msgs::msg::String decisionStateMessage(const DecisionState & state);

}  // namespace decision
