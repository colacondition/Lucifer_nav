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
enum class CenterSubstate { WaitCenter, Patrol };

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
