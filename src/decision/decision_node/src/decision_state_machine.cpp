#include "decision/decision_state_machine.hpp"

#include <utility>

namespace decision
{

DecisionStateMachine::DecisionStateMachine(
  HpRecoveryConfig hp_config, TargetConfig target_config)
: hp_config_(std::move(hp_config)), target_config_(std::move(target_config)) {}

const DecisionState & DecisionStateMachine::state() const { return state_; }
bool DecisionStateMachine::hpRecoveryActive() const { return hp_recovery_active_; }

bool DecisionStateMachine::eventIs(
  const DecisionInputs & inputs, TargetName target, ExecutorEventType type) const
{
  return inputs.executor_event.has_value() && inputs.executor_event->target == target &&
         inputs.executor_event->type == type;
}

StateMachineResult DecisionStateMachine::resultFor(
  const DecisionState & previous, const std::string & reason) const
{
  return {previous, state_, targetForState(state_), !(state_ == previous), reason};
}

StateMachineResult DecisionStateMachine::transitionTo(
  const DecisionState & previous, DecisionState next, const std::string & reason, double now_sec)
{
  state_ = std::move(next);
  if (state_ == DecisionState{CenterState{CenterSubstate::WaitCenter}}) {
    wait_center_since_sec_ = now_sec;
  }
  return resultFor(previous, reason);
}

StateMachineResult DecisionStateMachine::tick(const DecisionInputs & inputs, double now_sec)
{
  const DecisionState previous = state_;
  if (inputs.current_hp.has_value()) {
    if (hp_recovery_active_ && *inputs.current_hp >= hp_config_.high_threshold) {
      hp_recovery_active_ = false;
    } else if (!hp_recovery_active_ && *inputs.current_hp < hp_config_.low_threshold) {
      hp_recovery_active_ = true;
    }
  }

  if (!inputs.game_active) {
    if (state_ == DecisionState{HomeState{HomeSubstate::WaitHome}}) {
      return resultFor(previous, "game_inactive");
    }
    if (state_ == DecisionState{HomeState{HomeSubstate::WaitHp}}) {
      return transitionTo(previous, HomeState{HomeSubstate::WaitHome}, "game_inactive", now_sec);
    }
    if (state_ == DecisionState{MoveState{MoveSubstate::GoHome}}) {
      if (eventIs(inputs, TargetName::Home, ExecutorEventType::Succeeded)) {
        return transitionTo(previous, HomeState{HomeSubstate::WaitHome}, "home_reached_game_inactive", now_sec);
      }
      return resultFor(previous, "game_inactive");
    }
    return transitionTo(previous, MoveState{MoveSubstate::GoHome}, "game_inactive", now_sec);
  }

  if (hp_recovery_active_) {
    if (state_ == DecisionState{HomeState{HomeSubstate::WaitHome}}) {
      return transitionTo(previous, HomeState{HomeSubstate::WaitHp}, "low_hp_at_home", now_sec);
    }
    if (state_ == DecisionState{HomeState{HomeSubstate::WaitHp}}) {
      return resultFor(previous, "recovering");
    }
    if (state_ == DecisionState{MoveState{MoveSubstate::GoHome}}) {
      if (eventIs(inputs, TargetName::Home, ExecutorEventType::Succeeded)) {
        return transitionTo(previous, HomeState{HomeSubstate::WaitHp}, "home_reached_for_hp", now_sec);
      }
      return resultFor(previous, "recovering");
    }
    return transitionTo(previous, MoveState{MoveSubstate::GoHome}, "low_hp", now_sec);
  }

  if (state_ == DecisionState{HomeState{HomeSubstate::WaitHp}} ||
      state_ == DecisionState{MoveState{MoveSubstate::GoHome}}) {
    return transitionTo(previous, MoveState{MoveSubstate::GoCenter}, "recovery_complete", now_sec);
  }
  if (state_ == DecisionState{HomeState{HomeSubstate::WaitHome}}) {
    if (!inputs.current_hp.has_value()) {
      return resultFor(previous, "waiting_for_hp");
    }
    return transitionTo(previous, MoveState{MoveSubstate::GoCenter}, "game_started", now_sec);
  }
  if (state_ == DecisionState{MoveState{MoveSubstate::GoCenter}} &&
      eventIs(inputs, TargetName::Center, ExecutorEventType::Succeeded)) {
    return transitionTo(previous, CenterState{CenterSubstate::WaitCenter}, "center_reached", now_sec);
  }
  if (state_ == DecisionState{CenterState{CenterSubstate::WaitCenter}}) {
    const bool patrol_ready = target_config_.enable_patrol &&
      !target_config_.patrol_waypoint_file.empty() &&
      now_sec - wait_center_since_sec_ >= target_config_.patrol_interval_sec;
    if (patrol_ready) {
      return transitionTo(previous, CenterState{CenterSubstate::Patrol}, "patrol_interval", now_sec);
    }
    return resultFor(previous, "waiting_center");
  }
  if (state_ == DecisionState{CenterState{CenterSubstate::Patrol}} &&
      (eventIs(inputs, TargetName::Patrol, ExecutorEventType::Succeeded) ||
       eventIs(inputs, TargetName::Patrol, ExecutorEventType::Aborted))) {
    return transitionTo(
      previous, CenterState{CenterSubstate::WaitCenter}, "patrol_finished", now_sec);
  }

  return resultFor(previous, "");
}

}  // namespace decision
