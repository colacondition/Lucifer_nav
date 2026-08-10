#include "decision/decision_state_machine.hpp"

#include <utility>

namespace decision
{

DecisionStateMachine::DecisionStateMachine(
  HpRecoveryConfig hp_config, TargetConfig target_config, CombatConfig combat_config)
: hp_config_(std::move(hp_config)),
  target_config_(std::move(target_config)),
  combat_config_(std::move(combat_config)) {}

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
  const bool entering_center = std::holds_alternative<CenterState>(next);
  state_ = std::move(next);
  // 任何一次进入/切换到 CENTER 子状态都刷新计时基准，
  // 用于 Reposition/Engage 的最小保持时长判定。
  if (entering_center) {
    center_substate_since_sec_ = now_sec;
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
    // 换位脱离进行中（REPOSITION 刚进入且在宽限期内）：让换位动作先完成，
    // 暂缓低血回家，避免"刚切到 REPOSITION 就被扣血拽回家、路径都还没出来"。
    const bool repositioning = std::holds_alternative<CenterState>(state_) &&
      std::get<CenterState>(state_).substate == CenterSubstate::Reposition;
    const bool escape_remaining =
      now_sec - center_substate_since_sec_ < combat_config_.reposition_grace_sec;
    if (!(repositioning && escape_remaining)) {
      return transitionTo(previous, MoveState{MoveSubstate::GoHome}, "low_hp", now_sec);
    }
    // 宽限内：落到下方 combat 分支继续推进 REPOSITION；
    // 宽限结束仍低血，或换位途中态势转平静，都会在下一拍走到 home/hold。
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
    // 抵达中心，进入交战驱动的 HOLD 子状态。
    return transitionTo(previous, CenterState{CenterSubstate::Hold}, "center_reached", now_sec);
  }
  if (std::holds_alternative<CenterState>(state_)) {
    return tickCenter(inputs, now_sec);
  }

  return resultFor(previous, "");
}

StateMachineResult DecisionStateMachine::tickCenter(
  const DecisionInputs & inputs, double now_sec)
{
  const DecisionState previous = state_;
  const auto current = std::get<CenterState>(state_).substate;
  const double held_for = now_sec - center_substate_since_sec_;

  // 战斗感知关闭或数据无效时，退化为守中心（HOLD），行为等价于原地守点。
  if (!combat_config_.enable || !inputs.combat.valid) {
    if (current != CenterSubstate::Hold) {
      return transitionTo(previous, CenterState{CenterSubstate::Hold}, "combat_disabled", now_sec);
    }
    return resultFor(previous, "hold_no_combat");
  }

  const EngagementState engagement = inputs.combat.state;

  // 被压制/被偷：最高优先，立即换位脱离（Reposition 有最小保持时长防抖）。
  if (engagement == EngagementState::Suppressed) {
    if (current != CenterSubstate::Reposition) {
      return transitionTo(
        previous, CenterState{CenterSubstate::Reposition}, "suppressed_reposition", now_sec);
    }
    return resultFor(previous, "repositioning");
  }

  // 正在换位：保持到最小时长结束再重新评估，避免刚动就被打断。
  if (current == CenterSubstate::Reposition &&
      held_for < combat_config_.reposition_hold_sec) {
    return resultFor(previous, "reposition_hold");
  }

  // 正面交战：站定输出（Engage 有最小保持时长，避免交火状态抖动）。
  if (engagement == EngagementState::Engaging) {
    if (current != CenterSubstate::Engage) {
      return transitionTo(previous, CenterState{CenterSubstate::Engage}, "engage", now_sec);
    }
    return resultFor(previous, "engaging");
  }

  // 刚交火完，保持 Engage 到最小时长再转平静，避免频繁抖动。
  if (current == CenterSubstate::Engage && held_for < combat_config_.engage_hold_sec) {
    return resultFor(previous, "engage_hold");
  }

  // 平静：进入/保持 HOLD。守中心锚点维持占领，不发起身运动——
  // 换位只由交战态势驱动（被压制才 Reposition），不靠定时游走制造动作。
  if (current != CenterSubstate::Hold) {
    return transitionTo(previous, CenterState{CenterSubstate::Hold}, "calm_hold", now_sec);
  }
  return resultFor(previous, "holding");
}

}  // namespace decision
