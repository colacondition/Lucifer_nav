#pragma once

#include <optional>
#include <string>

#include "decision/decision_config.hpp"
#include "decision/decision_state.hpp"

namespace decision
{

enum class ExecutorEventType { Succeeded, Aborted };

struct ExecutorEvent {
  TargetName target;
  ExecutorEventType type;
};

struct DecisionInputs {
  bool game_active{false};
  std::optional<int> current_hp;
  std::optional<ExecutorEvent> executor_event;
  CombatAssessment combat;  // 交战态势，驱动 CENTER 段子状态
};

struct StateMachineResult {
  DecisionState previous_state;
  DecisionState state;
  TargetName target;
  bool transitioned{false};
  std::string reason;
};

class DecisionStateMachine
{
public:
  DecisionStateMachine(
    HpRecoveryConfig hp_config, TargetConfig target_config, CombatConfig combat_config = {});
  const DecisionState & state() const;
  bool hpRecoveryActive() const;
  StateMachineResult tick(const DecisionInputs & inputs, double now_sec);

private:
  bool eventIs(const DecisionInputs & inputs, TargetName target, ExecutorEventType type) const;
  StateMachineResult resultFor(const DecisionState & previous, const std::string & reason) const;
  StateMachineResult transitionTo(
    const DecisionState & previous, DecisionState next, const std::string & reason, double now_sec);

  // 依据交战态势推进 CENTER 段子状态，返回是否发生 CENTER 内部切换。
  StateMachineResult tickCenter(const DecisionInputs & inputs, double now_sec);

  HpRecoveryConfig hp_config_;
  TargetConfig target_config_;
  CombatConfig combat_config_;
  DecisionState state_{HomeState{HomeSubstate::WaitHome}};
  bool hp_recovery_active_{false};
  // 进入当前 CENTER 子状态的时刻，用于最小保持时长与 HOLD 换位计时。
  double center_substate_since_sec_{-1.0e9};
};

}  // namespace decision
