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
  DecisionStateMachine(HpRecoveryConfig hp_config, TargetConfig target_config);
  const DecisionState & state() const;
  bool hpRecoveryActive() const;
  StateMachineResult tick(const DecisionInputs & inputs, double now_sec);

private:
  bool eventIs(const DecisionInputs & inputs, TargetName target, ExecutorEventType type) const;
  StateMachineResult resultFor(const DecisionState & previous, const std::string & reason) const;
  StateMachineResult transitionTo(
    const DecisionState & previous, DecisionState next, const std::string & reason, double now_sec);

  HpRecoveryConfig hp_config_;
  TargetConfig target_config_;
  DecisionState state_{HomeState{HomeSubstate::WaitHome}};
  bool hp_recovery_active_{false};
  double wait_center_since_sec_{-1.0e9};
};

}  // namespace decision
