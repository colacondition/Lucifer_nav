#include <gtest/gtest.h>

#include "decision/decision_state_machine.hpp"

namespace
{
decision::DecisionStateMachine machine()
{
  decision::HpRecoveryConfig hp;
  hp.low_threshold = 120;
  hp.high_threshold = 400;
  decision::TargetConfig targets;
  targets.enable_patrol = true;
  targets.patrol_waypoint_file = "patrol.csv";
  targets.patrol_interval_sec = 10.0;
  return decision::DecisionStateMachine(hp, targets);
}

decision::DecisionInputs active(int hp)
{
  decision::DecisionInputs in;
  in.game_active = true;
  in.current_hp = hp;
  return in;
}

decision::ExecutorEvent success(decision::TargetName target)
{
  return {target, decision::ExecutorEventType::Succeeded};
}
}  // namespace

TEST(DecisionStateMachine, StartsAtWaitHome)
{
  auto sm = machine();
  EXPECT_EQ(decision::toString(sm.state()), "HOME.WAIT_HOME");
}

TEST(DecisionStateMachine, GameStartGoesCenterOnlyAfterHpArrives)
{
  auto sm = machine();
  decision::DecisionInputs missing_hp;
  missing_hp.game_active = true;
  EXPECT_EQ(decision::toString(sm.tick(missing_hp, 1.0).state), "HOME.WAIT_HOME");
  EXPECT_EQ(decision::toString(sm.tick(active(120), 2.0).state), "MOVE.GO_CENTER");
}

TEST(DecisionStateMachine, LowHpGoesHomeAndWaitsForHighThreshold)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto low = active(119);
  EXPECT_EQ(decision::toString(sm.tick(low, 2.0).state), "MOVE.GO_HOME");
  low.executor_event = success(decision::TargetName::Home);
  EXPECT_EQ(decision::toString(sm.tick(low, 3.0).state), "HOME.WAIT_HP");
  EXPECT_EQ(decision::toString(sm.tick(active(399), 4.0).state), "HOME.WAIT_HP");
  EXPECT_EQ(decision::toString(sm.tick(active(400), 5.0).state), "MOVE.GO_CENTER");
}

TEST(DecisionStateMachine, GameInactivePreemptsAndGameResumeReversesWhenHealthy)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  decision::DecisionInputs inactive;
  EXPECT_EQ(decision::toString(sm.tick(inactive, 2.0).state), "MOVE.GO_HOME");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 3.0).state), "MOVE.GO_CENTER");
}

TEST(DecisionStateMachine, GameResumeKeepsGoingHomeWhileRecovering)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  sm.tick(active(100), 2.0);
  decision::DecisionInputs inactive;
  sm.tick(inactive, 3.0);
  EXPECT_EQ(decision::toString(sm.tick(active(200), 4.0).state), "MOVE.GO_HOME");
}

TEST(DecisionStateMachine, GameInactiveChangesWaitHpToWaitHome)
{
  auto sm = machine();
  sm.tick(active(100), 1.0);
  decision::DecisionInputs inactive;
  EXPECT_EQ(decision::toString(sm.tick(inactive, 2.0).state), "HOME.WAIT_HOME");
}

TEST(DecisionStateMachine, MoveAbortKeepsCurrentMoveForExecutorRetry)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto center_aborted = active(500);
  center_aborted.executor_event = decision::ExecutorEvent{
    decision::TargetName::Center, decision::ExecutorEventType::Aborted};
  EXPECT_EQ(decision::toString(sm.tick(center_aborted, 2.0).state), "MOVE.GO_CENTER");

  sm.tick(active(100), 3.0);
  auto home_aborted = active(100);
  home_aborted.executor_event = decision::ExecutorEvent{
    decision::TargetName::Home, decision::ExecutorEventType::Aborted};
  EXPECT_EQ(decision::toString(sm.tick(home_aborted, 4.0).state), "MOVE.GO_HOME");
}

TEST(DecisionStateMachine, CenterSuccessStartsFullPatrolInterval)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  EXPECT_EQ(decision::toString(sm.tick(arrived, 2.0).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 11.9).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 12.0).state), "CENTER.PATROL");
}

TEST(DecisionStateMachine, PatrolSuccessAndAbortRestartWaitInterval)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  sm.tick(arrived, 2.0);
  sm.tick(active(500), 12.0);
  auto completed = active(500);
  completed.executor_event = success(decision::TargetName::Patrol);
  EXPECT_EQ(decision::toString(sm.tick(completed, 15.0).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 24.9).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 25.0).state), "CENTER.PATROL");

  decision::DecisionInputs aborted = active(500);
  aborted.executor_event = decision::ExecutorEvent{
    decision::TargetName::Patrol, decision::ExecutorEventType::Aborted};
  EXPECT_EQ(decision::toString(sm.tick(aborted, 26.0).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 35.9).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 36.0).state), "CENTER.PATROL");
}

TEST(DecisionStateMachine, DisabledOrUnconfiguredPatrolStaysAtWaitCenter)
{
  decision::HpRecoveryConfig hp;
  decision::TargetConfig disabled_targets;
  disabled_targets.enable_patrol = false;
  disabled_targets.patrol_waypoint_file = "patrol.csv";
  disabled_targets.patrol_interval_sec = 10.0;
  decision::DecisionStateMachine disabled_sm(hp, disabled_targets);
  disabled_sm.tick(active(500), 1.0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  disabled_sm.tick(arrived, 2.0);
  EXPECT_EQ(
    decision::toString(disabled_sm.tick(active(500), 100.0).state),
    "CENTER.WAIT_CENTER");

  decision::TargetConfig unconfigured_targets;
  unconfigured_targets.enable_patrol = true;
  decision::DecisionStateMachine unconfigured_sm(hp, unconfigured_targets);
  unconfigured_sm.tick(active(500), 1.0);
  unconfigured_sm.tick(arrived, 2.0);
  EXPECT_EQ(
    decision::toString(unconfigured_sm.tick(active(500), 100.0).state),
    "CENTER.WAIT_CENTER");
}

TEST(DecisionStateMachine, IgnoresStaleExecutorResult)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto stale = active(500);
  stale.executor_event = success(decision::TargetName::Patrol);
  EXPECT_EQ(decision::toString(sm.tick(stale, 2.0).state), "MOVE.GO_CENTER");
}

TEST(DecisionStateMachine, GameInactivePreemptsWaitCenterAndPatrol)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  sm.tick(arrived, 2.0);
  decision::DecisionInputs inactive;
  EXPECT_EQ(decision::toString(sm.tick(inactive, 3.0).state), "MOVE.GO_HOME");

  auto second = machine();
  second.tick(active(500), 1.0);
  second.tick(arrived, 2.0);
  second.tick(active(500), 12.0);
  EXPECT_EQ(decision::toString(second.tick(inactive, 13.0).state), "MOVE.GO_HOME");
}

TEST(DecisionStateMachine, LowHpPreemptsWaitCenterAndPatrol)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  sm.tick(arrived, 2.0);
  EXPECT_EQ(decision::toString(sm.tick(active(119), 3.0).state), "MOVE.GO_HOME");

  auto second = machine();
  second.tick(active(500), 1.0);
  second.tick(arrived, 2.0);
  second.tick(active(500), 12.0);
  EXPECT_EQ(decision::toString(second.tick(active(119), 13.0).state), "MOVE.GO_HOME");
}

TEST(DecisionStateMachine, WaitStateAbortDoesNotChangeDecisionState)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  sm.tick(arrived, 2.0);
  auto aborted = active(500);
  aborted.executor_event = decision::ExecutorEvent{
    decision::TargetName::WaitCenter, decision::ExecutorEventType::Aborted};
  EXPECT_EQ(decision::toString(sm.tick(aborted, 3.0).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 11.9).state), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(sm.tick(active(500), 12.0).state), "CENTER.PATROL");
}
