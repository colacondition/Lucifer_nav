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

// 带交战态势的输入：hp 健康、游戏进行，combat 有效并指定交战状态。
decision::DecisionInputs combat(decision::EngagementState state)
{
  decision::DecisionInputs in = active(500);
  in.combat.valid = true;
  in.combat.state = state;
  in.combat.firing = (state == decision::EngagementState::Engaging);
  in.combat.hit = (state != decision::EngagementState::Calm);
  return in;
}

// 把机器人推进到 CENTER.HOLD：游戏开始 -> GoCenter -> 到达中心。
void driveToCenter(decision::DecisionStateMachine & sm, double t0 = 1.0)
{
  sm.tick(active(500), t0);
  auto arrived = active(500);
  arrived.executor_event = success(decision::TargetName::Center);
  sm.tick(arrived, t0 + 1.0);
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

TEST(DecisionStateMachine, CenterArrivalEntersHold)
{
  auto sm = machine();
  driveToCenter(sm);
  EXPECT_EQ(decision::toString(sm.state()), "CENTER.HOLD");
}

TEST(DecisionStateMachine, EngagingHoldsPositionToFireBack)
{
  auto sm = machine();
  driveToCenter(sm);
  // 正面交火：应站定进入 ENGAGE。
  EXPECT_EQ(
    decision::toString(sm.tick(combat(decision::EngagementState::Engaging), 3.0).state),
    "CENTER.ENGAGE");
}

TEST(DecisionStateMachine, SuppressedTriggersReposition)
{
  auto sm = machine();
  driveToCenter(sm);
  // 被压制/被偷：应立即换位脱离。
  EXPECT_EQ(
    decision::toString(sm.tick(combat(decision::EngagementState::Suppressed), 3.0).state),
    "CENTER.REPOSITION");
}

TEST(DecisionStateMachine, CalmHoldStaysInCenterWithoutWandering)
{
  auto sm = machine();
  driveToCenter(sm);
  auto calm = combat(decision::EngagementState::Calm);
  // 平静守中心：长时间无交火也稳定在 HOLD，不产生任何换位自切换。
  // 换位只由交战态势驱动（被压制才 Reposition）。
  auto r1 = sm.tick(calm, 3.0);
  EXPECT_EQ(decision::toString(r1.state), "CENTER.HOLD");
  EXPECT_EQ(r1.reason, "holding");
  auto r2 = sm.tick(calm, 100.0);
  EXPECT_EQ(decision::toString(r2.state), "CENTER.HOLD");
  EXPECT_EQ(r2.reason, "holding");
}

TEST(DecisionStateMachine, EngageHoldsMinDurationBeforeCalm)
{
  auto sm = machine();  // 默认 engage_hold_sec = 0.4
  driveToCenter(sm);
  sm.tick(combat(decision::EngagementState::Engaging), 3.0);
  // 交火后立刻转平静：应保持 ENGAGE 到最小时长。
  EXPECT_EQ(
    decision::toString(sm.tick(combat(decision::EngagementState::Calm), 3.2).state),
    "CENTER.ENGAGE");
  // 超过最小时长后才回到 HOLD。
  EXPECT_EQ(
    decision::toString(sm.tick(combat(decision::EngagementState::Calm), 3.5).state),
    "CENTER.HOLD");
}

TEST(DecisionStateMachine, RepositionEscapesBeforeLowHpRetreat)
{
  auto sm = machine();  // 默认 reposition_grace_sec = 3.0
  driveToCenter(sm);
  // 先进入 REPOSITION（此时血量健康）。
  auto suppressed = combat(decision::EngagementState::Suppressed);
  EXPECT_EQ(decision::toString(sm.tick(suppressed, 3.0).state), "CENTER.REPOSITION");
  // 换位途中血量跌破低阈值：宽限期内仍保持 REPOSITION，不被打断回家。
  auto suppressed_low = suppressed;
  suppressed_low.current_hp = 119;
  EXPECT_EQ(decision::toString(sm.tick(suppressed_low, 3.5).state), "CENTER.REPOSITION");
  // 宽限结束仍低血 → 回家保命。
  EXPECT_EQ(
    decision::toString(sm.tick(suppressed_low, 3.0 + 3.0 + 0.1).state),
    "MOVE.GO_HOME");
}

TEST(DecisionStateMachine, CombatDisabledStaysAtHold)
{
  decision::HpRecoveryConfig hp;
  decision::TargetConfig targets;
  decision::CombatConfig combat_off;
  combat_off.enable = false;
  decision::DecisionStateMachine sm(hp, targets, combat_off);
  driveToCenter(sm);
  // 战斗感知关闭：即便态势数据存在，也稳定守在 HOLD。
  EXPECT_EQ(
    decision::toString(sm.tick(combat(decision::EngagementState::Engaging), 100.0).state),
    "CENTER.HOLD");
}

TEST(DecisionStateMachine, IgnoresStaleExecutorResult)
{
  auto sm = machine();
  sm.tick(active(500), 1.0);
  auto stale = active(500);
  stale.executor_event = success(decision::TargetName::Patrol);
  EXPECT_EQ(decision::toString(sm.tick(stale, 2.0).state), "MOVE.GO_CENTER");
}

TEST(DecisionStateMachine, GameInactivePreemptsCenter)
{
  auto sm = machine();
  driveToCenter(sm);
  decision::DecisionInputs inactive;  // game_active=false
  // 游戏结束优先级高于交战态势：无论在 CENTER 哪个子状态都回家。
  EXPECT_EQ(decision::toString(sm.tick(inactive, 3.0).state), "MOVE.GO_HOME");

  auto second = machine();
  driveToCenter(second);
  second.tick(combat(decision::EngagementState::Engaging), 3.0);  // 处于 ENGAGE
  EXPECT_EQ(decision::toString(second.tick(inactive, 4.0).state), "MOVE.GO_HOME");
}

TEST(DecisionStateMachine, LowHpPreemptsCenter)
{
  auto sm = machine();
  driveToCenter(sm);
  // 低血量撤退优先级高于交战态势。
  EXPECT_EQ(decision::toString(sm.tick(active(119), 3.0).state), "MOVE.GO_HOME");

  auto second = machine();
  driveToCenter(second);
  second.tick(combat(decision::EngagementState::Engaging), 3.0);  // 正在交火
  auto low = combat(decision::EngagementState::Engaging);
  low.current_hp = 119;
  // 即便在交火中，血量见底仍然回家保命。
  EXPECT_EQ(decision::toString(second.tick(low, 4.0).state), "MOVE.GO_HOME");
}
