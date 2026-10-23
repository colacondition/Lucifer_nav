// BT 决策场景单测：不起 ROS，用 **真实 XML + 真实叶子节点 + 假 MissionBackend**
// 跑完整决策场景。验证的是"行为等价"，而不只是 XML 结构：
//
//   1. 游戏未进行：在家的等 WaitHome；在中心/外面的先 GoHome 再 WaitHome；
//   2. 低血：回家 → WaitHp；HP 恢复 → GoCenter；在家的直接 WaitHp；
//   3. LOST：Holding（不新发任何 map 系目标）且压过其它分支；
//   4. 到中心前不追敌；Center 成功后中心段才允许 Chase；
//   5. 中心段：目标出现 Chase 抢占 Patrol；目标消失/追击下发失败回到 Patrol，
//      并且 Patrol 能继续下发 —— 旧 FSM "Chase 抢占 Patrol 后 patrol in_flight
//      永久为 true、走位卡死" 的回归测试；
//   6. 走位一组完成 → 组间停顿从完成时刻起算；
//   7. 状态字符串（/decision/state）随激活任务变化。
//
// FakeBackend 的段锁存（DecisionLatch）与生产 MissionRuntime 共用同一份实现。

#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "decision/bt_nodes.hpp"

namespace
{

using namespace decision;      // NOLINT：TargetName / PatrolPath 等
using namespace decision::bt;  // NOLINT

const std::string kTreeFile = std::string(DECISION_SOURCE_TREE_DIR) + "/rmul.xml";

// 假后端：只做"记录下发 / 返回事件 / 给派生输入"，行为完全由测试用例摆布。
class FakeBackend : public MissionBackend
{
public:
  struct FixedDispatch
  {
    TargetName target;
  };

  struct PathDispatch
  {
    TargetName target;
    std::vector<std::size_t> patrol_indices;
    double goal_x{0.0};
    double goal_y{0.0};
    bool has_goal{false};
  };

  // ---- 输入（测试摆布） ----
  double now{0.0};
  bool game_active{false};
  bool hp_low{false};
  bool hp_known{true};
  bool hp_zero{false};
  bool revived_flag{false};
  bool lost{false};
  bool chase_ready{false};
  bool patrol_enabled{true};
  double group_gap{4.5};
  bool patrol_pool_empty{false};
  PatrolPath next_patrol_path{std::vector<std::size_t>{0, 1, 2}, 3};
  ChaseDecision chase_decision{};
  bool patrol_dispatch_ok{true};
  bool chase_dispatch_ok{true};

  // ---- 记录（断言用） ----
  DecisionLatch latch;
  std::vector<FixedDispatch> fixed_dispatches;
  std::vector<PathDispatch> path_dispatches;
  std::deque<std::pair<TargetName, bool>> pending_results;
  std::vector<std::string> log_lines;
  std::string active_state{"BOOT"};
  int chase_report_count{0};
  bool last_chase_dispatched{false};

  // ---- MissionBackend ----
  double nowSec() const override { return now; }
  bool gameActive() const override { return game_active; }
  bool hpLow() const override { return hp_low; }
  bool hpKnown() const override { return hp_known; }
  bool hpZero() const override { return hp_zero; }
  bool revived() const override { return revived_flag; }
  bool localizationLost() const override { return lost; }
  bool homeReached() const override { return latch.home_reached; }
  bool centerReached() const override { return latch.center_reached; }
  bool chaseReady() override { return chase_ready; }

  bool ensureFixedTarget(TargetName target) override
  {
    fixed_dispatches.push_back({target});
    latch.onDispatched(target);
    return true;
  }

  std::optional<bool> takeResult(TargetName target) override
  {
    for (auto it = pending_results.begin(); it != pending_results.end(); ++it) {
      if (it->first != target) {
        continue;
      }
      const bool success = it->second;
      pending_results.erase(it);
      return success;
    }
    return std::nullopt;
  }

  bool centerPatrolEnabled() const override { return patrol_enabled; }
  double patrolGroupGapSec() const override { return group_gap; }
  bool patrolPoolEmpty() const override { return patrol_pool_empty; }
  PatrolPath takePatrolPath() override { return next_patrol_path; }

  bool dispatchPatrol(const PatrolPath & path) override
  {
    if (!patrol_dispatch_ok) {
      return false;
    }
    PathDispatch dispatch;
    dispatch.target = TargetName::Patrol;
    dispatch.patrol_indices = path.indices;
    path_dispatches.push_back(std::move(dispatch));
    return true;
  }

  ChaseDecision evaluateChase() override { return chase_decision; }

  bool dispatchChase(const ChaseDecision & decision) override
  {
    if (!chase_dispatch_ok) {
      return false;
    }
    PathDispatch dispatch;
    dispatch.target = TargetName::Chase;
    dispatch.goal_x = decision.goal_x;
    dispatch.goal_y = decision.goal_y;
    dispatch.has_goal = true;
    path_dispatches.push_back(std::move(dispatch));
    return true;
  }

  void reportChase(const ChaseDecision & /*decision*/, bool dispatched) override
  {
    ++chase_report_count;
    last_chase_dispatched = dispatched;
  }

  void setActiveState(const std::string & state) override { active_state = state; }
  void logInfo(const std::string & message) override { log_lines.push_back(message); }
  void logWarn(const std::string & message) override { log_lines.push_back(message); }

  // ---- 测试工具 ----
  void emitResult(TargetName target, bool success)
  {
    latch.onResult(target, success);
    pending_results.emplace_back(target, success);
  }

  std::size_t countFixed(TargetName target) const
  {
    std::size_t count = 0;
    for (const auto & dispatch : fixed_dispatches) {
      if (dispatch.target == target) {
        ++count;
      }
    }
    return count;
  }

  std::size_t countPath(TargetName target) const
  {
    std::size_t count = 0;
    for (const auto & dispatch : path_dispatches) {
      if (dispatch.target == target) {
        ++count;
      }
    }
    return count;
  }

  TargetName lastFixedTarget() const
  {
    return fixed_dispatches.empty() ? TargetName::Unknown : fixed_dispatches.back().target;
  }

  // 让"目标可用 + 该下发"的最小追击判据。
  void makeChaseActionable(double goal_x = 2.0, double goal_y = 1.0)
  {
    chase_ready = true;
    chase_decision.ready = true;
    chase_decision.move_needed = true;
    chase_decision.should_dispatch = true;
    chase_decision.raw_x = goal_x;
    chase_decision.raw_y = goal_y;
    chase_decision.goal_x = goal_x;
    chase_decision.goal_y = goal_y;
    chase_decision.enemy_distance = 1.5;
    chase_decision.reason = "ready";
  }
};

class BtDecisionScenario : public ::testing::Test
{
protected:
  void SetUp() override
  {
    registerDecisionNodes(factory);
    blackboard = makeDecisionBlackboard();
    setBackend(blackboard, &backend);
    tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(kTreeFile, blackboard));
  }

  BT::NodeStatus tick()
  {
    syncBlackboard(blackboard, backend);
    return tree->tickRoot();
  }

  // 中心段已经到达（测试里想跳过 GoCenter 时用）。
  void markCentered()
  {
    backend.latch.home_reached = false;
    backend.latch.center_reached = true;
    backend.game_active = true;
    backend.hp_known = true;
  }

  FakeBackend backend;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  std::unique_ptr<BT::Tree> tree;
};

// ① 游戏未进行 + 一开始就在家：直接 WaitHome，不发 Home。
TEST_F(BtDecisionScenario, GameInactiveWaitsHomeAtStartup)
{
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HOME");
  EXPECT_EQ(backend.countFixed(TargetName::Home), 0u);
  EXPECT_GE(backend.countFixed(TargetName::WaitHome), 1u);

  // 再拍保持同一分支（不会掉到低优先级）。
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HOME");
}

// ② 游戏结束/未进行但人在中心：先回家（GoHome），到点后 WaitHome。
TEST_F(BtDecisionScenario, GameInactiveGoesHomeWhenAway)
{
  backend.latch.home_reached = false;
  backend.latch.center_reached = true;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "MOVE.GO_HOME_SAFE");
  EXPECT_EQ(backend.lastFixedTarget(), TargetName::HomeSafe);

  backend.now = 3.0;
  backend.emitResult(TargetName::HomeSafe, true);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HOME");
  EXPECT_TRUE(backend.homeReached());
  EXPECT_GE(backend.countFixed(TargetName::WaitHome), 1u);
}

// ③ 低血（在中心）：回家 → WaitHp；HP 恢复后回中心。
TEST_F(BtDecisionScenario, LowHpReturnsHomeWaitsHpThenGoesBackToCenter)
{
  markCentered();
  backend.hp_low = true;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  // 残血但本局没死过 → 走绕远安全线，不走隧道。
  EXPECT_EQ(backend.active_state, "MOVE.GO_HOME_SAFE");
  EXPECT_EQ(backend.lastFixedTarget(), TargetName::HomeSafe);
  EXPECT_FALSE(backend.centerReached());  // 回家动作清掉"已到中心"

  backend.emitResult(TargetName::HomeSafe, true);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HP");
  // wait_hp.csv 已删除：到补给区后原地保持，不再下发 WaitHp。
  EXPECT_EQ(backend.countFixed(TargetName::WaitHp), 0u);

  // HP 恢复到迟滞高阈值以上 → 回中心。
  backend.hp_low = false;
  backend.now = 6.0;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "MOVE.GO_CENTER");
  EXPECT_EQ(backend.lastFixedTarget(), TargetName::Center);

  backend.now = 12.0;
  backend.emitResult(TargetName::Center, true);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(backend.centerReached());
  EXPECT_EQ(backend.active_state, "CENTER.PATROL");
}

// ④ 低血但本来就在家：直接 WaitHp，不要多跑一次 Home。
TEST_F(BtDecisionScenario, LowHpAtHomeWaitsHpWithoutGoingHome)
{
  backend.game_active = true;
  backend.hp_known = true;
  backend.hp_low = true;
  backend.latch.home_reached = true;
  backend.latch.center_reached = false;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HP");
  EXPECT_EQ(backend.countFixed(TargetName::Home), 0u);
  EXPECT_EQ(backend.countFixed(TargetName::HomeSafe), 0u);
  EXPECT_EQ(backend.countFixed(TargetName::WaitHp), 0u);
}

// ④b 血量 0：死亡等待复活，不发任何移动目标。
TEST_F(BtDecisionScenario, DeadHoldsWithoutDispatching)
{
  backend.game_active = true;
  backend.hp_known = true;
  backend.hp_zero = true;
  backend.hp_low = true;
  backend.latch.home_reached = false;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOLDING");
  EXPECT_TRUE(backend.fixed_dispatches.empty());
}

// ④c 刚复活（20% 血/无敌/虚弱）低血回家：走隧道快线，不走安全线。
TEST_F(BtDecisionScenario, RevivedLowHpUsesTunnelRoute)
{
  backend.game_active = true;
  backend.hp_known = true;
  backend.hp_low = true;
  backend.revived_flag = true;
  backend.latch.home_reached = false;
  backend.latch.center_reached = true;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "MOVE.GO_HOME_TUNNEL");
  EXPECT_EQ(backend.lastFixedTarget(), TargetName::HomeTunnel);
  EXPECT_EQ(backend.countFixed(TargetName::HomeSafe), 0u);

  backend.emitResult(TargetName::HomeTunnel, true);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HP");
}

// ④d 刚复活但人已在补给区：直接 WaitHp，不再跑回家。
TEST_F(BtDecisionScenario, RevivedAtHomeWaitsHp)
{
  backend.game_active = true;
  backend.hp_known = true;
  backend.hp_low = true;
  backend.revived_flag = true;
  backend.latch.home_reached = true;
  backend.latch.center_reached = false;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOME.WAIT_HP");
  EXPECT_EQ(backend.countFixed(TargetName::HomeTunnel), 0u);
  EXPECT_EQ(backend.countFixed(TargetName::HomeSafe), 0u);
}

// ⑤ LOST 压过一切且不新发 map 系目标；LOST 解除后回到中心段。
TEST_F(BtDecisionScenario, LocalizationLostHoldsWithoutNewDispatch)
{
  markCentered();
  backend.lost = true;

  const std::size_t fixed_before = backend.fixed_dispatches.size();
  const std::size_t path_before = backend.path_dispatches.size();
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOLDING");
  EXPECT_EQ(backend.fixed_dispatches.size(), fixed_before);
  EXPECT_EQ(backend.path_dispatches.size(), path_before);

  // LOST 解除：回到中心段；先 hold 中心，过了 group_gap 才随机走位。
  backend.lost = false;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "CENTER.PATROL");
  EXPECT_EQ(backend.countPath(TargetName::Patrol), 0u);
  backend.now += 10.0;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_GE(backend.countPath(TargetName::Patrol), 1u);
}

// ⑥ 到中心前不追敌；Center 成功后才允许 Chase。
TEST_F(BtDecisionScenario, NoChaseBeforeCenterReached)
{
  backend.game_active = true;
  backend.hp_known = true;
  backend.latch.home_reached = true;
  backend.latch.center_reached = false;
  backend.makeChaseActionable();

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "MOVE.GO_CENTER");
  EXPECT_EQ(backend.countPath(TargetName::Chase), 0u);

  backend.now = 5.0;
  backend.emitResult(TargetName::Center, true);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(backend.centerReached());
  EXPECT_EQ(backend.active_state, "CENTER.CHASE");
  EXPECT_EQ(backend.countPath(TargetName::Chase), 1u);
}

// ⑦ 回归：Chase 抢占 Patrol 后，目标消失时 Patrol 必须能继续下发（旧 FSM 会永久卡死）。
TEST_F(BtDecisionScenario, ChasePreemptsPatrolThenPatrolResumes)
{
  markCentered();
  backend.chase_ready = false;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);  // onStart：先 hold 中心一个 group_gap
  backend.now = 5.0;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "CENTER.PATROL");
  ASSERT_EQ(backend.countPath(TargetName::Patrol), 1u);

  // 目标出现 → Chase 抢占。
  backend.makeChaseActionable();
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "CENTER.CHASE");
  EXPECT_EQ(backend.countPath(TargetName::Chase), 1u);

  // 目标消失 → 回到走位；如果 Patrol 的 in_flight 没被 onHalted 清掉，这里就不会再下发。
  backend.chase_ready = false;
  backend.chase_decision = ChaseDecision{};
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);  // 先落回 Patrol（onStart 重置 hold）
  backend.now += 10.0;  // 越过组间停顿
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "CENTER.PATROL");
  EXPECT_GT(backend.countPath(TargetName::Patrol), 1u);
}

// ⑧ 追击下发失败 → 交还主决策（走位照跑），不占着这一拍不动。
TEST_F(BtDecisionScenario, ChaseDispatchFailureFallsBackToPatrol)
{
  markCentered();
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);  // 初始化中心 hold
  backend.now = 5.0;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);  // 先走一组，确保 Patrol 已下发
  backend.makeChaseActionable();
  backend.chase_dispatch_ok = false;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  // 追击失败后树在下一拍落回 Patrol；本拍可能仍显示 CHASE。
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.countPath(TargetName::Chase), 0u);
  EXPECT_EQ(backend.active_state, "CENTER.PATROL");
  EXPECT_EQ(backend.countPath(TargetName::Patrol), 1u);
  EXPECT_GE(backend.chase_report_count, 1);
  EXPECT_FALSE(backend.last_chase_dispatched);
}

// ⑨ 走位一组完成：组间停顿从"走完那一刻"起算，停顿内不换组。
TEST_F(BtDecisionScenario, PatrolGroupGapStartsAtCompletionTime)
{
  markCentered();
  backend.group_gap = 4.5;
  backend.chase_ready = false;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);  // onStart：先 hold
  backend.now = 5.0;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(backend.countPath(TargetName::Patrol), 1u);

  backend.now = 100.0;
  backend.emitResult(TargetName::Patrol, true);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.countPath(TargetName::Patrol), 1u);  // 完成这一拍不立刻换组

  backend.now = 103.0;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.countPath(TargetName::Patrol), 1u);  // 停顿未到

  backend.now = 105.0;  // 100 + 4.5 = 104.5 之后
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.countPath(TargetName::Patrol), 2u);  // 停顿到了，取下一组
}

// ⑩ 回血途中 HP 恢复：直接改去中心（旧 FSM 的 recovery_complete），不继续回家。
TEST_F(BtDecisionScenario, HpRecoveredWhileGoingHomeSwitchesToCenter)
{
  backend.game_active = true;
  backend.hp_known = true;
  backend.hp_low = true;
  backend.latch.home_reached = false;
  backend.latch.center_reached = true;

  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "MOVE.GO_HOME_SAFE");

  backend.hp_low = false;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "MOVE.GO_CENTER");
  EXPECT_EQ(backend.lastFixedTarget(), TargetName::Center);
}

// ⑪ 比赛未开始但已经 LOST：保持 Holding（旧 node 的完整性门先于任何下发）。
TEST_F(BtDecisionScenario, LostBeatsGameInactiveWithoutDispatch)
{
  backend.game_active = false;
  backend.lost = true;

  const std::size_t fixed_before = backend.fixed_dispatches.size();
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(backend.active_state, "HOLDING");
  EXPECT_EQ(backend.fixed_dispatches.size(), fixed_before);
}

}  // namespace
