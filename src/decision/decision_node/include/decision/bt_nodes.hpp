#pragma once

// BehaviorTree.CPP v3 决策层：黑板约定、运行时接口、条件/动作节点。
//
// 这个头文件**不依赖 ROS**（只依赖 behaviortree_cpp_v3 + decision 里的纯数据头），所以
// `test/test_bt_tree.cpp` 可以不起 ROS、用假 backend 跑完整场景。
//
// 分层：
//   MissionBackend  —— 叶子对"决策运行时"的接口（下发/事件/位姿/巡逻/追击/日志/状态）。
//                      生产实现是 mission_runtime.cpp（ROS 接线），单测实现是 FakeBackend。
//   条件节点        —— 只读黑板里的派生 bool（节点每拍 syncBlackboard 写入）。
//   动作节点        —— StatefulActionNode，自持生命周期（onStart/onRunning/onHalted）。
//   PriorityFallback—— 自定义控制节点：按子节点顺序做**可抢占**的优先级回退，每拍从
//                      最高优先级重新求值；同一时刻只允许一个 RUNNING 子树。
//                      （内置 ReactiveFallback 会继续 tick 低优先级兄弟节点，多个异步
//                      动作会同时 RUNNING；内置 Fallback 又有记忆、不会重新求值优先级，
//                      所以这里需要自己的控制节点。）
//
// 与旧 FSM 的对应关系见 docs/decision/bt-scaffold.md。

#include <cstddef>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/control_node.h>

#include "decision/center_patrol.hpp"
#include "decision/types.hpp"

namespace decision
{
namespace bt
{

// ==================== 黑板 key ====================
//
// 输入（节点每拍由后端状态 syncBlackboard 写入；动作/条件只读）：
namespace keys
{
inline constexpr char kBackend[] = "mission_backend";          // MissionBackend*（生命周期由节点持有）
inline constexpr char kGameActive[] = "game_active";           // bool：比赛进行中
inline constexpr char kHpLow[] = "hp_low";                     // bool：低血迟滞结果
inline constexpr char kHpKnown[] = "hp_known";                 // bool：收到过机器人状态
inline constexpr char kHpZero[] = "hp_zero";                   // bool：血量为 0（死亡等待复活）
inline constexpr char kRevived[] = "revived";                  // bool：本局刚复活（20% 血/无敌/虚弱）
inline constexpr char kLocalizationLost[] = "localization_lost"; // bool：fast_location 报 LOST
inline constexpr char kHomeReached[] = "home_reached";         // bool：处于"家"阶段（旧 FSM 的 HOME.*）
inline constexpr char kCenterReached[] = "center_reached";     // bool：已到中心（旧 FSM 的 CENTER）
inline constexpr char kChaseReady[] = "chase_ready";           // bool：可追（enabled+zone+目标+TF）
}  // namespace keys

// 旧 FSM 的"当前段"用两个锁存 bool 表达：home_reached = HOME 段，center_reached = CENTER 段。
// 段切换在 backend 下发目标时更新，成功到达时置位。
// 生产 runtime 与单测 FakeBackend 共用这一份逻辑，避免两边语义漂移。
struct DecisionLatch
{
  bool home_reached{true};  // 初始在出生点：等价旧 FSM 初始 HOME.WAIT_HOME
  bool center_reached{false};

  void onDispatched(TargetName target)
  {
    if (target == TargetName::Center) {
      home_reached = false;
    } else if (isHomeTarget(target)) {
      center_reached = false;
    }
  }

  void onResult(TargetName target, bool success)
  {
    if (!success) {
      return;
    }
    if (isHomeTarget(target)) {
      home_reached = true;
    } else if (target == TargetName::Center) {
      center_reached = true;
    }
  }
};

// ==================== 运行时接口 ====================

// 追击判据的结论（由 backend 按旧 node 的 dispatchChaseIfAble 同一套判据算出）。
struct ChaseDecision
{
  bool ready{false};           // enabled + zone 可用 + 目标可用 + 云台 TF 可用
  bool move_needed{false};     // 目标点离车够远（否则已经站在 standoff 圈上）
  bool should_dispatch{false}; // 到了重发时机（移动死区 + 重发间隔）
  double raw_x{0.0};           // 敌人 map 坐标
  double raw_y{0.0};
  double goal_x{0.0};          // standoff + clampToZone 之后的目标点
  double goal_y{0.0};
  double enemy_distance{0.0};  // 目标点到敌人的实际距离（区域挤压时 < standoff）
  std::string reason;          // 调试原因：ready/hold/on_standoff/target_unusable/...
};

// BT 叶子看到的世界。生产实现：decision::MissionRuntime（mission_runtime.hpp）。
// 单测实现：test_bt_tree.cpp 里的 FakeBackend。
class MissionBackend
{
public:
  virtual ~MissionBackend() = default;

  virtual double nowSec() const = 0;

  // ---- 派生输入 ----
  virtual bool gameActive() const = 0;
  virtual bool hpLow() const = 0;
  virtual bool hpKnown() const = 0;
  // 死亡等待复活：发送中的目标不应继续；复活后由 revived 决定走哪条回家路线。
  virtual bool hpZero() const { return false; }
  // 本局刚复活（20% 血、无敌 30s、虚弱）：回家走隧道快线。
  virtual bool revived() const { return false; }
  virtual bool localizationLost() const = 0;
  virtual bool homeReached() const = 0;
  virtual bool centerReached() const = 0;
  virtual bool chaseReady() = 0;

  // ---- 固定目标（Home / WaitHome / WaitHp / Center / WaitCenter）----
  // 语义与旧 node 的 stepExecutorTarget 一致：已在下发/防抖期内会 no-op；
  // 返回 true 表示本拍真的发了一次请求（动作节点不需要区分，保持 RUNNING）。
  virtual bool ensureFixedTarget(TargetName target) = 0;
  // 消费执行器终态事件（只取匹配 target 的那一条）。nullopt = 本拍没有。
  virtual std::optional<bool> takeResult(TargetName target) = 0;

  // ---- 中心走位 ----
  virtual bool centerPatrolEnabled() const = 0;
  virtual double patrolGroupGapSec() const = 0;
  virtual bool patrolPoolEmpty() const = 0;
  // 窥视下一组（不推进游标）。
  virtual PatrolPath takePatrolPath() = 0;
  // 下发一组走位（穿过最后一个点不停）；成功才推进游标。
  virtual bool dispatchPatrol(const PatrolPath & path) = 0;

  // ---- 追击 ----
  virtual ChaseDecision evaluateChase() = 0;
  // 下发追击目标点；成功时 backend 内部记 reissue 时间。
  virtual bool dispatchChase(const ChaseDecision & decision) = 0;
  // 调试上报（node 侧节流发布 target_debug）。
  virtual void reportChase(const ChaseDecision & decision, bool dispatched) = 0;

  // ---- 状态/日志 ----
  virtual void setActiveState(const std::string & state) = 0;
  virtual void logInfo(const std::string & message) = 0;
  virtual void logWarn(const std::string & message) = 0;
};

// 缺 key / 类型不符时返回 fallback，不抛异常（Blackboard::get 会抛 BT::RuntimeError）。
template<typename T>
T readBlackboard(
  const BT::Blackboard::Ptr & blackboard, const std::string & key, const T & fallback)
{
  if (!blackboard) {
    return fallback;
  }
  try {
    return blackboard->get<T>(key);
  } catch (const std::exception &) {
    return fallback;
  }
}

template<typename T>
void writeBlackboard(
  const BT::Blackboard::Ptr & blackboard, const std::string & key, const T & value)
{
  if (!blackboard) {
    return;
  }
  try {
    blackboard->set<T>(key, value);
  } catch (const std::exception &) {
    // 类型冲突说明有节点写错 key；单测/日志会暴露，这里不打死整棵树。
  }
}

inline void setBackend(const BT::Blackboard::Ptr & blackboard, MissionBackend * backend)
{
  writeBlackboard(blackboard, keys::kBackend, backend);
}

inline MissionBackend * backendOf(const BT::NodeConfiguration & config)
{
  return readBlackboard<MissionBackend *>(config.blackboard, keys::kBackend, nullptr);
}

// ==================== 控制节点：可抢占优先级回退 ====================
//
// 语义（每拍）：
//   1. 从最高优先级子节点开始依次 tick；
//   2. 第一个返回 RUNNING 的子节点被选中，**停止 tick 更低优先级**；
//   3. 第一个返回 SUCCESS 的子节点结束本拍（下一拍从最高优先级重来）；
//   4. 选中的子节点与上一拍不同 ⇒ halt 上一拍的子树（抢占，触发 onHalted）。
//
// 这样游戏结束/低血/LOST 可以在任意时刻抢占正在执行的走位/追击；同时任何时刻只有
// 一个子树 RUNNING，异步动作不会互相打架（内置 ReactiveFallback 做不到这一点：
// 它会继续 tick 低优先级兄弟，并禁止多个 RUNNING 子节点）。
class PriorityFallback : public BT::ControlNode
{
public:
  PriorityFallback(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    constexpr std::size_t kNone = static_cast<std::size_t>(-1);
    const std::size_t previous = running_child_;

    std::size_t selected = kNone;
    BT::NodeStatus result = BT::NodeStatus::FAILURE;
    for (std::size_t index = 0; index < childrenCount(); ++index) {
      const BT::NodeStatus status = children()[index]->executeTick();
      if (status == BT::NodeStatus::RUNNING || status == BT::NodeStatus::SUCCESS) {
        selected = index;
        result = status;
        break;
      }
      // FAILURE：落到下一优先级。
    }

    if (previous != kNone && previous != selected) {
      haltChild(previous);  // 抢占：把上一拍的 RUNNING 子树停掉（动作 onHalted 生效）
    }
    running_child_ = result == BT::NodeStatus::RUNNING ? selected : kNone;
    return result;
  }

  void halt() override
  {
    haltChildren();
    running_child_ = static_cast<std::size_t>(-1);
  }

private:
  std::size_t running_child_{static_cast<std::size_t>(-1)};
};

// ==================== 条件节点 ====================
//
// 条件只读派生 bool —— 迟滞/时间窗/TF/zone/目标可用性都在 backend 里判完。
// 这样 XML 只表达"优先级"，条件逻辑可单测，不依赖 BT 状态。

class BoolKeyCondition : public BT::ConditionNode
{
public:
  BoolKeyCondition(
    const std::string & name, const BT::NodeConfiguration & config, const char * key)
  : BT::ConditionNode(name, config), key_(key)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return readBlackboard<bool>(config().blackboard, key_, false) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  const char * key_;
};

#define DECISION_BOOL_CONDITION(ClassName, KeyName) \
  class ClassName : public BoolKeyCondition \
  { \
public: \
  ClassName(const std::string & name, const BT::NodeConfiguration & config) \
  : BoolKeyCondition(name, config, KeyName) {} \
  }

DECISION_BOOL_CONDITION(GameActiveCondition, keys::kGameActive);
DECISION_BOOL_CONDITION(HpLowCondition, keys::kHpLow);
DECISION_BOOL_CONDITION(HpKnownCondition, keys::kHpKnown);
DECISION_BOOL_CONDITION(HpZeroCondition, keys::kHpZero);
DECISION_BOOL_CONDITION(RevivedCondition, keys::kRevived);
DECISION_BOOL_CONDITION(LocalizationLostCondition, keys::kLocalizationLost);
DECISION_BOOL_CONDITION(HomeReachedCondition, keys::kHomeReached);
DECISION_BOOL_CONDITION(CenterReachedCondition, keys::kCenterReached);
DECISION_BOOL_CONDITION(ChaseReadyCondition, keys::kChaseReady);

#undef DECISION_BOOL_CONDITION

// ==================== 动作节点基类 ====================

class MissionActionBase : public BT::StatefulActionNode
{
public:
  MissionActionBase(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

protected:
  MissionBackend * backend() const { return backendOf(config()); }

  // 进入动作时发布 /decision/state 字符串；backend 缺失返回 false（动作直接 FAILURE）。
  bool begin(const char * state)
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return false;
    }
    mission_backend->setActiveState(state);
    return true;
  }
};

// 等在家（游戏未进行）：固定目标 WaitHome，永不"完成"——优先级/条件变化时被抢占。
class WaitHomeAction : public MissionActionBase
{
public:
  WaitHomeAction(const std::string & name, const BT::NodeConfiguration & config)
  : MissionActionBase(name, config)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin("HOME.WAIT_HOME")) {
      return BT::NodeStatus::FAILURE;
    }
    backend()->ensureFixedTarget(TargetName::WaitHome);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    mission_backend->ensureFixedTarget(TargetName::WaitHome);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}
};

// 等回血（低血量）：**不再需要 wait_hp.csv**，到达家/补给区后原地保持，
// HP 恢复由 HpLow 条件失效触发抢占。
class WaitHpAction : public MissionActionBase
{
public:
  WaitHpAction(const std::string & name, const BT::NodeConfiguration & config)
  : MissionActionBase(name, config)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin("HOME.WAIT_HP")) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}
};

// 回家（游戏未进行 / 低血）：固定目标，成功才算完成；失败由 backend 的重发判据
// 在本拍/下一拍重试（动作保持 RUNNING，不自己退状态）。
//
// 三条回家的区别只在 TargetName / 状态字符串；路线数据在 waypoint_store 里分开：
//   GoHome       -> home_safe.csv   比赛未进行/结束（无战斗，走安全路线）
//   GoHomeTunnel -> home_tunnel.csv 复活后无敌/虚弱期，走隧道快线尽快到补给区
//   GoHomeSafe   -> home_safe.csv   残血回家绕远路，不在隧道里被拦截
class GoHomeViaAction : public MissionActionBase
{
public:
  GoHomeViaAction(
    const std::string & name, const BT::NodeConfiguration & config,
    TargetName target, const char * state)
  : MissionActionBase(name, config), target_(target), state_(state)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin(state_)) {
      return BT::NodeStatus::FAILURE;
    }
    backend()->ensureFixedTarget(target_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    if (const auto result = mission_backend->takeResult(target_);
      result.has_value() && *result)
    {
      return BT::NodeStatus::SUCCESS;
    }
    mission_backend->ensureFixedTarget(target_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  TargetName target_;
  const char * state_;
};

class GoHomeAction : public GoHomeViaAction
{
public:
  GoHomeAction(const std::string & name, const BT::NodeConfiguration & config)
  : GoHomeViaAction(name, config, TargetName::HomeSafe, "MOVE.GO_HOME_SAFE") {}
};

class GoHomeTunnelAction : public GoHomeViaAction
{
public:
  GoHomeTunnelAction(const std::string & name, const BT::NodeConfiguration & config)
  : GoHomeViaAction(name, config, TargetName::HomeTunnel, "MOVE.GO_HOME_TUNNEL") {}
};

class GoHomeSafeAction : public GoHomeViaAction
{
public:
  GoHomeSafeAction(const std::string & name, const BT::NodeConfiguration & config)
  : GoHomeViaAction(name, config, TargetName::HomeSafe, "MOVE.GO_HOME_SAFE") {}
};

// 回中心：固定目标 Center，Center 成功才算到达；到达后 center_reached latch 置位，
// 树进入"中心段"（追敌/走位）。
class GoCenterAction : public MissionActionBase
{
public:
  GoCenterAction(const std::string & name, const BT::NodeConfiguration & config)
  : MissionActionBase(name, config)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin("MOVE.GO_CENTER")) {
      return BT::NodeStatus::FAILURE;
    }
    backend()->ensureFixedTarget(TargetName::Center);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    if (const auto result = mission_backend->takeResult(TargetName::Center);
      result.has_value() && *result)
    {
      return BT::NodeStatus::SUCCESS;
    }
    mission_backend->ensureFixedTarget(TargetName::Center);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}
};

// LOST：不新发 map 系目标，也不取消执行器里正在跑的目标（与旧 node 的完整性门一致）。
class HoldingAction : public MissionActionBase
{
public:
  HoldingAction(const std::string & name, const BT::NodeConfiguration & config)
  : MissionActionBase(name, config)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin("HOLDING")) {
      return BT::NodeStatus::FAILURE;
    }
    backend()->logWarn("Holding：不新发 map 系目标（LOST 或死亡等待复活）");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

// 中心段追击：每拍重算 standoff + clampToZone；到重发时机就下发。
// 条件（ChaseReady）不成立时本节点不会被 tick，Fallback 自然落到走位。
class ChaseStandoffAction : public MissionActionBase
{
public:
  ChaseStandoffAction(const std::string & name, const BT::NodeConfiguration & config)
  : MissionActionBase(name, config)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin("CENTER.CHASE")) {
      return BT::NodeStatus::FAILURE;
    }
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    const ChaseDecision decision = mission_backend->evaluateChase();
    if (!decision.ready) {
      return BT::NodeStatus::FAILURE;  // 目标/TF 失效：交还走位
    }
    if (!decision.move_needed) {
      mission_backend->reportChase(decision, false);  // 已经在 standoff 圈上
      return BT::NodeStatus::RUNNING;
    }
    if (!decision.should_dispatch) {
      mission_backend->reportChase(decision, false);  // 等重发死区/间隔
      return BT::NodeStatus::RUNNING;
    }
    const bool dispatched = mission_backend->dispatchChase(decision);
    mission_backend->reportChase(decision, dispatched);
    if (!dispatched) {
      // 执行器没起来/还没就绪：交还主决策（走位照常跑），下一拍再试。
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}
};

// 中心段走位：在中心区网格（或退化到 patrol.csv）上按顺序取 path_len 个点，
// 走完一组、等 group_gap_sec，再取下一组。
//
// 生命周期自持：本节点一直 RUNNING，内部维护 in_flight/next_patrol；
// 被高优先级抢占时 onHalted 把 in_flight 清掉 —— 旧 FSM 的
// "Chase 抢占 Patrol 后 patrol_path_in_flight_ 永久为 true、走位卡死" 就是这里修的。
class PatrolCenterAction : public MissionActionBase
{
public:
  PatrolCenterAction(const std::string & name, const BT::NodeConfiguration & config)
  : MissionActionBase(name, config)
  {
  }

  BT::NodeStatus onStart() override
  {
    if (!begin("CENTER.PATROL")) {
      return BT::NodeStatus::FAILURE;
    }
    // 到达中心后先 hold 中心一个 group_gap，再开始随机点走位；
    // 这样"刚到中心"不会立刻离开，被挤开时 ensureFixedTarget(Center) 会纠偏。
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    in_flight_ = false;
    next_patrol_sec_ = mission_backend->nowSec() + mission_backend->patrolGroupGapSec();
    mission_backend->ensureFixedTarget(TargetName::Center);
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    MissionBackend * mission_backend = backend();
    if (mission_backend == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    // 本节点可能从上一拍的 RUNNING 直接继续（onStart 不会重跑）；每拍把
    // /decision/state 拉回 CENTER.PATROL，避免被 Chase 抢占后状态字符串停在 CHASE。
    mission_backend->setActiveState("CENTER.PATROL");
    // 走位被关掉：守住中心锚点（不再需要 wait_center.csv）。
    if (!mission_backend->centerPatrolEnabled()) {
      mission_backend->ensureFixedTarget(TargetName::Center);
      return BT::NodeStatus::RUNNING;
    }

    if (const auto result = mission_backend->takeResult(TargetName::Patrol); result.has_value()) {
      (void)result;
      // 走完/失败都算这一组结束：停顿从"走完这一刻"起算（与旧 node 一致）。
      in_flight_ = false;
      next_patrol_sec_ = mission_backend->nowSec() + mission_backend->patrolGroupGapSec();
      mission_backend->logInfo("中心走位：一组结束，等待组间停顿");
    }

    // 不巡逻时持续 hold 中心：被敌人挤开 >0.35m 且持续 0.8s 会重发 center。
    mission_backend->ensureFixedTarget(TargetName::Center);
    if (mission_backend->patrolPoolEmpty()) {
      return BT::NodeStatus::RUNNING;  // 没有候选点：原地不动，不猜
    }
    if (in_flight_) {
      return BT::NodeStatus::RUNNING;  // 这一组还在走
    }
    if (mission_backend->nowSec() < next_patrol_sec_) {
      return BT::NodeStatus::RUNNING;  // 等下一个随机走位时刻
    }

    const PatrolPath pick = mission_backend->takePatrolPath();
    if (pick.indices.empty()) {
      return BT::NodeStatus::RUNNING;
    }
    if (!mission_backend->dispatchPatrol(pick)) {
      // 下发失败（执行器没起/防抖）：不推进游标、不推后触发时刻，下一拍自然重试。
      return BT::NodeStatus::RUNNING;
    }
    in_flight_ = true;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // 关键：被 Chase/回家/低血抢占后不能继续认为"这一组在飞"，否则走位永久卡住。
    // 不取消执行器里的目标 —— 下一个动作下发新目标时 action 服务器会抢占它。
    in_flight_ = false;
  }

private:
  bool in_flight_{false};
  double next_patrol_sec_{-1.0e9};
};

// ==================== 黑板/注册 ====================

inline BT::Blackboard::Ptr makeDecisionBlackboard()
{
  auto blackboard = BT::Blackboard::create();
  setBackend(blackboard, nullptr);
  blackboard->set(keys::kGameActive, false);
  blackboard->set(keys::kHpLow, false);
  blackboard->set(keys::kHpKnown, false);
  blackboard->set(keys::kLocalizationLost, false);
  blackboard->set(keys::kHomeReached, true);
  blackboard->set(keys::kCenterReached, false);
  blackboard->set(keys::kChaseReady, false);
  return blackboard;
}

// 每拍 tree.tickRoot() 之前调用：把 backend 的派生输入同步进黑板。
// 生产节点与单测共用；条件节点因此永远只做一次 bool 读取。
inline void syncBlackboard(const BT::Blackboard::Ptr & blackboard, MissionBackend & backend)
{
  if (!blackboard) {
    return;
  }
  blackboard->set(keys::kGameActive, backend.gameActive());
  blackboard->set(keys::kHpLow, backend.hpLow());
  blackboard->set(keys::kHpKnown, backend.hpKnown());
  blackboard->set(keys::kHpZero, backend.hpZero());
  blackboard->set(keys::kRevived, backend.revived());
  blackboard->set(keys::kLocalizationLost, backend.localizationLost());
  blackboard->set(keys::kHomeReached, backend.homeReached());
  blackboard->set(keys::kCenterReached, backend.centerReached());
  blackboard->set(keys::kChaseReady, backend.chaseReady());
}

inline void registerDecisionNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<PriorityFallback>("PriorityFallback");

  factory.registerNodeType<GameActiveCondition>("GameActive");
  factory.registerNodeType<HpLowCondition>("HpLow");
  factory.registerNodeType<HpKnownCondition>("HpKnown");
  factory.registerNodeType<HpZeroCondition>("HpZero");
  factory.registerNodeType<RevivedCondition>("Revived");
  factory.registerNodeType<LocalizationLostCondition>("LocalizationLost");
  factory.registerNodeType<HomeReachedCondition>("HomeReached");
  factory.registerNodeType<CenterReachedCondition>("CenterReached");
  factory.registerNodeType<ChaseReadyCondition>("ChaseReady");

  factory.registerNodeType<WaitHomeAction>("WaitHome");
  factory.registerNodeType<WaitHpAction>("WaitHp");
  factory.registerNodeType<GoHomeAction>("GoHome");
  factory.registerNodeType<GoHomeTunnelAction>("GoHomeTunnel");
  factory.registerNodeType<GoHomeSafeAction>("GoHomeSafe");
  factory.registerNodeType<GoCenterAction>("GoCenter");
  factory.registerNodeType<HoldingAction>("Holding");
  factory.registerNodeType<ChaseStandoffAction>("ChaseStandoff");
  factory.registerNodeType<PatrolCenterAction>("PatrolCenter");
}

}  // namespace bt
}  // namespace decision
