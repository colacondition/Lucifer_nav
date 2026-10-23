#include "decision/mission_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <random>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace decision
{
namespace
{

constexpr double kTfLookupRetrySec = 1.0;
constexpr double kTargetDebugPeriodSec = 0.5;
constexpr double kDuplicateCheckPeriodSec = 5.0;
constexpr double kMissingCsvWarnPeriodSec = 5.0;

rclcpp::QoS latchedQos()
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  return qos;
}

}  // namespace

MissionRuntime::MissionRuntime(DecisionConfig config, rclcpp::Node & node)
: node_(node), config_(std::move(config))
{
  waypoint_store_ = std::make_unique<WaypointStore>(config_);
  waypoint_executor_ = std::make_unique<WaypointExecutorClient>(config_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  goal_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(config_.topics.goal, 10);
  executor_waypoints_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
    config_.waypoint_executor.executor_waypoints_topic, latchedQos());
  saved_waypoint_file_pub_ = node_.create_publisher<std_msgs::msg::String>(
    config_.waypoint_executor.saved_waypoint_file_topic, latchedQos());
  target_debug_pub_ = node_.create_publisher<std_msgs::msg::String>(
    "center_patrol/target_debug", rclcpp::QoS(rclcpp::KeepLast(10)));

  follow_client_ = rclcpp_action::create_client<FollowWaypoints>(
    &node_, config_.waypoint_executor.follow_action);
  through_client_ = rclcpp_action::create_client<FollowWaypoints>(
    &node_, config_.waypoint_executor.through_action);
  waypoint_executor_->setRosInterfaces(
    executor_waypoints_pub_, saved_waypoint_file_pub_, goal_pub_, follow_client_,
    through_client_);
  // 执行器终态 → 事件队列（动作在 tick 里按目标名消费）+ 段锁存。
  waypoint_executor_->setResultCallback(
    [this](TargetName target, bool success) { handleExecutorResult(target, success); });

  target_input_ = TargetInput(config_.target_chase.input);
}

void MissionRuntime::initialize()
{
  loadCenterZone();
  // 候选点必须在启动时建好并打日志：现场要能一眼看出"这条线切在哪"。
  setupPatrolCandidates();
  RCLCPP_INFO(
    node_.get_logger(),
    "决策运行时就绪：loop=%.1fHz retry=%.2fs executor_timeout=%.1fs chase=%s patrol=%s",
    config_.loop_hz, config_.retry_interval_sec, config_.executor_result_timeout_sec,
    config_.target_chase.enable ? "on" : "off", config_.center_patrol.enable ? "on" : "off");
}

double MissionRuntime::nowSec() const
{
  return node_.get_clock()->now().seconds();
}

// ==================== 输入 ====================

void MissionRuntime::updateRobotStatus(
  const decision_interfaces::msg::RobotStatus & msg, double now_sec)
{
  context_.setRobotStatus(msg, now_sec);
  // 低血迟滞（120/400）：与旧 FSM 的 hp_recovery_active_ 同一判据。
  const int current_hp = static_cast<int>(msg.current_hp);
  if (current_hp <= 0) {
    dead_seen_ = true;
  } else if (dead_seen_) {
    // 血量从 0 回到 >0：本局刚复活（规则：20% 血、无敌 30s、虚弱）。
    dead_seen_ = false;
    revived_ = true;
  }
  if (hp_low_) {
    if (current_hp >= config_.hp_recovery.high_threshold) {
      hp_low_ = false;
    }
  } else if (current_hp < config_.hp_recovery.low_threshold) {
    hp_low_ = true;
  }
}

void MissionRuntime::updateGameStatus(
  const decision_interfaces::msg::GameStatus & msg, double now_sec)
{
  context_.setGameStatus(msg, now_sec);
  // 新一局/结算：清掉上一局的死亡/复活记忆，避免下一局残血误走隧道。
  if (!gameActive()) {
    dead_seen_ = false;
    revived_ = false;
  }
}

void MissionRuntime::updateLocalizationStatus(const std::string & payload)
{
  context_.setLocalizationStatus(payload);
}

void MissionRuntime::updateTargetPayload(const std::string & payload, double now_sec)
{
  // 上游消息没有 header，只能用到达时刻当时间戳。
  target_input_.update(parseTargetPayload(payload), now_sec);
  if (!target_seen_logged_) {
    target_seen_logged_ = true;
    RCLCPP_INFO(
      node_.get_logger(), "依赖就绪：自瞄开始发数据（%s）", config_.target_chase.topic.c_str());
  }
  ++target_msg_count_;
}

// ==================== 派生输入 ====================

bool MissionRuntime::gameActive() const
{
  const auto status = context_.gameStatus();
  return status.has_value() &&
         static_cast<int>(status->game_progress) == config_.game.progress &&
         static_cast<int>(status->stage_remain_time) >= config_.game.lower_remain_time &&
         static_cast<int>(status->stage_remain_time) <= config_.game.higher_remain_time;
}

bool MissionRuntime::hpLow() const
{
  return hp_low_;
}

bool MissionRuntime::hpKnown() const
{
  return context_.effectiveCurrentHp().has_value();
}

bool MissionRuntime::hpZero() const
{
  const auto hp = context_.effectiveCurrentHp();
  return hp.has_value() && *hp <= 0;
}

bool MissionRuntime::revived() const
{
  return revived_;
}

bool MissionRuntime::localizationLost() const
{
  return config_.integrity_gate.enable && context_.localizationLost();
}

bool MissionRuntime::homeReached() const
{
  return latch_.home_reached;
}

bool MissionRuntime::centerReached() const
{
  return latch_.center_reached;
}

bool MissionRuntime::chaseReady()
{
  return evaluateChase().ready;
}

// ==================== 位姿 ====================

void MissionRuntime::refreshPoseCaches()
{
  if (poses_refreshed_) {
    return;
  }
  poses_refreshed_ = true;
  const double now_sec = nowSec();
  robot_pose_cache_ = poseInMap(
    config_.maintain_goal.robot_base_frame, now_sec, next_tf_lookup_attempt_sec_,
    "car pose for maintain-goal", car_pose_ready_logged_);
  lidar_pose_cache_ = poseInMap(
    config_.center_zone.lidar_frame, now_sec, next_lidar_tf_attempt_sec_,
    "gimbal pose for target chase", lidar_pose_ready_logged_);
}

std::optional<Pose> MissionRuntime::robotPoseInMap()
{
  refreshPoseCaches();
  return robot_pose_cache_;
}

std::optional<Pose> MissionRuntime::lidarPoseInMap()
{
  refreshPoseCaches();
  return lidar_pose_cache_;
}

std::optional<Pose> MissionRuntime::poseInMap(
  const std::string & frame, double now_sec, double & next_attempt, const char * what,
  bool & ready_logged)
{
  if (now_sec < next_attempt) {
    return std::nullopt;
  }
  try {
    const auto transform =
      tf_buffer_->lookupTransform(config_.goal_frame_id, frame, tf2::TimePointZero);
    Pose pose;
    pose.x = transform.transform.translation.x;
    pose.y = transform.transform.translation.y;
    pose.z = transform.transform.translation.z;
    pose.qx = transform.transform.rotation.x;
    pose.qy = transform.transform.rotation.y;
    pose.qz = transform.transform.rotation.z;
    pose.qw = transform.transform.rotation.w;
    next_attempt = -1.0e9;
    if (!ready_logged) {
      ready_logged = true;
      RCLCPP_INFO(
        node_.get_logger(), "依赖就绪：%s（%s -> %s）", what,
        config_.goal_frame_id.c_str(), frame.c_str());
    }
    return pose;
  } catch (const tf2::TransformException & ex) {
    next_attempt = now_sec + kTfLookupRetrySec;
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "Cannot look up %s: transform %s <- %s unavailable: %s", what,
      config_.goal_frame_id.c_str(), frame.c_str(), ex.what());
    return std::nullopt;
  }
}

// ==================== 下发 ====================

std::optional<std::vector<Pose>> MissionRuntime::loadPreparedWaypoints(
  TargetName target, TargetMode mode)
{
  if (mode == TargetMode::ExecutorFollow) {
    return waypoint_store_->prepareFollowWaypoints(target, std::nullopt);
  }
  return waypoint_store_->loadWaypoints(target);
}

bool MissionRuntime::ensureFixedTarget(TargetName target)
{
  constexpr TargetMode kMode = TargetMode::ExecutorFollow;
  const auto & state = waypoint_executor_->state();
  if (state.running_target == target && state.running_mode == kMode) {
    return false;  // 已在跑，不用重发
  }

  std::optional<Pose> robot_pose;
  std::optional<Pose> target_pose;
  if (state.active_target == target &&
    state.result_status == ExecutorResultStatus::Succeeded)
  {
    robot_pose = robotPoseInMap();
    target_pose = waypoint_store_->anchorPose(target);
  }

  const double now_sec = nowSec();
  if (!waypoint_executor_->shouldRequestTarget(
      target, kMode, robot_pose, target_pose, now_sec))
  {
    return false;
  }

  const auto waypoints = loadPreparedWaypoints(target, kMode);
  if (!waypoints.has_value() || waypoints->empty()) {
    warnMissingCsv(target, now_sec);
    return false;
  }

  const bool immediate_preempt = waypoint_executor_->shouldPreemptImmediately(target);
  const bool request_sent = waypoint_executor_->stepExecutorTarget(
    target, kMode, waypoint_store_->waypointFile(target), *waypoints, immediate_preempt,
    now_sec);
  if (request_sent) {
    latch_.onDispatched(target);
    RCLCPP_INFO(
      node_.get_logger(), "Requested executor target [%s] in mode [follow]",
      decision::toString(target).c_str());
  }
  return request_sent;
}

bool MissionRuntime::dispatchPath(const std::vector<Pose> & path, TargetName target)
{
  if (path.empty()) {
    return false;
  }
  const double now_sec = nowSec();
  const bool sent = waypoint_executor_->stepExecutorTarget(
    target, TargetMode::ExecutorFollow, waypoint_store_->waypointFile(target), path,
    /*immediate_preempt=*/false, now_sec,
    // 走位/追击都是"连续移动"，穿过最后一个点不停，否则每段末尾都会顿一下。
    /*stop_at_final=*/false);
  if (sent) {
    latch_.onDispatched(target);
  }
  return sent;
}

void MissionRuntime::warnMissingCsv(TargetName target, double now_sec)
{
  auto & last_warn_sec = last_missing_csv_warn_sec_[target];
  if (now_sec - last_warn_sec < kMissingCsvWarnPeriodSec) {
    return;
  }
  last_warn_sec = now_sec;
  RCLCPP_WARN(
    node_.get_logger(),
    "Target [%s] has no CSV. Fixed-point fallback has been removed.",
    decision::toString(target).c_str());
}

// ==================== 巡逻 ====================

bool MissionRuntime::centerPatrolEnabled() const
{
  return config_.center_patrol.enable;
}

double MissionRuntime::patrolGroupGapSec() const
{
  return config_.center_patrol.group_gap_sec;
}

bool MissionRuntime::patrolPoolEmpty() const
{
  return active_pool_ == nullptr || active_pool_->empty();
}

PatrolPath MissionRuntime::takePatrolPath()
{
  if (patrolPoolEmpty()) {
    return PatrolPath{};
  }
  // 不再读 patrol.csv：从中心区内生成的网格点里随机取 path_len 个，
  // 每次走位落点不同；点全部来自 range.csv 的内缩区域。
  std::vector<std::size_t> indices(active_pool_->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::mt19937 rng(std::random_device{}());
  std::shuffle(indices.begin(), indices.end(), rng);
  const std::size_t count = std::min<std::size_t>(
    static_cast<std::size_t>(std::max(1, config_.center_patrol.path_len)),
    indices.size());
  indices.resize(count);
  PatrolPath path;
  path.indices = std::move(indices);
  return path;
}

bool MissionRuntime::dispatchPatrol(const PatrolPath & path)
{
  if (patrolPoolEmpty() || path.indices.empty()) {
    return false;
  }
  std::vector<Pose> waypoints;
  waypoints.reserve(path.indices.size());
  for (const std::size_t index : path.indices) {
    if (index < active_pool_->size()) {
      waypoints.push_back((*active_pool_)[index]);
    }
  }
  if (waypoints.empty() || !dispatchPath(waypoints, TargetName::Patrol)) {
    return false;
  }
  patrol_cursor_ = path.cursor;
  RCLCPP_INFO(
    node_.get_logger(), "Center patrol: %zu waypoints from the zone grid, next cursor %zu",
    waypoints.size(), patrol_cursor_);
  return true;
}

const std::vector<Pose> * MissionRuntime::tacticalPool(double now_sec)
{
  loadTacticalPoints();
  if (!tactical_points_.empty()) {
    return &tactical_points_;
  }
  if (now_sec - last_tactical_warn_sec_ >= kMissingCsvWarnPeriodSec) {
    last_tactical_warn_sec_ = now_sec;
    RCLCPP_WARN(
      node_.get_logger(),
      "No center waypoint pool (patrol CSV missing); CENTER stays put instead of walking.");
  }
  return nullptr;
}

void MissionRuntime::loadTacticalPoints()
{
  if (tactical_pool_loaded_) {
    return;
  }
  tactical_pool_loaded_ = true;
  auto points = waypoint_store_->loadWaypoints(TargetName::Patrol);
  if (points.has_value() && !points->empty()) {
    tactical_points_ = std::move(*points);
  }
}

// 启动时把区域角点读进来、算好安全区，并顺手校验一遍巡逻点。只做一次。
void MissionRuntime::loadCenterZone()
{
  if (!config_.center_zone.enable) {
    RCLCPP_INFO(node_.get_logger(), "中心区域校验已关闭（center_zone.enable=false）");
    return;
  }
  const auto corners = waypoint_store_->loadWaypoints(TargetName::Zone);
  if (!corners.has_value() || corners->empty()) {
    const auto path = waypoint_store_->waypointFile(TargetName::Zone);
    if (path.empty()) {
      RCLCPP_WARN(
        node_.get_logger(),
        "中心区域角点没配（targets.zone_waypoint_file 是空的）。追击不可用、走位不受影响。\n"
        "  正常启动请用 `ros2 launch decision decision.launch.py`（或 bringup 的 "
        "real.launch.py）；直接 ros2 run 起节点不会配这些路径。");
    } else {
      RCLCPP_WARN(
        node_.get_logger(),
        "读不到中心区域角点：targets.zone_waypoint_file='%s' 打不开。追击不可用、走位不受影响。",
        path.c_str());
    }
    return;
  }
  center_zone_ = makeCenterZone(*corners, config_.center_zone.margin_m);
  if (!center_zone_.usable) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "中心区域角点不可用（读了 %zu 个点，margin=%.2f m）。要 4 个不重合、不共线、凸的"
      "角点；margin 大到把区域缩没也会判不可用。追击不会启用。",
      corners->size(), config_.center_zone.margin_m);
    return;
  }

  double x0 = center_zone_.safe.front().x, x1 = x0;
  double y0 = center_zone_.safe.front().y, y1 = y0;
  for (const auto & v : center_zone_.safe) {
    x0 = std::min(x0, v.x);
    x1 = std::max(x1, v.x);
    y0 = std::min(y0, v.y);
    y1 = std::max(y1, v.y);
  }
  RCLCPP_INFO(
    node_.get_logger(),
    "中心区域就绪：角点 %zu 个，margin=%.2f m，安全区包围盒 %.2f x %.2f m",
    corners->size(), config_.center_zone.margin_m, x1 - x0, y1 - y0);

  const auto patrol = waypoint_store_->loadWaypoints(TargetName::Patrol);
  if (!patrol.has_value() || patrol->empty()) {
    return;
  }
  int far_outside = 0;
  std::string names;
  for (std::size_t i = 0; i < patrol->size(); ++i) {
    if (zoneContains(center_zone_, (*patrol)[i])) {
      continue;
    }
    const auto nearest = clampToZone(center_zone_, (*patrol)[i]);
    const double overshoot = std::hypot(nearest.x - (*patrol)[i].x, nearest.y - (*patrol)[i].y);
    if (overshoot > config_.center_zone.warn_outside_m) {
      ++far_outside;
      names += " #" + std::to_string(i) + "(越界 " + std::to_string(overshoot) + "m)";
    }
  }
  if (far_outside > 0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "有 %d 个巡逻点跑到中心区域框外超过 %.2f m：%s\n"
      "  框是往内缩过的，贴边甚至微出几厘米都正常；但超出这么多通常是录点或 map 有问题。",
      far_outside, config_.center_zone.warn_outside_m, names.c_str());
  }
}

void MissionRuntime::setupPatrolCandidates()
{
  if (patrol_candidates_tried_) {
    return;
  }
  patrol_candidates_tried_ = true;

  if (center_zone_.usable) {
    grid_points_ = generateZoneGrid(center_zone_, config_.center_patrol.grid_spacing_m);
  }
  if (!grid_points_.empty()) {
    active_pool_ = &grid_points_;
    RCLCPP_INFO(
      node_.get_logger(), "走位：区域网格 %zu 个点（格距 %.2f m）",
      grid_points_.size(), config_.center_patrol.grid_spacing_m);
    return;
  }

  loadTacticalPoints();
  if (!tactical_points_.empty()) {
    active_pool_ = &tactical_points_;
    RCLCPP_WARN(
      node_.get_logger(), "中心区域不可用；走位退回 patrol.csv 的 %zu 个点（按文件顺序）",
      tactical_points_.size());
    return;
  }
  RCLCPP_WARN(
    node_.get_logger(),
    "既没有中心区域也没有 patrol.csv（patrol_waypoint_file='%s'）：CENTER 段会原地不动。\n"
    "  用 decision.launch.py 或 bringup 的 real.launch.py 启动才会配上这些路径。",
    waypoint_store_->waypointFile(TargetName::Patrol).c_str());
}

// ==================== 追击 ====================

bt::ChaseDecision MissionRuntime::evaluateChase()
{
  if (chase_decision_valid_) {
    return chase_decision_;
  }
  chase_decision_valid_ = true;
  chase_dispatched_ = false;
  bt::ChaseDecision decision;
  decision.reason = "chase_disabled";

  // 只有到了中心段才允许追敌（旧 FSM：CENTER 段动作）。
  if (!latch_.center_reached) {
    decision.reason = "not_in_center";
    chase_decision_ = decision;
    return chase_decision_;
  }
  if (!config_.target_chase.enable) {
    chase_decision_ = decision;
    return chase_decision_;
  }
  if (!center_zone_.usable) {
    decision.reason = "zone_unusable";
    chase_decision_ = decision;
    return chase_decision_;
  }
  // 先看目标可不可用，**再**查 TF：没收到目标时一次 TF 都不查。
  const auto & target = target_input_.state();
  if (!target.usable) {
    decision.reason = "target_unusable";
    chase_decision_ = decision;
    return chase_decision_;
  }
  const auto lidar = lidarPoseInMap();
  if (!lidar.has_value()) {
    decision.reason = "no_lidar_tf";
    chase_decision_ = decision;
    return chase_decision_;
  }

  const auto raw = targetToMap(*lidar, target.x, target.y);
  const auto robot = robotPoseInMap();
  const auto ring = robot.has_value() ?
    standoffGoal(raw, *robot, config_.target_chase.standoff_m) : raw;
  // 区域优先于保持距离：出区就丢占领权，那个代价比"离得近一点"大得多。
  const auto goal = clampToZone(center_zone_, ring);

  decision.ready = true;
  decision.raw_x = raw.x;
  decision.raw_y = raw.y;
  decision.goal_x = goal.x;
  decision.goal_y = goal.y;
  decision.enemy_distance = std::hypot(goal.x - raw.x, goal.y - raw.y);
  decision.reason = "ready";
  if (robot.has_value()) {
    const double move = std::hypot(goal.x - robot->x, goal.y - robot->y);
    if (move < config_.target_chase.min_goal_move_m) {
      decision.move_needed = false;
      decision.reason = "on_standoff";  // 已经站在圈上（或区域把它挤在圈附近）
      chase_decision_ = decision;
      return chase_decision_;
    }
  }
  decision.move_needed = true;
  const double now_sec = nowSec();
  decision.should_dispatch = target_input_.shouldReissue(goal.x, goal.y, now_sec);
  decision.reason = decision.should_dispatch ? "ready" : "hold";
  chase_decision_ = decision;
  return chase_decision_;
}

bool MissionRuntime::dispatchChase(const bt::ChaseDecision & decision)
{
  const std::vector<Pose> path{Pose{decision.goal_x, decision.goal_y}};
  if (!dispatchPath(path, TargetName::Chase)) {
    return false;
  }
  target_input_.markReissued(decision.goal_x, decision.goal_y, nowSec());
  RCLCPP_INFO(
    node_.get_logger(), "Target chase: gimbal=(%.2f, %.2f) -> map=(%.2f, %.2f)%s",
    target_input_.state().x, target_input_.state().y, decision.goal_x, decision.goal_y,
    (std::hypot(decision.raw_x - decision.goal_x, decision.raw_y - decision.goal_y) > 1.0e-6) ?
    " (clamped into zone)" : "");
  return true;
}

void MissionRuntime::reportChase(const bt::ChaseDecision & decision, bool dispatched)
{
  setChaseOutcome(decision, dispatched);
}

void MissionRuntime::setChaseOutcome(const bt::ChaseDecision & decision, bool dispatched)
{
  chase_decision_ = decision;
  chase_decision_valid_ = true;
  chase_dispatched_ = dispatched;
}

// ==================== 结果/事件 ====================

void MissionRuntime::handleExecutorResult(TargetName target, bool success)
{
  latch_.onResult(target, success);
  if (success && isHomeTarget(target)) {
    // 已到补给区：虚弱解除，后续再低血按"残血"处理，走安全路线。
    revived_ = false;
  }
  pending_events_.push_back(PendingEvent{target, success});
  if (success) {
    RCLCPP_INFO(
      node_.get_logger(), "Executor target [%s] completed", decision::toString(target).c_str());
  } else {
    RCLCPP_WARN(
      node_.get_logger(), "Executor target [%s] aborted", decision::toString(target).c_str());
  }
}

std::optional<bool> MissionRuntime::takeResult(TargetName target)
{
  for (auto it = pending_events_.begin(); it != pending_events_.end(); ++it) {
    if (it->target != target) {
      continue;
    }
    const bool success = it->success;
    pending_events_.erase(it);
    return success;
  }
  return std::nullopt;
}

// ==================== 每拍 ====================

void MissionRuntime::beginTick()
{
  poses_refreshed_ = false;
  chase_decision_valid_ = false;
  // 超时守卫先于树 tick：合成 Aborted 触发的 result 回调产生的事件要在本拍可见。
  if (waypoint_executor_->checkGoalTimeout(nowSec())) {
    RCLCPP_INFO(
      node_.get_logger(), "Executor goal timeout guard fired; target will be re-requested");
  }
}

void MissionRuntime::endTick()
{
  // 没被任何动作消费的事件（Chase/Wait 类目标的完成)在本拍结束丢弃：
  // 它们只影响调试，不应污染下一拍。
  pending_events_.clear();
  if (!target_debug_pub_) {
    return;
  }
  const double now_sec = nowSec();
  if (now_sec < next_target_debug_sec_) {
    return;
  }
  next_target_debug_sec_ = now_sec + kTargetDebugPeriodSec;
  std_msgs::msg::String message;
  message.data = buildTargetDebugString();
  target_debug_pub_->publish(message);
}

std::string MissionRuntime::buildTargetDebugString() const
{
  const auto & target = target_input_.state();
  std::ostringstream os;
  os << std::fixed << std::setprecision(2);
  os << "usable=" << (target.usable ? 1 : 0) << " id=" << target.published_id << "("
     << publishedIdName(target.published_id) << ")"
     << " gimbal=(" << target.x << "," << target.y << ")"
     << " range=" << target.range_m << " msgs=" << target_msg_count_;
  if (chase_decision_.ready) {
    os << " map_raw=(" << chase_decision_.raw_x << "," << chase_decision_.raw_y << ")"
       << " goal=(" << chase_decision_.goal_x << "," << chase_decision_.goal_y << ")"
       << " enemy_dist=" << chase_decision_.enemy_distance;
  }
  os << " zone=" << (center_zone_.usable ? "ok" : "unusable")
     << " dispatched=" << (chase_dispatched_ ? 1 : 0)
     << " reason=" << chase_decision_.reason;
  return os.str();
}

// ==================== 状态/日志 ====================

void MissionRuntime::setActiveState(const std::string & state)
{
  active_state_ = state;
}

void MissionRuntime::logInfo(const std::string & message)
{
  RCLCPP_INFO(node_.get_logger(), "%s", message.c_str());
}

void MissionRuntime::logWarn(const std::string & message)
{
  RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
}

// ==================== 自检 ====================

void MissionRuntime::checkDuplicateNodes()
{
  const double now_sec = nowSec();
  if (now_sec < next_dup_check_sec_) {
    return;
  }
  next_dup_check_sec_ = now_sec + kDuplicateCheckPeriodSec;

  // 同名决策节点：会各自往执行器下发目标、互相抢占。
  const std::string self = node_.get_fully_qualified_name();
  std::size_t count = 0;
  for (const auto & name : node_.get_node_names()) {
    if (name == self) {
      ++count;
    }
  }
  if (count > 1) {
    if (!dup_node_reported_) {
      dup_node_reported_ = true;
      RCLCPP_ERROR(
        node_.get_logger(),
        "图里有 %zu 个同名节点 [%s]：两份决策会互相抢占，并刷 "
        "'unknown goal/result response'。请只保留一份",
        count, self.c_str());
    }
  } else {
    dup_node_reported_ = false;
  }

  // 两个同名 action server 会让客户端收到两份响应，还会抢底盘。
  const std::string & executor_name = config_.waypoint_executor.node_name;
  if (!executor_name.empty()) {
    std::size_t executors = 0;
    for (const auto & name : node_.get_node_names()) {
      if (name == "/" + executor_name) {
        ++executors;
      }
    }
    if (executors > 1) {
      if (!dup_executor_reported_) {
        dup_executor_reported_ = true;
        RCLCPP_ERROR(
          node_.get_logger(),
          "图里有 %zu 个执行器 [%s]：会收到重复响应、两个执行器还会抢底盘。"
          "别把 decision launch 和 sim/real launch 一起起（执行器由导航栈提供）。",
          executors, executor_name.c_str());
      }
    } else {
      dup_executor_reported_ = false;
    }
  }
}

}  // namespace decision
