#include <algorithm>
#include <chrono>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <decision_interfaces/action/follow_waypoints.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "decision/decision_config.hpp"
#include "decision/decision_context.hpp"
#include "decision/decision_state_machine.hpp"
#include "decision/types.hpp"
#include "decision/waypoint_executor_client.hpp"
#include "decision/waypoint_store.hpp"
#include "decision_interfaces/msg/game_status.hpp"
#include "decision_interfaces/msg/robot_status.hpp"

namespace
{

constexpr double kTfLookupRetrySec = 1.0;

rclcpp::QoS latchedQos()
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  return qos;
}

}  // namespace

class BtActionReplacementNode : public rclcpp::Node
{
public:
  BtActionReplacementNode()
  : Node("bt_action_replacement")
  {
    decision::declareDecisionParameters(*this);
    config_ = decision::loadDecisionConfig(*this);

    context_ = decision::DecisionContext(config_.combat);
    state_machine_ = std::make_unique<decision::DecisionStateMachine>(
      config_.hp_recovery, config_.targets, config_.combat);
    waypoint_store_ = std::make_unique<decision::WaypointStore>(config_);
    waypoint_executor_ = std::make_unique<decision::WaypointExecutorClient>(config_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(config_.topics.goal, 10);
    decision_state_pub_ = create_publisher<std_msgs::msg::String>(
      config_.topics.decision_state, decision::decisionStateQos());
    decision_state_pub_->publish(decision::decisionStateMessage(state_machine_->state()));
    saved_waypoint_file_pub_ = create_publisher<std_msgs::msg::String>(
      config_.waypoint_executor.saved_waypoint_file_topic, latchedQos());
    executor_waypoints_pub_ = create_publisher<nav_msgs::msg::Path>(
      config_.waypoint_executor.executor_waypoints_topic, latchedQos());
    follow_client_ = rclcpp_action::create_client<decision_interfaces::action::FollowWaypoints>(
      this, config_.waypoint_executor.follow_action);
    through_client_ = rclcpp_action::create_client<decision_interfaces::action::FollowWaypoints>(
      this, config_.waypoint_executor.through_action);
    waypoint_executor_->setRosInterfaces(
      executor_waypoints_pub_, saved_waypoint_file_pub_, goal_pub_, follow_client_, through_client_);
    // action 结果 → 状态机事件（替代旧的「状态话题 + 代次关联」）。
    waypoint_executor_->setResultCallback(
      [this](decision::TargetName target, bool success) {
        pending_executor_event_ = decision::ExecutorEvent{
          target,
          success ? decision::ExecutorEventType::Succeeded : decision::ExecutorEventType::Aborted};
        if (success) {
          RCLCPP_INFO(
            get_logger(), "Executor target [%s] completed",
            decision::toString(target).c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "Executor target [%s] aborted",
            decision::toString(target).c_str());
        }
      });

    robot_status_sub_ = create_subscription<decision_interfaces::msg::RobotStatus>(
      config_.topics.robot_status, 10,
      [this](const decision_interfaces::msg::RobotStatus & msg) {
        context_.setRobotStatus(msg, nowSec());
      });
    game_status_sub_ = create_subscription<decision_interfaces::msg::GameStatus>(
      config_.topics.game_status, 10,
      [this](const decision_interfaces::msg::GameStatus & msg) {
        context_.setGameStatus(msg, nowSec());
      });
    if (config_.integrity_gate.enable) {
      localization_status_sub_ = create_subscription<std_msgs::msg::String>(
        config_.topics.localization_status, latchedQos(),
        [this](const std_msgs::msg::String & msg) {
          context_.setLocalizationStatus(msg.data);
        });
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / config_.loop_hz),
      [this]() {
        onTimer();
      });

    RCLCPP_INFO(get_logger(), "bt_action_replacement C++ node started");
  }

private:
  double nowSec()
  {
    return get_clock()->now().seconds();
  }

  bool gameActive() const
  {
    const auto status = context_.gameStatus();
    return status.has_value() &&
      static_cast<int>(status->game_progress) == config_.game.progress &&
      static_cast<int>(status->stage_remain_time) >= config_.game.lower_remain_time &&
      static_cast<int>(status->stage_remain_time) <= config_.game.higher_remain_time;
  }

  void onTimer()
  {
    const double now_sec = nowSec();
    decision::DecisionInputs inputs;
    inputs.game_active = gameActive();
    inputs.current_hp = context_.effectiveCurrentHp();
    inputs.executor_event = std::exchange(pending_executor_event_, std::nullopt);
    inputs.combat = context_.combatAssessment();
    // 诊断用：周期性打印战斗态势，便于在仿真里核对开火/挨打/交战状态判定。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Combat: firing=%d hit=%d state=%s",
      inputs.combat.firing ? 1 : 0, inputs.combat.hit ? 1 : 0,
      decision::toString(inputs.combat.state).c_str());
    const auto result = state_machine_->tick(inputs, now_sec);

    if (result.transitioned) {
      RCLCPP_INFO(
        get_logger(), "Decision state %s -> %s (%s)",
        decision::toString(result.previous_state).c_str(),
        decision::toString(result.state).c_str(), result.reason.c_str());
      decision_state_pub_->publish(decision::decisionStateMessage(result.state));
    }

    // 丢定位时仍推进状态机（血量/比赛事件不能丢），但不再发 map 系目标。
    // ENGAGE 会按 currentRobotPose 钉点；LOST 时 TF 是过期的上一拍 map→odom。
    // 从没收到过 localization_status 不拦：mapping_nav / 无定位仿真要能下发。
    if (config_.integrity_gate.enable && context_.localizationLost()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "定位丢失（%s）；不下发 map 系目标，等重定位。",
        context_.localizationStatusPayload().c_str());
      return;
    }

    // CENTER 段下发策略：Hold 守中心锚点，Reposition 从战术点池挑单点换位，
    // Engage 站定不下发。其余状态走原有固定映射。
    if (std::holds_alternative<decision::CenterState>(result.state)) {
      handleCenterTactical(result, now_sec);
    } else {
      tactical_path_.clear();
      tactical_path_pending_ = false;
      tactical_stop_sent_ = false;
      stepExecutorTarget(result.target, decision::TargetMode::ExecutorFollow, now_sec);
    }
  }

  // 依据 CENTER 子状态决定下发：
  //   HOLD       守中心锚点维持占领，漂移由 maintain 逻辑纠偏回 WaitCenter
  //   REPOSITION 被压制，从战术点池随机挑 N 个不重复点连成路径换位脱离；
  //              中途判定正面交火会由 ENGAGE 分支下发"停当前位"打断路径
  //   ENGAGE     站定当前位专注输出：若换位路径还在走，立即停住
  void handleCenterTactical(const decision::StateMachineResult & result, double now_sec)
  {
    const auto substate = std::get<decision::CenterState>(result.state).substate;

    if (substate == decision::CenterSubstate::Engage) {
      // 正面交火：站定输出。若换位路径仍在执行，下发"当前位"把它打断。
      if (!tactical_stop_sent_ &&
        waypoint_executor_->state().running_target == decision::TargetName::WaitCenter &&
        waypoint_executor_->state().running_mode == decision::TargetMode::ExecutorFollow)
      {
        if (dispatchStopAtCurrentPose(now_sec)) {
          tactical_stop_sent_ = true;
        }
      }
      return;
    }

    if (substate == decision::CenterSubstate::Hold) {
      // 呆在中心：守 WaitCenter 锚点，漂移时自动纠偏。
      tactical_stop_sent_ = false;
      stepExecutorTarget(decision::TargetName::WaitCenter, decision::TargetMode::ExecutorFollow, now_sec);
      return;
    }

    // Reposition：换位事件重新选路径，之后持续重试下发直到 executor 接受。
    const auto * pool = tacticalPool(now_sec);
    if (pool == nullptr) {
      // 战术点池不可用（缺 patrol.csv）：原地不动，不跑任何巡逻兜底。
      return;
    }
    if (result.reason == "suppressed_reposition") {
      pickRandomPath(*pool);
      tactical_path_pending_ = true;
      tactical_stop_sent_ = false;
    }
    // 一次性下发可能被 stepExecutorTarget 的 retry 间隔 / 服务发现 / in-flight
    // 等瞬时门控吞掉，导致"状态切到 REPOSITION 但机器人不动"。
    // 改为持续重试，直到路径真正发出。
    if (tactical_path_pending_ && dispatchTacticalPath(*pool, now_sec)) {
      tactical_path_pending_ = false;
    }
  }

  // 惰性加载战术点池：直接复用 patrol.csv 的多个点，无需新建 CSV。
  const std::vector<decision::Pose> * tacticalPool(double now_sec)
  {
    if (!tactical_pool_loaded_) {
      tactical_pool_loaded_ = true;
      auto points = waypoint_store_->loadWaypoints(decision::TargetName::Patrol);
      if (points.has_value() && !points->empty()) {
        tactical_points_ = std::move(*points);
      }
    }
    if (!tactical_points_.empty()) {
      return &tactical_points_;
    }
    if (now_sec - last_tactical_warn_sec_ >= 5.0) {
      last_tactical_warn_sec_ = now_sec;
      RCLCPP_WARN(
        get_logger(),
        "No tactical waypoint pool (patrol CSV missing); CENTER holds in place (no patrol fallback).");
    }
    return nullptr;
  }

  // 从战术点池随机挑 N 个不重复的点，形成换位路径（Fisher-Yates 部分打乱）。
  void pickRandomPath(const std::vector<decision::Pose> & pool)
  {
    tactical_path_.clear();
    const std::size_t pool_size = pool.size();
    const std::size_t len = std::min(
      static_cast<std::size_t>(config_.combat.reposition_path_len), pool_size);
    if (pool_size == 0 || len == 0) {
      return;
    }
    std::vector<std::size_t> indices(pool_size);
    for (std::size_t i = 0; i < pool_size; ++i) {
      indices[i] = i;
    }
    for (std::size_t i = 0; i < len; ++i) {
      const std::size_t j = randomIndex(pool_size - i);
      std::swap(indices[i], indices[i + j]);
    }
    tactical_path_.assign(indices.begin(), indices.begin() + len);
  }

  std::size_t randomIndex(std::size_t pool_size)
  {
    std::uniform_int_distribution<std::size_t> dist(0, pool_size - 1);
    return dist(rng_);
  }

  // 把换位路径（多个点）作为目标下发，经原有 follow 链路依次走完。
  // 非抢占下发：stepExecutorTarget 会在调服务前重发路径，保证 override 生效。
  // 返回是否真正发出请求（供调用方决定是否继续重试）。
  bool dispatchTacticalPath(const std::vector<decision::Pose> & pool, double now_sec)
  {
    std::vector<decision::Pose> path;
    for (const std::size_t idx : tactical_path_) {
      if (idx < pool.size()) {
        path.push_back(pool[idx]);
      }
    }
    if (path.empty()) {
      return false;
    }
    const bool sent = waypoint_executor_->stepExecutorTarget(
      decision::TargetName::WaitCenter, decision::TargetMode::ExecutorFollow,
      waypoint_store_->waypointFile(decision::TargetName::Patrol), path,
      /*immediate_preempt=*/false, now_sec);
    if (sent) {
      RCLCPP_INFO(
        get_logger(), "Reposition path: %zu waypoints, first (%.2f, %.2f)",
        path.size(), path.front().x, path.front().y);
    }
    return sent;
  }

  // 正面交火站定：下发"当前位"单点目标，把正在执行的换位路径打断。
  bool dispatchStopAtCurrentPose(double now_sec)
  {
    const auto pose = currentRobotPose(now_sec);
    if (!pose.has_value()) {
      return false;
    }
    const std::vector<decision::Pose> single{*pose};
    const bool sent = waypoint_executor_->stepExecutorTarget(
      decision::TargetName::WaitCenter, decision::TargetMode::ExecutorFollow,
      waypoint_store_->waypointFile(decision::TargetName::Patrol), single,
      /*immediate_preempt=*/false, now_sec);
    if (sent) {
      RCLCPP_INFO(get_logger(), "Engage: hold current position (%.2f, %.2f)", pose->x, pose->y);
    }
    return sent;
  }

  void stepExecutorTarget(
    decision::TargetName target,
    decision::TargetMode mode,
    double now_sec)
  {
    const auto state = waypoint_executor_->state();
    if (state.running_target == target && state.running_mode == mode) {
      return;
    }

    std::optional<decision::Pose> robot_pose;
    std::optional<decision::Pose> target_pose;
    if (state.active_target == target &&
      state.result_status == decision::ExecutorResultStatus::Succeeded)
    {
      robot_pose = currentRobotPose(now_sec);
      target_pose = waypoint_store_->anchorPose(target);
    }

    if (!waypoint_executor_->shouldRequestTarget(
        target, mode, robot_pose, target_pose, now_sec))
    {
      return;
    }

    const auto waypoints = loadPreparedWaypoints(target, mode);
    if (!waypoints.has_value() || waypoints->empty()) {
      warnMissingCsv(target, now_sec);
      return;
    }

    const bool immediate_preempt = waypoint_executor_->shouldPreemptImmediately(target);
    const bool request_sent = waypoint_executor_->stepExecutorTarget(
      target, mode, waypoint_store_->waypointFile(target), *waypoints, immediate_preempt, now_sec);
    if (request_sent) {
      RCLCPP_INFO(
        get_logger(), "Requested executor target [%s] in mode [%s]",
        decision::toString(target).c_str(),
        mode == decision::TargetMode::ExecutorFollow ? "follow" : "through");
    }
  }

  std::optional<decision::Pose> currentRobotPose(double now_sec)
  {
    if (now_sec < next_tf_lookup_attempt_sec_) {
      return std::nullopt;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        config_.goal_frame_id,
        config_.maintain_goal.robot_base_frame,
        tf2::TimePointZero);
      decision::Pose pose;
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.z = transform.transform.translation.z;
      pose.qx = transform.transform.rotation.x;
      pose.qy = transform.transform.rotation.y;
      pose.qz = transform.transform.rotation.z;
      pose.qw = transform.transform.rotation.w;
      next_tf_lookup_attempt_sec_ = -1.0e9;
      return pose;
    } catch (const tf2::TransformException & ex) {
      next_tf_lookup_attempt_sec_ = now_sec + kTfLookupRetrySec;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot maintain wait target: transform %s <- %s unavailable: %s",
        config_.goal_frame_id.c_str(),
        config_.maintain_goal.robot_base_frame.c_str(),
        ex.what());
      return std::nullopt;
    }
  }

  std::optional<std::vector<decision::Pose>> loadPreparedWaypoints(
    decision::TargetName target,
    decision::TargetMode mode)
  {
    if (mode == decision::TargetMode::ExecutorFollow) {
      return waypoint_store_->prepareFollowWaypoints(target, std::nullopt);
    }
    return waypoint_store_->loadWaypoints(target);
  }

  void warnMissingCsv(decision::TargetName target, double now_sec)
  {
    auto & last_warn_sec = last_missing_csv_warn_sec_[target];
    if (now_sec - last_warn_sec < 5.0) {
      return;
    }

    last_warn_sec = now_sec;
    RCLCPP_WARN(
      get_logger(), "Target [%s] has no CSV. Fixed-point fallback has been removed.",
      decision::toString(target).c_str());
  }

  decision::DecisionConfig config_;
  decision::DecisionContext context_;
  std::unique_ptr<decision::DecisionStateMachine> state_machine_;
  std::unique_ptr<decision::WaypointStore> waypoint_store_;
  std::unique_ptr<decision::WaypointExecutorClient> waypoint_executor_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub_;
  rclcpp_action::Client<decision_interfaces::action::FollowWaypoints>::SharedPtr follow_client_;
  rclcpp_action::Client<decision_interfaces::action::FollowWaypoints>::SharedPtr through_client_;
  rclcpp::Subscription<decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<decision::ExecutorEvent> pending_executor_event_;
  std::map<decision::TargetName, double> last_missing_csv_warn_sec_;
  double next_tf_lookup_attempt_sec_{-1.0e9};

  // 中心区战术点池（复用 patrol.csv 的点）与随机换位路径。
  std::vector<decision::Pose> tactical_points_;
  bool tactical_pool_loaded_{false};
  double last_tactical_warn_sec_{-1.0e9};
  // 当前换位路径的点索引（随机 N 个不重复），待下发与"停当前位"标志。
  std::vector<std::size_t> tactical_path_;
  bool tactical_path_pending_{false};
  bool tactical_stop_sent_{false};
  std::mt19937 rng_{static_cast<std::mt19937::result_type>(
    std::chrono::steady_clock::now().time_since_epoch().count())};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BtActionReplacementNode>());
  rclcpp::shutdown();
  return 0;
}
