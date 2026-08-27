#include "decision/waypoint_executor_client.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace decision
{

WaypointExecutorClient::WaypointExecutorClient(DecisionConfig config)
: config_(std::move(config))
{
}

const WaypointExecutorState & WaypointExecutorClient::state() const
{
  return state_;
}

void WaypointExecutorClient::setRosInterfaces(
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr direct_goal_pub,
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client,
  rclcpp_action::Client<FollowWaypoints>::SharedPtr through_client)
{
  // 前两个发布器保留给 RViz 面板（override 高亮 / 当前文件显示），
  // 执行器的数据通路已经走 action。
  executor_waypoints_pub_ = std::move(executor_waypoints_pub);
  saved_waypoint_file_pub_ = std::move(saved_waypoint_file_pub);
  direct_goal_pub_ = std::move(direct_goal_pub);
  follow_client_ = std::move(follow_client);
  through_client_ = std::move(through_client);
}

void WaypointExecutorClient::setActiveTarget(TargetName target)
{
  state_.active_target = target;
}

void WaypointExecutorClient::setResultCallback(std::function<void(TargetName, bool)> callback)
{
  result_callback_ = std::move(callback);
}

void WaypointExecutorClient::setRunningTarget(
  TargetName target,
  TargetMode mode,
  double now_sec)
{
  state_.active_target = target;
  state_.running_target = target;
  state_.running_mode = mode;
  state_.result_status = ExecutorResultStatus::None;
  state_.last_send_time_sec = now_sec;
}

WaypointExecutorState WaypointExecutorClient::applyExecutorResult(
  TargetName target,
  TargetMode mode,
  bool success,
  double now_sec)
{
  // 结果只对「当前仍在跑的目标」生效：被抢占的旧目标晚到的结果直接忽略。
  const bool matches_running =
    state_.running_target == target && state_.running_mode == mode;
  if (!matches_running) {
    return state_;
  }

  auto updated = state_;
  if (success) {
    updated.result_status = ExecutorResultStatus::Succeeded;
    updated.last_goal_success_time_sec = now_sec;
  } else {
    updated.result_status = ExecutorResultStatus::Aborted;
  }
  updated.running_target.reset();
  updated.running_mode.reset();
  state_ = updated;
  return state_;
}

bool WaypointExecutorClient::shouldPreemptImmediately(TargetName target) const
{
  (void)state_;
  return target == TargetName::Home || target == TargetName::WaitHome;
}

bool WaypointExecutorClient::needMaintainReissue(
  TargetName target,
  const std::optional<Pose> & robot_pose,
  const std::optional<Pose> & target_pose,
  double now_sec)
{
  if (!config_.maintain_goal.enable || !isWaitTarget(target) ||
    !robot_pose.has_value() || !target_pose.has_value())
  {
    clearMaintainDrift();
    return false;
  }

  if (distance(*robot_pose, *target_pose) <= config_.maintain_goal.xy_tolerance) {
    clearMaintainDrift();
    return false;
  }

  if (maintain_target_ != target) {
    maintain_target_ = target;
    maintain_drift_start_sec_ = now_sec;
    return false;
  }

  if (now_sec - maintain_drift_start_sec_ < config_.maintain_goal.drift_hold_sec) {
    return false;
  }

  maintain_drift_start_sec_ = now_sec;
  return true;
}

bool WaypointExecutorClient::checkGoalTimeout(double now_sec)
{
  if (config_.executor_result_timeout_sec <= 0.0) {
    return false;
  }
  if (!goal_in_flight_) {
    return false;
  }
  const double elapsed = now_sec - last_goal_progress_sec_;
  if (elapsed < config_.executor_result_timeout_sec) {
    return false;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("decision"),
    "FollowWaypoints goal produced no event for %.1fs (timeout %.1fs); "
    "synthesizing abort so the target can be re-requested",
    elapsed, config_.executor_result_timeout_sec);

  // 收尾在途状态。此后 shouldRequestTarget 对同目标重新放行：
  // running 已清（或 status 已非 Succeeded），retry_interval 的防抖
  // 由 stepExecutorTarget 自带的 last_send_time 判断兜住节奏。
  goal_in_flight_ = false;
  active_goal_handle_.reset();

  if (state_.running_target.has_value() && state_.running_mode.has_value()) {
    const auto target = *state_.running_target;
    const auto mode = *state_.running_mode;
    applyExecutorResult(target, mode, /*success=*/false, now_sec);
    if (result_callback_) {
      result_callback_(target, false);
    }
  } else {
    // 还没走到 goal_response：running 未设，把挂起态标记为失败即可。
    if (state_.result_status == ExecutorResultStatus::None) {
      state_.result_status = ExecutorResultStatus::Aborted;
    }
  }
  return true;
}

bool WaypointExecutorClient::shouldRequestTarget(
  TargetName target,
  TargetMode mode,
  const std::optional<Pose> & robot_pose,
  const std::optional<Pose> & target_pose,
  double now_sec)
{
  if (state_.running_target == target && state_.running_mode == mode) {
    return false;
  }

  if (state_.active_target != target || state_.result_status != ExecutorResultStatus::Succeeded) {
    clearMaintainDrift();
    return true;
  }

  if (!config_.maintain_goal.enable || !isWaitTarget(target)) {
    clearMaintainDrift();
    return false;
  }

  if (robot_pose.has_value() && target_pose.has_value()) {
    return needMaintainReissue(target, robot_pose, target_pose, now_sec);
  }

  clearMaintainDrift();
  return false;
}

bool WaypointExecutorClient::stepExecutorTarget(
  TargetName target,
  TargetMode mode,
  const std::string & waypoint_file,
  const std::vector<Pose> & waypoints,
  bool immediate_preempt,
  double now_sec)
{
  if (mode != TargetMode::ExecutorFollow && mode != TargetMode::ExecutorThrough) {
    return false;
  }
  if (waypoints.empty()) {
    return false;
  }

  auto client = actionClient(mode);
  if (!client || !client->action_server_is_ready()) {
    return false;
  }

  // 旧 Trigger 实现靠 generation 关联 in-flight 请求，必须挡住重复下发；
  // action 模型里「发新目标」本身就是抢占手段（服务器停旧任务跑新任务），
  // 挡在这里会让非 Home 目标永远无法打断正在执行的任务（决策卡死）。
  //
  // 防抖只作用于「同目标的重复重试」：切目标（抢占）必须立刻发出。
  const bool target_changed =
    state_.running_target != target || state_.running_mode != mode;
  if (!immediate_preempt && !target_changed &&
    now_sec - state_.last_send_time_sec < config_.retry_interval_sec)
  {
    return false;
  }

  // 面板兼容：发布 override 航点与文件（旧 Trigger 通路仍依赖这两个话题，
  // RViz 面板也显示它们）；执行器本体走 action。
  const auto path_msg = buildPathMessage(waypoints);
  if (executor_waypoints_pub_) {
    executor_waypoints_pub_->publish(path_msg);
  }
  if (saved_waypoint_file_pub_ && !waypoint_file.empty()) {
    std_msgs::msg::String file_msg;
    file_msg.data = waypoint_file;
    saved_waypoint_file_pub_->publish(file_msg);
  }
  if (immediate_preempt && direct_goal_pub_) {
    // Home 型目标：先把第一个航点直接发给导航，不等执行器起跑。
    direct_goal_pub_->publish(buildPoseStamped(waypoints.front()));
  }

  state_.last_send_time_sec = now_sec;
  state_.active_target = target;
  state_.result_status = ExecutorResultStatus::None;
  last_goal_progress_sec_ = now_sec;

  auto goal = FollowWaypoints::Goal();
  goal.waypoints = path_msg;

  auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  options.goal_response_callback =
    [this, target, mode, now_sec](
      std::shared_ptr<GoalHandleFollowWaypoints> goal_handle) {
      try {
      if (!goal_handle) {
        // 被拒绝：目标/航点无效，视为一次失败但不重置 running（可能仍在跑旧目标）。
        if (state_.active_target == target && state_.result_status == ExecutorResultStatus::None) {
          state_.result_status = ExecutorResultStatus::Aborted;
        }
        return;
      }
      active_goal_handle_ = goal_handle;
      setRunningTarget(target, mode, now_sec);
      // goal 被接受也算一次进展：重置超时基准。
      last_goal_progress_sec_ = now_sec;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          rclcpp::get_logger("decision"), "goal_response_callback threw: %s", ex.what());
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("decision"), "goal_response_callback threw");
      }
    };
  options.result_callback =
    [this, target, mode](const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult & wrapped) {
      try {
      // 只认最新的 goal：被抢占的旧目标晚到的结果不能覆盖新目标的状态。
      if (!active_goal_handle_ ||
        wrapped.goal_id != active_goal_handle_->get_goal_id())
      {
        return;
      }
      goal_in_flight_ = false;
      active_goal_handle_.reset();
      // 终态结果是最强的「有进展」信号。
      last_goal_progress_sec_ =
        rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
      const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped.result && wrapped.result->success;
      // 只把「确实应用到当前目标」的结果喂给状态机：被抢占的旧目标晚到的
      // canceled 结果不能误报成一个 Aborted 事件。
      const bool matches_running =
        state_.running_target == target && state_.running_mode == mode;
      if (matches_running) {
        applyExecutorResult(
          target, mode, success,
          rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds());
        if (result_callback_) {
          result_callback_(target, success);
        }
      }
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          rclcpp::get_logger("decision"), "result_callback threw: %s", ex.what());
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("decision"), "result_callback threw");
      }
    };

  goal_in_flight_ = true;
  client->async_send_goal(goal, options);
  return true;
}

nav_msgs::msg::Path WaypointExecutorClient::buildPathMessage(
  const std::vector<Pose> & waypoints) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.goal_frame_id;
  path.header.stamp = rclcpp::Clock().now();
  path.poses.reserve(waypoints.size());
  for (const auto & waypoint : waypoints) {
    auto pose = buildPoseStamped(waypoint);
    pose.header.stamp = path.header.stamp;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped WaypointExecutorClient::buildPoseStamped(const Pose & pose) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = config_.goal_frame_id;
  msg.header.stamp = rclcpp::Clock().now();
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.position.z = pose.z;
  msg.pose.orientation.x = pose.qx;
  msg.pose.orientation.y = pose.qy;
  msg.pose.orientation.z = pose.qz;
  msg.pose.orientation.w = pose.qw;
  return msg;
}

rclcpp_action::Client<WaypointExecutorClient::FollowWaypoints>::SharedPtr
WaypointExecutorClient::actionClient(TargetMode mode) const
{
  if (mode == TargetMode::ExecutorFollow) {
    return follow_client_;
  }
  if (mode == TargetMode::ExecutorThrough) {
    return through_client_;
  }
  return nullptr;
}

double WaypointExecutorClient::distance(const Pose & a, const Pose & b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool WaypointExecutorClient::isWaitTarget(TargetName target) const
{
  return target == TargetName::WaitHome || target == TargetName::WaitHp ||
         target == TargetName::WaitCenter;
}

void WaypointExecutorClient::clearMaintainDrift()
{
  maintain_target_.reset();
  maintain_drift_start_sec_ = -1.0e9;
}

}  // namespace decision
