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
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr follow_client,
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr through_client)
{
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

std::optional<TargetName> WaypointExecutorClient::targetForStatus(
  TargetMode mode,
  const std::string & status)
{
  const bool matches_in_flight =
    start_request_in_flight_ && start_request_mode_ == mode;
  if (status == "RUNNING" && matches_in_flight) {
    handoff_request_generation_ = start_request_generation_;
    return start_request_target_;
  }
  const bool is_terminal = status == "COMPLETED" || status == "ABORTED";
  if (is_terminal && matches_in_flight &&
    handoff_request_generation_ == start_request_generation_)
  {
    return start_request_target_;
  }
  if (state_.running_target.has_value() && state_.running_mode == mode) {
    return state_.running_target;
  }
  if (matches_in_flight) {
    return start_request_target_;
  }
  return std::nullopt;
}

bool WaypointExecutorClient::shouldPreemptImmediately(TargetName target) const
{
  (void)state_;
  return target == TargetName::Home || target == TargetName::WaitHome;
}

WaypointExecutorState WaypointExecutorClient::onExecutorStatus(
  TargetName target,
  TargetMode mode,
  const std::string & status,
  double now_sec)
{
  const bool matches_running =
    state_.running_target == target && state_.running_mode == mode;
  const bool matches_in_flight =
    start_request_in_flight_ && start_request_target_ == target && start_request_mode_ == mode;
  if (!matches_running && !matches_in_flight) {
    return state_;
  }

  auto updated = state_;
  if (status == "COMPLETED") {
    updated.result_status = ExecutorResultStatus::Succeeded;
    updated.running_target.reset();
    updated.running_mode.reset();
    updated.last_goal_success_time_sec = now_sec;
  } else if (status == "ABORTED") {
    updated.result_status = ExecutorResultStatus::Aborted;
    updated.running_target.reset();
    updated.running_mode.reset();
  } else {
    updated.result_status = ExecutorResultStatus::Unknown;
  }

  if (matches_in_flight && (status == "COMPLETED" || status == "ABORTED")) {
    terminal_request_generation_ = start_request_generation_;
    if (handoff_request_generation_ == start_request_generation_) {
      handoff_request_generation_.reset();
    }
  }
  if (state_.active_target == target) {
    state_ = updated;
    return state_;
  }
  if (matches_running) {
    state_.running_target.reset();
    state_.running_mode.reset();
  }

  return updated;
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
  if (!executor_waypoints_pub_ || !saved_waypoint_file_pub_ || waypoints.empty()) {
    return false;
  }
  if (start_request_in_flight_ && !immediate_preempt) {
    return false;
  }

  const bool pending_matches = pending_target_ == target && pending_mode_ == mode;
  if (!pending_matches) {
    executor_waypoints_pub_->publish(buildPathMessage(waypoints));
    if (immediate_preempt && direct_goal_pub_) {
      direct_goal_pub_->publish(buildPoseStamped(waypoints.front()));
    }

    std_msgs::msg::String file_msg;
    file_msg.data = waypoint_file;
    saved_waypoint_file_pub_->publish(file_msg);

    pending_target_ = target;
    pending_mode_ = mode;
    pending_since_sec_ = now_sec;
    state_.active_target = target;
    state_.result_status = ExecutorResultStatus::None;
    if (!immediate_preempt) {
      return false;
    }
  }

  if (start_request_in_flight_) {
    return false;
  }

  if (!immediate_preempt &&
    now_sec - pending_since_sec_ < config_.waypoint_executor.start_delay_sec)
  {
    return false;
  }
  if (!immediate_preempt && now_sec - state_.last_send_time_sec < config_.retry_interval_sec) {
    return false;
  }

  auto client = serviceClient(mode);
  if (!client || !client->wait_for_service(std::chrono::seconds(0))) {
    return false;
  }

  start_request_in_flight_ = true;
  state_.last_send_time_sec = now_sec;
  start_request_target_ = target;
  start_request_mode_ = mode;
  start_request_generation_ = ++next_start_request_generation_;
  handoff_request_generation_.reset();
  const auto request_id = start_request_generation_;
  const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    request,
    [this, target, mode, now_sec, request_id](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      onStartResponse(target, mode, now_sec, request_id, std::move(future));
    });
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

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr WaypointExecutorClient::serviceClient(
  TargetMode mode) const
{
  if (mode == TargetMode::ExecutorFollow) {
    return follow_client_;
  }
  if (mode == TargetMode::ExecutorThrough) {
    return through_client_;
  }
  return nullptr;
}

void WaypointExecutorClient::onStartResponse(
  TargetName target,
  TargetMode mode,
  double request_time_sec,
  std::uint64_t request_id,
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  if (!start_request_in_flight_ || request_id != start_request_generation_) {
    return;
  }

  const bool pending_matches_request = pending_target_ == target && pending_mode_ == mode;
  const bool terminal_received = terminal_request_generation_ == request_id;
  start_request_in_flight_ = false;
  start_request_target_.reset();
  start_request_mode_.reset();
  start_request_generation_ = 0;
  handoff_request_generation_.reset();
  if (terminal_received) {
    terminal_request_generation_.reset();
  }
  if (pending_matches_request) {
    pending_target_.reset();
    pending_mode_.reset();
    pending_since_sec_ = -1.0e9;
  }

  try {
    const auto response = future.get();
    if (!response->success) {
      if (pending_matches_request && !terminal_received) {
        state_.result_status = ExecutorResultStatus::Aborted;
      }
      return;
    }
  } catch (const std::exception &) {
    if (pending_matches_request && !terminal_received) {
      state_.result_status = ExecutorResultStatus::Unknown;
    }
    return;
  }

  if (terminal_received || !pending_matches_request) {
    return;
  }
  setRunningTarget(target, mode, request_time_sec);
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
