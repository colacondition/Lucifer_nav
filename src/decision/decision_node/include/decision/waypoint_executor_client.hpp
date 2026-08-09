#pragma once

#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "decision/decision_config.hpp"
#include "decision/decision_context.hpp"
#include "decision/types.hpp"
#include "decision/waypoint_store.hpp"

namespace decision
{

enum class ExecutorResultStatus
{
  None,
  Succeeded,
  Aborted,
  Unknown
};

struct WaypointExecutorState
{
  std::optional<TargetName> active_target;
  std::optional<TargetName> running_target;
  std::optional<TargetMode> running_mode;
  ExecutorResultStatus result_status{ExecutorResultStatus::None};
  double last_send_time_sec{-1.0e9};
  double last_goal_success_time_sec{-1.0e9};
};

class WaypointExecutorClient
{
public:
  explicit WaypointExecutorClient(DecisionConfig config);

  const WaypointExecutorState & state() const;
  void setRosInterfaces(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr direct_goal_pub,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr follow_client,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr through_client);
  void setActiveTarget(TargetName target);
  void setRunningTarget(TargetName target, TargetMode mode, double now_sec);
  std::optional<TargetName> targetForStatus(TargetMode mode, const std::string & status);

  bool shouldPreemptImmediately(TargetName target) const;
  WaypointExecutorState onExecutorStatus(
    TargetName target,
    TargetMode mode,
    const std::string & status,
    double now_sec);
  bool needMaintainReissue(
    TargetName target,
    const std::optional<Pose> & robot_pose,
    const std::optional<Pose> & target_pose,
    double now_sec);
  bool shouldRequestTarget(
    TargetName target,
    TargetMode mode,
    const std::optional<Pose> & robot_pose,
    const std::optional<Pose> & target_pose,
    double now_sec);
  bool stepExecutorTarget(
    TargetName target,
    TargetMode mode,
    const std::string & waypoint_file,
    const std::vector<Pose> & waypoints,
    bool immediate_preempt,
    double now_sec);

private:
  nav_msgs::msg::Path buildPathMessage(const std::vector<Pose> & waypoints) const;
  geometry_msgs::msg::PoseStamped buildPoseStamped(const Pose & pose) const;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr serviceClient(TargetMode mode) const;
  void onStartResponse(
    TargetName target,
    TargetMode mode,
    double request_time_sec,
    std::uint64_t request_id,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  double distance(const Pose & a, const Pose & b) const;
  bool isWaitTarget(TargetName target) const;
  void clearMaintainDrift();

  DecisionConfig config_;
  WaypointExecutorState state_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr direct_goal_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr follow_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr through_client_;
  std::optional<TargetName> pending_target_;
  std::optional<TargetMode> pending_mode_;
  double pending_since_sec_{-1.0e9};
  bool start_request_in_flight_{false};
  std::optional<TargetName> start_request_target_;
  std::optional<TargetMode> start_request_mode_;
  std::uint64_t start_request_generation_{0};
  std::uint64_t next_start_request_generation_{0};
  std::optional<std::uint64_t> handoff_request_generation_;
  std::optional<std::uint64_t> terminal_request_generation_;
  std::optional<TargetName> maintain_target_;
  double maintain_drift_start_sec_{-1.0e9};
};

}  // namespace decision
