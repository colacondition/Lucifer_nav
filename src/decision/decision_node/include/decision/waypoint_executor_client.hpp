#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <decision_interfaces/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "decision/decision_config.hpp"
#include "decision/decision_context.hpp"
#include "decision/types.hpp"
#include "decision/waypoint_store.hpp"

namespace decision
{

// 哪些目标属于"守点型"：它们到达完成后会持续纠偏 —— 漂出 maintain_goal.xy_tolerance
// 并持续 drift_hold_sec 就重发目标。
//
// 抽成自由函数是为了能单测。**中心走位用的 TargetName::Patrol 必须不在这个集合里**：
// 走位的活动半径最远 1.58 m，而 xy_tolerance 只有 0.35 m，一旦走位被当成守点目标，
// 纠偏会每 0.8 s 把车拽回 wait_center 锚点一次，走位就废了。
// 这条隔离现在是"类型上"的，不依赖调用路径恰好绕过了门控。
bool targetNeedsMaintainGoal(TargetName target);


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
  // **上次下发的**目标/模式（不管有没有被接受）。
  //
  // ⚠️ 去抖必须看这个，不能看 running_target：running_target 只在 goal 被
  //    **接受**时才置位。目标连接受都没接受就被拒/被立刻 abort 时（典型：
  //    TF 还没起来，执行器拿不到车体位姿），running_target 一直是 Unknown，
  //    于是每一拍都被判成"换目标"、去抖永久失效 —— 实测会以 loop_hz(30Hz)
  //    疯狂重发，刷满 "unknown goal response, ignoring..." 并把真正的病因埋掉。
  std::optional<TargetName> last_attempt_target;
  std::optional<TargetMode> last_attempt_mode;
  ExecutorResultStatus result_status{ExecutorResultStatus::None};
  double last_send_time_sec{-1.0e9};
  double last_goal_success_time_sec{-1.0e9};
};

// 决策侧的执行器客户端。旧实现用 Trigger 服务 + 独立状态话题，靠三代
// generation 计数把异步响应和状态话题关联起来（最脆弱的一段）；现在改用
// FollowWaypoints action：goal 自带航点，结果由 result 回调直接送达，
// 抢占/取消/超时由 action 语义天然解决，状态关联逻辑整体删除。
class WaypointExecutorClient
{
public:
  using FollowWaypoints = decision_interfaces::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  explicit WaypointExecutorClient(DecisionConfig config);

  const WaypointExecutorState & state() const;
  void setRosInterfaces(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr direct_goal_pub,
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client,
    rclcpp_action::Client<FollowWaypoints>::SharedPtr through_client);
  void setActiveTarget(TargetName target);
  void setRunningTarget(TargetName target, TargetMode mode, double now_sec);
  // 终态通知钩子：result 回调里触发（decision_node 用它喂状态机事件）。
  void setResultCallback(std::function<void(TargetName, bool)> callback);
  // 终态结果应用（result 回调与单测共用）：仅当仍是对应目标时更新状态。
  WaypointExecutorState applyExecutorResult(
    TargetName target, TargetMode mode, bool success, double now_sec);

  bool shouldPreemptImmediately(TargetName target) const;
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
  // 结果超时守卫：goal 已下发但 last_goal_progress_sec_ 起算超过
  // config_.executor_result_timeout_sec（>0 时启用）仍无任何动作事件，
  // 就合成一次 Aborted 终态（必要时触发 result_callback_）并放行重发。
  // 返回 true 表示本拍合成了一次超时失败。executor 崩溃重生后 rclcpp_action
  // 客户端不会收到任何回调，没有这条守卫 running_target 会永久占位。
  bool checkGoalTimeout(double now_sec);
  // stop_at_final=false 让执行器**穿过**最后一个航点不停（也不开接近段）。
  // 默认 true = 到点停住，是 Home / WaitHp / Center 的语义。
  bool stepExecutorTarget(
    TargetName target,
    TargetMode mode,
    const std::string & waypoint_file,
    const std::vector<Pose> & waypoints,
    bool immediate_preempt,
    double now_sec,
    bool stop_at_final = true);

  // —— 测试钩子（仅单元测试使用）——
  // 把客户端置入「goal 已下发、等待首个动作事件」的悬挂态。
  void markGoalInFlightForTest(double now_sec)
  {
    goal_in_flight_ = true;
    last_goal_progress_sec_ = now_sec;
  }

private:
  nav_msgs::msg::Path buildPathMessage(const std::vector<Pose> & waypoints) const;
  geometry_msgs::msg::PoseStamped buildPoseStamped(const Pose & pose) const;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr actionClient(TargetMode mode) const;
  double distance(const Pose & a, const Pose & b) const;
  bool isWaitTarget(TargetName target) const;
  void clearMaintainDrift();

  DecisionConfig config_;
  WaypointExecutorState state_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr direct_goal_pub_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr through_client_;
  // 最近一次派发的 goal 句柄（结果回调用它做「只认最新」判断）。
  std::shared_ptr<GoalHandleFollowWaypoints> active_goal_handle_;
  std::function<void(TargetName, bool)> result_callback_;
  bool goal_in_flight_{false};
  // 最近一次动作事件（send / goal 接受 / result）的时刻，超时守卫的基准。
  double last_goal_progress_sec_{-1.0e9};
  std::optional<TargetName> maintain_target_;
  double maintain_drift_start_sec_{-1.0e9};
};

}  // namespace decision
