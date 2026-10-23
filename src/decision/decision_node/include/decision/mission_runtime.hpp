#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <decision_interfaces/action/follow_waypoints.hpp>
#include <decision_interfaces/msg/game_status.hpp>
#include <decision_interfaces/msg/robot_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "decision/bt_nodes.hpp"
#include "decision/center_patrol.hpp"
#include "decision/center_zone.hpp"
#include "decision/decision_config.hpp"
#include "decision/decision_context.hpp"
#include "decision/target_input.hpp"
#include "decision/types.hpp"
#include "decision/waypoint_executor_client.hpp"
#include "decision/waypoint_store.hpp"

namespace decision
{

// 决策运行时：把旧 bt_action_replacement_node 里"与 ROS/执行器/巡逻/追击相关"的接线
// 全部收在这里。BT 叶子只通过 bt::MissionBackend 接口调用它：
//
//   - 输入：订阅回调把 msg 喂进来（RobotStatus/GameStatus/localization/自瞄载荷），
//           这里维护低血迟滞、比赛进行判定、目标输入滤波；
//   - 输出：固定目标/走位路径/追击点的下发（FollowWaypoints action），以及
//           /goal_pose 立即抢占、执行器面板话题、target_debug；
//   - 状态：home/center 两个 latch（替代旧 FSM 的段）、巡逻游标、执行器终态事件队列。
//
// 决策"选哪条分支"交给行为树（config/trees/rmul.xml + bt_nodes.hpp），这里不再有
// 优先级判断。这样旧 node 927 行的编排被拆成"运行时接线" + "XML 优先级"。
class MissionRuntime : public bt::MissionBackend
{
public:
  using FollowWaypoints = decision_interfaces::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  MissionRuntime(DecisionConfig config, rclcpp::Node & node);
  ~MissionRuntime() override = default;

  // 启动时调用一次：读中心区角点、生成走位点池、打启动日志。
  void initialize();

  // 每拍 tree.tickRoot() 前后调用。
  //   beginTick：清本拍缓存（位姿/追击判据）+ 执行器结果超时守卫；
  //   endTick  ：丢弃没被任何动作消费的终态事件 + 节流发布 target_debug。
  void beginTick();
  void endTick();

  // ---------- 输入（订阅回调调用） ----------
  void updateRobotStatus(const decision_interfaces::msg::RobotStatus & msg, double now_sec);
  void updateGameStatus(const decision_interfaces::msg::GameStatus & msg, double now_sec);
  void updateLocalizationStatus(const std::string & payload);
  void updateTargetPayload(const std::string & payload, double now_sec);

  // ---------- bt::MissionBackend ----------
  double nowSec() const override;
  bool gameActive() const override;
  bool hpLow() const override;
  bool hpKnown() const override;
  bool hpZero() const override;
  bool revived() const override;
  bool localizationLost() const override;
  bool homeReached() const override;
  bool centerReached() const override;
  bool chaseReady() override;
  bool ensureFixedTarget(TargetName target) override;
  std::optional<bool> takeResult(TargetName target) override;
  bool centerPatrolEnabled() const override;
  double patrolGroupGapSec() const override;
  bool patrolPoolEmpty() const override;
  PatrolPath takePatrolPath() override;
  bool dispatchPatrol(const PatrolPath & path) override;
  bt::ChaseDecision evaluateChase() override;
  bool dispatchChase(const bt::ChaseDecision & decision) override;
  void reportChase(const bt::ChaseDecision & decision, bool dispatched) override;
  void setActiveState(const std::string & state) override;
  void logInfo(const std::string & message) override;
  void logWarn(const std::string & message) override;

  // ---------- 节点侧读取 ----------
  const DecisionConfig & config() const { return config_; }
  const std::string & activeState() const { return active_state_; }
  std::string localizationStatusPayload() const
  {
    return context_.localizationStatusPayload();
  }
  bool hasRobotStatus() const { return context_.effectiveCurrentHp().has_value(); }
  bool hasGameStatus() const { return context_.gameStatus().has_value(); }

  // 重复节点名 / 重复执行器检查（节流，节点每拍调用）。
  void checkDuplicateNodes();

  // target_debug 字符串（节点按 0.5s 节流发布到 center_patrol/target_debug）。
  std::string buildTargetDebugString() const;

private:
  struct PendingEvent
  {
    TargetName target;
    bool success;
  };

  void handleExecutorResult(TargetName target, bool success);
  bool dispatchPath(const std::vector<Pose> & path, TargetName target);
  std::optional<Pose> poseInMap(
    const std::string & frame, double now_sec, double & next_attempt, const char * what,
    bool & ready_logged);
  std::optional<Pose> robotPoseInMap();
  std::optional<Pose> lidarPoseInMap();
  void refreshPoseCaches();
  std::optional<std::vector<Pose>> loadPreparedWaypoints(
    TargetName target, TargetMode mode);
  void warnMissingCsv(TargetName target, double now_sec);
  void setChaseOutcome(const bt::ChaseDecision & decision, bool dispatched);

  void loadCenterZone();
  void setupPatrolCandidates();
  void loadTacticalPoints();
  const std::vector<Pose> * tacticalPool(double now_sec);

  rclcpp::Node & node_;
  DecisionConfig config_;
  DecisionContext context_;
  std::unique_ptr<WaypointStore> waypoint_store_;
  std::unique_ptr<WaypointExecutorClient> waypoint_executor_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_debug_pub_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr through_client_;

  // 输入状态
  bool hp_low_{false};
  // 死亡/复活判定：hp==0 时 dead_seen_=true；下一次 hp>0 视为"本局刚复活"。
  // revived_ 一直保持到成功回家（到补给区解除虚弱）或本局结束。
  bool dead_seen_{false};
  bool revived_{false};
  TargetInput target_input_;
  std::uint64_t target_msg_count_{0};
  bool target_seen_logged_{false};

  // 段锁存（替代旧 FSM 的 HOME/CENTER 段）
  bt::DecisionLatch latch_;

  // 追击
  CenterZone center_zone_;
  bool chase_decision_valid_{false};
  bt::ChaseDecision chase_decision_;
  bool chase_dispatched_{false};

  // 走位
  const std::vector<Pose> * active_pool_{nullptr};
  std::vector<Pose> grid_points_;
  std::vector<Pose> tactical_points_;
  bool tactical_pool_loaded_{false};
  bool patrol_candidates_tried_{false};
  std::size_t patrol_cursor_{0};
  double last_tactical_warn_sec_{-1.0e9};

  // TF 缓存/重试
  bool poses_refreshed_{false};
  std::optional<Pose> robot_pose_cache_;
  std::optional<Pose> lidar_pose_cache_;
  double next_tf_lookup_attempt_sec_{-1.0e9};
  double next_lidar_tf_attempt_sec_{-1.0e9};
  bool car_pose_ready_logged_{false};
  bool lidar_pose_ready_logged_{false};

  // 执行器终态事件队列：动作在 tick 期间按目标名消费；endTick 丢弃剩余。
  std::deque<PendingEvent> pending_events_;

  // /decision/state
  std::string active_state_{"BOOT"};

  // debug/节流
  double next_target_debug_sec_{-1.0e9};
  std::map<TargetName, double> last_missing_csv_warn_sec_;
  double next_dup_check_sec_{0.0};
  bool dup_node_reported_{false};
  bool dup_executor_reported_{false};
};

}  // namespace decision
