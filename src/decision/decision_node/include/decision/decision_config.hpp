#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace decision
{

struct TopicConfig
{
  std::string robot_status{"robot_status"};
  std::string game_status{"game_status"};
  std::string goal{"/goal_pose"};
  std::string decision_state{"/decision/state"};
  // fast_location 二态 OK/LOST；没这条时门是开的（建图/仿真可以不起定位）。
  std::string localization_status{"/localization_status"};
};

struct IntegrityGateConfig
{
  // 仅 LOST 冻 map 系下发；OK（含握住 TF / 走廊投影）不停。
  bool enable{true};
};

struct WaypointExecutorConfig
{
  std::string saved_waypoint_file_topic{"/waypoint_editor/saved_waypoint_file"};
  std::string executor_waypoints_topic{"/waypoint_editor/executor_waypoints"};
  // action 接口（decision → 执行器的数据通路；旧 Trigger 服务/状态话题已废弃）
  std::string follow_action{"/waypoint_editor/follow_waypoints"};
  std::string through_action{"/waypoint_editor/through_waypoints"};
  double start_delay_sec{0.2};
};

struct GameConfig
{
  int progress{4};
  int lower_remain_time{0};
  int higher_remain_time{300};
};

struct HpRecoveryConfig
{
  int low_threshold{120};
  int high_threshold{400};
};

struct TargetConfig
{
  std::string patrol_waypoint_file;
  std::string center_waypoint_file;
  std::string wait_center_waypoint_file;
  std::string home_waypoint_file;
  std::string wait_home_waypoint_file;
  std::string wait_hp_waypoint_file;
};

struct WaypointConfig
{
  double switch_distance{0.6};
  double final_goal_tolerance{0.35};
};

struct MaintainGoalConfig
{
  bool enable{true};
  std::string robot_base_frame{"base_link_fake"};
  double xy_tolerance{0.35};
  double drift_hold_sec{0.8};
};

// 战斗感知参数：把中心区行为从"定时巡逻"改为"按交战态势反应"。
// 只依据两个直接信号判定，不做掉血率推断：
//   shooter_heat 在开火（热量高） → 正面交火 ENGAGING，站定输出
//   is_attacked   在挨打           → 被压制/被偷 SUPPRESSED，换位脱离
//   两者皆无                       → 平静 CALM，守中心
// 切换防抖由 engage_hold / reposition_hold 最小保持时长承担。
struct CombatConfig
{
  bool enable{true};
  // shooter_heat 高于此值视为"我正在开火"（热量非零即视为最近在射击）。
  int firing_heat_threshold{1};
  // 进入 ENGAGE 后至少保持多久，避免交火状态抖动。
  double engage_hold_sec{0.4};
  // REPOSITION（被压制换位）后至少保持多久再重新评估。
  double reposition_hold_sec{0.5};
  // REPOSITION 换位路径长度：从点池随机挑 N 个不重复的点连成路径依次走完。
  int reposition_path_len{3};
  // 换位脱离进行中暂缓低血回家撤退的宽限时长：给换位动作时间完成，
  // 避免"刚切到 REPOSITION 就被扣血拽回家、路径都还没出来"。
  // 宽限结束仍低血才回家保命。
  double reposition_grace_sec{3.0};
};

struct DecisionConfig
{
  double loop_hz{10.0};
  double retry_interval_sec{1.0};
  std::string goal_frame_id{"map"};
  TopicConfig topics;
  IntegrityGateConfig integrity_gate;
  WaypointExecutorConfig waypoint_executor;
  GameConfig game;
  HpRecoveryConfig hp_recovery;
  TargetConfig targets;
  WaypointConfig waypoint;
  MaintainGoalConfig maintain_goal;
  CombatConfig combat;
};

void declareDecisionParameters(rclcpp::Node & node);
DecisionConfig loadDecisionConfig(rclcpp::Node & node);
void validateDecisionConfig(DecisionConfig & config);

}  // namespace decision
