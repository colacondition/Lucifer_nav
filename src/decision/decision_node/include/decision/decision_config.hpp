#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "decision/target_input.hpp"

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
  // 执行器的节点名。用来做"是不是起了两个执行器"的自诊断 ——
  // 两个同名 action server 会让客户端收到**两份**响应（第二份被 rclcpp_action
  // 记为 "unknown goal/result response, ignoring..."），而且两个执行器会各自
  // 驱动底盘互相打架。这个名字由 launch 的 `name=` 决定。
  std::string node_name{"waypoint_follow_executor"};
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
  // 两类回家路线：
  //   home_tunnel —— 复活后无敌/虚弱期走隧道快线，尽快到补给区解除虚弱；
  //   home_safe   —— 残血回家绕远路，避免在隧道被拦截打死。
  std::string home_tunnel_waypoint_file;
  std::string home_safe_waypoint_file;
  std::string wait_home_waypoint_file;
  std::string wait_hp_waypoint_file;
  // 中心占领区四个角点（range.csv），沿用同一套航点 CSV 格式
  std::string zone_waypoint_file;
};

// 中心占领区：场地上的一个凸四边形，用来把追击目标夹在安全区内。
//
// ⚠️ 录点约定：range.csv 里的框是**实际判定区往里缩过**的，所以框里就含有安全余量。
// 这带来两个好处：
//   1. 框 ⊂ 真实区 ⇒ 夹到框里天然不会越出真实占领区
//   2. patrol.csv 的点贴框边、甚至微出框几厘米都是正常的，**不是错误**
// 因此 margin 只要很小（对冲定位抖动），叠大了就是双重内缩、白白浪费本来就不大的区域。
struct CenterZoneConfig
{
  bool enable{true};
  // 用来把自瞄的"云台系"坐标转到 map 的位姿取自哪个 frame。
  //
  // ⚠️ 必须是 **base_link**，不能是 maintain_goal.robot_base_frame(base_link_fake)。
  //    super_lio 约定：`odom->base_link` 这个 TF 装的是**雷达位姿**
  //    （small_glim_node.cpp:345 "the base_link odometry actually carries the lidar pose"），
  //    而雷达装在云台上 ⇒ 它就是云台在 map 里的位姿，正是自瞄需要的那个旋转。
  //    base_link_fake 的 yaw 被人为改成"路径切向"，拿它旋转会得到完全错误的方向。
  std::string lidar_frame{"base_link"};
  // 在四个角点基础上再往内缩多少米。默认 0.10：框本身已含余量，这只对冲定位抖动。
  double margin_m{0.10};
  // 某个巡逻点跑到框外超过这么多米就告警 —— 那才是录点/map 出错，而不是有意的内缩。
  double warn_outside_m{0.30};
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

// 追击：把敌人位置夹进区域当目标点。
//
// 判据全在 target_input.hpp 里做完了（防抖/迟滞/滤波/重发），这里只放参数。
struct TargetChaseConfig
{
  bool enable{true};
  // 与敌人保持的距离（米）。目标点取"敌人周围这个半径的圆上、离车最近的那一点"，
  // 而不是敌人本身 —— 离太近自瞄打不稳。
  double standoff_m{1.5};
  // 目标点离车比这还近就不下发（已经在圈上了，不用动）。避免原地抖。
  double min_goal_move_m{0.3};
  // 自瞄的话题名。上游 QoS 是 RELIABLE，订阅端**必须**也用 reliable，
  // 否则一条都收不到、而且不报错。
  std::string topic{"auto_aim_target_pos"};
  TargetInputConfig input;
};

// 中心走位：在中心区域里按顺序走网格点，走完一组停一会儿再走下一组。
//
// 刻意**没有**激进度、也**不看时间**：哪一侧算"激进"取决于场上实际结构
// （过道是 S 型的、哪一侧有掩护、敌人从哪个口进来），不是能从坐标推出来的量。
// 走位本身也不改变占领权，它只是"别站着不动"。
struct CenterPatrolConfig
{
  bool enable{true};
  // 区域网格的格距（米）。越小点越密、走得越细，但也越琐碎。
  double grid_spacing_m{0.5};
  // 一条路径取几个网格点。
  int path_len{3};
  // 一组走完之后停顿多久再走下一组（从"走完那一刻"起算）。
  double group_gap_sec{4.5};
};

struct DecisionConfig
{
  double loop_hz{10.0};
  double retry_interval_sec{1.0};
  // 目标下发后在此时限内收不到任何动作事件（goal 接受 / 结果返回）时，
  // 合成一次 Aborted 终态并放行重发——兜住 executor 崩溃重生后旧 action
  // 永不回结果、decision 永不重发的静默挂死。0 = 关闭（时限必须大于
  // 最长合法航点段的执行时间，按赛场实测给值；见 bringup decision.yaml）。
  double executor_result_timeout_sec{0.0};
  std::string goal_frame_id{"map"};
  TopicConfig topics;
  IntegrityGateConfig integrity_gate;
  WaypointExecutorConfig waypoint_executor;
  GameConfig game;
  HpRecoveryConfig hp_recovery;
  TargetConfig targets;
  WaypointConfig waypoint;
  MaintainGoalConfig maintain_goal;
  CenterZoneConfig center_zone;
  TargetChaseConfig target_chase;
  CenterPatrolConfig center_patrol;
};

void declareDecisionParameters(rclcpp::Node & node);
DecisionConfig loadDecisionConfig(rclcpp::Node & node);
void validateDecisionConfig(DecisionConfig & config);

}  // namespace decision
