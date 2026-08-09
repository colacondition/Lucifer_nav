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
};

struct WaypointExecutorConfig
{
  std::string saved_waypoint_file_topic{"/waypoint_editor/saved_waypoint_file"};
  std::string executor_waypoints_topic{"/waypoint_editor/executor_waypoints"};
  std::string follow_status_topic{"/waypoint_editor/follow_status"};
  std::string through_status_topic{"/waypoint_editor/through_status"};
  std::string follow_service{"start_waypoint_following"};
  std::string through_service{"start_waypoint_through"};
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
  bool enable_patrol{false};
  std::string waypoint_map_name;
  std::string waypoint_search_root;
  std::string patrol_waypoint_file;
  std::string center_waypoint_file;
  std::string wait_center_waypoint_file;
  std::string home_waypoint_file;
  std::string wait_home_waypoint_file;
  std::string wait_hp_waypoint_file;
  double patrol_interval_sec{10.0};
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
  double target_change_hold_sec{0.25};
};

struct DecisionConfig
{
  double loop_hz{10.0};
  double retry_interval_sec{1.0};
  std::string goal_frame_id{"map"};
  TopicConfig topics;
  WaypointExecutorConfig waypoint_executor;
  GameConfig game;
  HpRecoveryConfig hp_recovery;
  TargetConfig targets;
  WaypointConfig waypoint;
  MaintainGoalConfig maintain_goal;
};

void declareDecisionParameters(rclcpp::Node & node);
DecisionConfig loadDecisionConfig(rclcpp::Node & node);
void validateDecisionConfig(DecisionConfig & config);

}  // namespace decision
