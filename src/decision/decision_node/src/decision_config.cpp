#include "decision/decision_config.hpp"

#include <algorithm>

namespace decision
{
namespace
{

template<typename T>
void declareParameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  node.declare_parameter<T>(name, default_value);
}

}  // namespace

void declareDecisionParameters(rclcpp::Node & node)
{
  const DecisionConfig defaults;

  declareParameter(node, "loop_hz", defaults.loop_hz);
  declareParameter(node, "retry_interval_sec", defaults.retry_interval_sec);
  declareParameter(node, "executor_result_timeout_sec", defaults.executor_result_timeout_sec);
  declareParameter(node, "goal_frame_id", defaults.goal_frame_id);

  declareParameter(node, "topics.robot_status", defaults.topics.robot_status);
  declareParameter(node, "topics.game_status", defaults.topics.game_status);
  declareParameter(node, "topics.goal", defaults.topics.goal);
  declareParameter(node, "topics.decision_state", defaults.topics.decision_state);
  declareParameter(
    node, "topics.localization_status", defaults.topics.localization_status);
  declareParameter(node, "integrity_gate.enable", defaults.integrity_gate.enable);

  declareParameter(
    node, "waypoint_executor.saved_waypoint_file_topic",
    defaults.waypoint_executor.saved_waypoint_file_topic);
  declareParameter(
    node, "waypoint_executor.executor_waypoints_topic",
    defaults.waypoint_executor.executor_waypoints_topic);
  declareParameter(node, "waypoint_executor.follow_action", defaults.waypoint_executor.follow_action);
  declareParameter(node, "waypoint_executor.through_action", defaults.waypoint_executor.through_action);
  declareParameter(node, "waypoint_executor.start_delay_sec", defaults.waypoint_executor.start_delay_sec);

  declareParameter(node, "game.progress", defaults.game.progress);
  declareParameter(node, "game.lower_remain_time", defaults.game.lower_remain_time);
  declareParameter(node, "game.higher_remain_time", defaults.game.higher_remain_time);

  declareParameter(node, "hp_recovery.low_threshold", defaults.hp_recovery.low_threshold);
  declareParameter(node, "hp_recovery.high_threshold", defaults.hp_recovery.high_threshold);

  declareParameter(node, "targets.patrol_waypoint_file", defaults.targets.patrol_waypoint_file);
  declareParameter(node, "targets.center_waypoint_file", defaults.targets.center_waypoint_file);
  declareParameter(node, "targets.wait_center_waypoint_file", defaults.targets.wait_center_waypoint_file);
  declareParameter(node, "targets.home_waypoint_file", defaults.targets.home_waypoint_file);
  declareParameter(node, "targets.wait_home_waypoint_file", defaults.targets.wait_home_waypoint_file);
  declareParameter(node, "targets.wait_hp_waypoint_file", defaults.targets.wait_hp_waypoint_file);

  declareParameter(node, "waypoint.switch_distance", defaults.waypoint.switch_distance);
  declareParameter(node, "waypoint.final_goal_tolerance", defaults.waypoint.final_goal_tolerance);

  declareParameter(node, "maintain_goal.enable", defaults.maintain_goal.enable);
  declareParameter(
    node, "maintain_goal.robot_base_frame", defaults.maintain_goal.robot_base_frame);
  declareParameter(node, "maintain_goal.xy_tolerance", defaults.maintain_goal.xy_tolerance);
  declareParameter(node, "maintain_goal.drift_hold_sec", defaults.maintain_goal.drift_hold_sec);

  declareParameter(node, "combat.enable", defaults.combat.enable);
  declareParameter(node, "combat.firing_heat_threshold", defaults.combat.firing_heat_threshold);
  declareParameter(node, "combat.engage_hold_sec", defaults.combat.engage_hold_sec);
  declareParameter(node, "combat.reposition_hold_sec", defaults.combat.reposition_hold_sec);
  declareParameter(node, "combat.reposition_path_len", defaults.combat.reposition_path_len);
  declareParameter(node, "combat.reposition_grace_sec", defaults.combat.reposition_grace_sec);

}

DecisionConfig loadDecisionConfig(rclcpp::Node & node)
{
  DecisionConfig config;

  node.get_parameter("loop_hz", config.loop_hz);
  node.get_parameter("retry_interval_sec", config.retry_interval_sec);
  node.get_parameter("executor_result_timeout_sec", config.executor_result_timeout_sec);
  node.get_parameter("goal_frame_id", config.goal_frame_id);

  node.get_parameter("topics.robot_status", config.topics.robot_status);
  node.get_parameter("topics.game_status", config.topics.game_status);
  node.get_parameter("topics.goal", config.topics.goal);
  node.get_parameter("topics.decision_state", config.topics.decision_state);
  node.get_parameter("topics.localization_status", config.topics.localization_status);
  node.get_parameter("integrity_gate.enable", config.integrity_gate.enable);

  node.get_parameter(
    "waypoint_executor.saved_waypoint_file_topic",
    config.waypoint_executor.saved_waypoint_file_topic);
  node.get_parameter(
    "waypoint_executor.executor_waypoints_topic",
    config.waypoint_executor.executor_waypoints_topic);
  node.get_parameter("waypoint_executor.follow_action", config.waypoint_executor.follow_action);
  node.get_parameter("waypoint_executor.through_action", config.waypoint_executor.through_action);
  node.get_parameter("waypoint_executor.start_delay_sec", config.waypoint_executor.start_delay_sec);

  node.get_parameter("game.progress", config.game.progress);
  node.get_parameter("game.lower_remain_time", config.game.lower_remain_time);
  node.get_parameter("game.higher_remain_time", config.game.higher_remain_time);

  node.get_parameter("hp_recovery.low_threshold", config.hp_recovery.low_threshold);
  node.get_parameter("hp_recovery.high_threshold", config.hp_recovery.high_threshold);

  node.get_parameter("targets.patrol_waypoint_file", config.targets.patrol_waypoint_file);
  node.get_parameter("targets.center_waypoint_file", config.targets.center_waypoint_file);
  node.get_parameter("targets.wait_center_waypoint_file", config.targets.wait_center_waypoint_file);
  node.get_parameter("targets.home_waypoint_file", config.targets.home_waypoint_file);
  node.get_parameter("targets.wait_home_waypoint_file", config.targets.wait_home_waypoint_file);
  node.get_parameter("targets.wait_hp_waypoint_file", config.targets.wait_hp_waypoint_file);

  node.get_parameter("waypoint.switch_distance", config.waypoint.switch_distance);
  node.get_parameter("waypoint.final_goal_tolerance", config.waypoint.final_goal_tolerance);

  node.get_parameter("maintain_goal.enable", config.maintain_goal.enable);
  node.get_parameter("maintain_goal.robot_base_frame", config.maintain_goal.robot_base_frame);
  node.get_parameter("maintain_goal.xy_tolerance", config.maintain_goal.xy_tolerance);
  node.get_parameter("maintain_goal.drift_hold_sec", config.maintain_goal.drift_hold_sec);

  node.get_parameter("combat.enable", config.combat.enable);
  node.get_parameter("combat.firing_heat_threshold", config.combat.firing_heat_threshold);
  node.get_parameter("combat.engage_hold_sec", config.combat.engage_hold_sec);
  node.get_parameter("combat.reposition_hold_sec", config.combat.reposition_hold_sec);
  node.get_parameter("combat.reposition_path_len", config.combat.reposition_path_len);
  node.get_parameter("combat.reposition_grace_sec", config.combat.reposition_grace_sec);

  validateDecisionConfig(config);
  return config;
}

void validateDecisionConfig(DecisionConfig & config)
{
  config.loop_hz = std::max(config.loop_hz, 0.1);
  config.retry_interval_sec = std::max(config.retry_interval_sec, 0.1);
  // 负值视为关闭：比抛配置错误更宽容，语义也明确。
  config.executor_result_timeout_sec = std::max(config.executor_result_timeout_sec, 0.0);
  config.waypoint.switch_distance = std::max(config.waypoint.switch_distance, 0.05);
  config.waypoint.final_goal_tolerance = std::max(config.waypoint.final_goal_tolerance, 0.05);
  config.maintain_goal.xy_tolerance = std::max(config.maintain_goal.xy_tolerance, 0.05);
  config.maintain_goal.drift_hold_sec = std::max(config.maintain_goal.drift_hold_sec, 0.0);
  if (config.hp_recovery.high_threshold < config.hp_recovery.low_threshold) {
    std::swap(config.hp_recovery.high_threshold, config.hp_recovery.low_threshold);
  }

  config.combat.engage_hold_sec = std::max(config.combat.engage_hold_sec, 0.0);
  config.combat.reposition_hold_sec = std::max(config.combat.reposition_hold_sec, 0.0);
  config.combat.reposition_path_len = std::max(config.combat.reposition_path_len, 1);
  config.combat.reposition_grace_sec = std::max(config.combat.reposition_grace_sec, 0.0);
}

}  // namespace decision
