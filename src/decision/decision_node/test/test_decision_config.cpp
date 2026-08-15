#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "decision/decision_config.hpp"
#include "decision/types.hpp"

TEST(DecisionConfig, LoadsDefaults)
{
  rclcpp::Node node("config_test_node");
  decision::declareDecisionParameters(node);

  EXPECT_FALSE(node.has_parameter("attacked_evade.enable"));

  auto config = decision::loadDecisionConfig(node);

  EXPECT_EQ(config.topics.robot_status, "robot_status");
  EXPECT_EQ(config.topics.game_status, "game_status");
  EXPECT_EQ(config.topics.goal, "/goal_pose");
  EXPECT_EQ(config.topics.decision_state, "/decision/state");
  EXPECT_EQ(config.waypoint_executor.follow_action, "/waypoint_editor/follow_waypoints");
  EXPECT_EQ(config.waypoint_executor.through_action, "/waypoint_editor/through_waypoints");
  EXPECT_DOUBLE_EQ(config.loop_hz, 10.0);
  EXPECT_EQ(config.hp_recovery.low_threshold, 120);
  EXPECT_EQ(config.hp_recovery.high_threshold, 400);
  EXPECT_TRUE(config.targets.wait_hp_waypoint_file.empty());
  EXPECT_EQ(config.maintain_goal.robot_base_frame, "base_link_fake");
}

TEST(DecisionConfig, ClampsAndSwapsInvalidValues)
{
  decision::DecisionConfig config;
  config.loop_hz = -1.0;
  config.retry_interval_sec = -2.0;
  config.hp_recovery.low_threshold = 400;
  config.hp_recovery.high_threshold = 120;
  config.waypoint.switch_distance = 0.0;
  config.waypoint.final_goal_tolerance = 0.0;

  decision::validateDecisionConfig(config);

  EXPECT_DOUBLE_EQ(config.loop_hz, 0.1);
  EXPECT_DOUBLE_EQ(config.retry_interval_sec, 0.1);
  EXPECT_EQ(config.hp_recovery.low_threshold, 120);
  EXPECT_EQ(config.hp_recovery.high_threshold, 400);
  EXPECT_DOUBLE_EQ(config.waypoint.switch_distance, 0.05);
  EXPECT_DOUBLE_EQ(config.waypoint.final_goal_tolerance, 0.05);
}

TEST(DecisionTypes, ConvertsTargetNames)
{
  EXPECT_EQ(decision::toString(decision::TargetName::WaitHome), "wait_home");
  EXPECT_EQ(decision::toString(decision::TargetName::WaitHp), "wait_hp");
  EXPECT_EQ(decision::toString(decision::TargetName::Home), "home");
  EXPECT_EQ(decision::toString(decision::TargetName::Patrol), "patrol");
  EXPECT_EQ(decision::toString(decision::TargetName::Center), "center");
  EXPECT_EQ(decision::toString(decision::TargetName::WaitCenter), "wait_center");
  EXPECT_EQ(decision::toString(decision::TargetName::Unknown), "unknown");

  EXPECT_EQ(decision::targetNameFromString("wait_home"), decision::TargetName::WaitHome);
  EXPECT_EQ(decision::targetNameFromString("wait_hp"), decision::TargetName::WaitHp);
  EXPECT_EQ(decision::targetNameFromString("home"), decision::TargetName::Home);
  EXPECT_EQ(decision::targetNameFromString("patrol"), decision::TargetName::Patrol);
  EXPECT_EQ(decision::targetNameFromString("center"), decision::TargetName::Center);
  EXPECT_EQ(decision::targetNameFromString("wait_center"), decision::TargetName::WaitCenter);
  EXPECT_EQ(decision::targetNameFromString("unknown"), decision::TargetName::Unknown);
  EXPECT_EQ(decision::targetNameFromString("not_a_target"), decision::TargetName::Unknown);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
