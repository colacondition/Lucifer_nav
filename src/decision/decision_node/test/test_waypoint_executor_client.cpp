#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "decision/waypoint_executor_client.hpp"

TEST(WaypointExecutorClient, HomeAndWaitHomePreemptImmediately)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);

  EXPECT_TRUE(client.shouldPreemptImmediately(decision::TargetName::Home));
  EXPECT_TRUE(client.shouldPreemptImmediately(decision::TargetName::WaitHome));
  EXPECT_FALSE(client.shouldPreemptImmediately(decision::TargetName::Center));
  EXPECT_FALSE(client.shouldPreemptImmediately(decision::TargetName::Patrol));
}

TEST(WaypointExecutorClient, CompletedStatusMarksSuccess)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 10.0);

  auto state = client.onExecutorStatus(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    "COMPLETED",
    11.0);

  EXPECT_EQ(state.result_status, decision::ExecutorResultStatus::Succeeded);
  EXPECT_FALSE(state.running_target.has_value());
}

TEST(WaypointExecutorClient, AbortedStatusMarksFailure)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 10.0);

  auto state = client.onExecutorStatus(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    "ABORTED",
    11.0);

  EXPECT_EQ(state.result_status, decision::ExecutorResultStatus::Aborted);
  EXPECT_FALSE(state.running_target.has_value());
}

TEST(WaypointExecutorClient, CompletedTargetDoesNotNeedResend)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(decision::TargetName::Home, decision::TargetMode::ExecutorFollow, 10.0);

  auto state = client.onExecutorStatus(
    decision::TargetName::Home,
    decision::TargetMode::ExecutorFollow,
    "COMPLETED",
    11.0);

  EXPECT_EQ(state.active_target, decision::TargetName::Home);
  EXPECT_EQ(state.result_status, decision::ExecutorResultStatus::Succeeded);
  EXPECT_FALSE(state.running_target.has_value());
}

TEST(WaypointExecutorClient, MaintainReissueUsesDistanceAndHoldTime)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  config.maintain_goal.xy_tolerance = 0.35;
  config.maintain_goal.drift_hold_sec = 0.8;
  decision::WaypointExecutorClient client(config);

  decision::Pose robot;
  robot.x = 2.0;
  robot.y = 0.0;
  decision::Pose target;
  target.x = 0.0;
  target.y = 0.0;

  EXPECT_FALSE(client.needMaintainReissue(decision::TargetName::WaitCenter, robot, target, 1.0));
  EXPECT_TRUE(client.needMaintainReissue(decision::TargetName::WaitCenter, robot, target, 2.0));
  EXPECT_FALSE(client.needMaintainReissue(decision::TargetName::WaitCenter, robot, target, 2.1));
  EXPECT_TRUE(client.needMaintainReissue(decision::TargetName::WaitCenter, robot, target, 2.9));
}

TEST(WaypointExecutorClient, MaintainsOnlyWaitTargets)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  config.maintain_goal.xy_tolerance = 0.35;
  config.maintain_goal.drift_hold_sec = 0.8;

  decision::Pose robot;
  robot.x = 2.0;
  decision::Pose anchor;

  for (const auto target : {
      decision::TargetName::WaitHome,
      decision::TargetName::WaitHp,
      decision::TargetName::WaitCenter})
  {
    decision::WaypointExecutorClient client(config);
    EXPECT_FALSE(client.needMaintainReissue(target, robot, anchor, 1.0));
    EXPECT_TRUE(client.needMaintainReissue(target, robot, anchor, 2.0));
  }

  for (const auto target : {
      decision::TargetName::Home,
      decision::TargetName::Center,
      decision::TargetName::Patrol})
  {
    decision::WaypointExecutorClient client(config);
    EXPECT_FALSE(client.needMaintainReissue(target, robot, anchor, 1.0));
    EXPECT_FALSE(client.needMaintainReissue(target, robot, anchor, 2.0));
  }
}

TEST(WaypointExecutorClient, CompletedWaitTargetRequestsAgainAfterDrift)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  config.maintain_goal.xy_tolerance = 0.35;
  config.maintain_goal.drift_hold_sec = 0.8;
  decision::WaypointExecutorClient client(config);

  client.setRunningTarget(
    decision::TargetName::WaitHp, decision::TargetMode::ExecutorFollow, 10.0);
  client.onExecutorStatus(
    decision::TargetName::WaitHp,
    decision::TargetMode::ExecutorFollow,
    "COMPLETED",
    11.0);

  decision::Pose robot;
  robot.x = 2.0;
  decision::Pose anchor;

  EXPECT_FALSE(client.shouldRequestTarget(
    decision::TargetName::WaitHp,
    decision::TargetMode::ExecutorFollow,
    robot,
    anchor,
    11.1));
  EXPECT_TRUE(client.shouldRequestTarget(
    decision::TargetName::WaitHp,
    decision::TargetMode::ExecutorFollow,
    robot,
    anchor,
    12.0));
}

TEST(WaypointExecutorClient, CompletedMoveTargetDoesNotMaintain)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  decision::WaypointExecutorClient client(config);

  client.setRunningTarget(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 10.0);
  client.onExecutorStatus(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    "COMPLETED",
    11.0);

  decision::Pose robot;
  robot.x = 2.0;
  decision::Pose anchor;

  EXPECT_FALSE(client.shouldRequestTarget(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    robot,
    anchor,
    20.0));
}

TEST(WaypointExecutorClient, CompletedWaitTargetDoesNotRestartWhenPoseIsUnavailable)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  decision::WaypointExecutorClient client(config);

  client.setRunningTarget(
    decision::TargetName::WaitHome, decision::TargetMode::ExecutorFollow, 10.0);
  client.onExecutorStatus(
    decision::TargetName::WaitHome,
    decision::TargetMode::ExecutorFollow,
    "COMPLETED",
    11.0);

  EXPECT_FALSE(client.shouldRequestTarget(
    decision::TargetName::WaitHome,
    decision::TargetMode::ExecutorFollow,
    std::nullopt,
    std::nullopt,
    14.0));
  EXPECT_FALSE(client.shouldRequestTarget(
    decision::TargetName::WaitHome,
    decision::TargetMode::ExecutorFollow,
    std::nullopt,
    std::nullopt,
    1000.0));
}

TEST(WaypointExecutorClient, DispatchPublishesWaypointsThenStartsServiceAfterDelay)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_dispatch_test");
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const auto waypoints_topic = "executor_waypoints_" + suffix;
  const auto saved_file_topic = "saved_waypoint_file_" + suffix;
  const auto follow_service_name = "start_waypoint_following_" + suffix;
  const auto through_service_name = "start_waypoint_through_" + suffix;

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();

  std::vector<nav_msgs::msg::Path> received_paths;
  std::vector<std_msgs::msg::String> received_files;
  int service_calls = 0;

  auto path_sub = node->create_subscription<nav_msgs::msg::Path>(
    waypoints_topic, latched_qos,
    [&received_paths](const nav_msgs::msg::Path & msg) {
      received_paths.push_back(msg);
    });
  auto file_sub = node->create_subscription<std_msgs::msg::String>(
    saved_file_topic, latched_qos,
    [&received_files](const std_msgs::msg::String & msg) {
      received_files.push_back(msg);
    });
  auto direct_goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "direct_goal_" + suffix, 10);
  auto follow_service = node->create_service<std_srvs::srv::Trigger>(
    follow_service_name,
    [&service_calls](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++service_calls;
      response->success = true;
      response->message = "started";
    });

  decision::DecisionConfig config;
  config.goal_frame_id = "map";
  config.waypoint_executor.start_delay_sec = 0.2;
  decision::WaypointExecutorClient client(config);
  client.setRosInterfaces(
    node->create_publisher<nav_msgs::msg::Path>(waypoints_topic, latched_qos),
    node->create_publisher<std_msgs::msg::String>(saved_file_topic, latched_qos),
    direct_goal_pub,
    node->create_client<std_srvs::srv::Trigger>(follow_service_name),
    node->create_client<std_srvs::srv::Trigger>(through_service_name));

  decision::Pose first;
  first.x = 1.0;
  first.y = 2.0;
  first.qw = 1.0;
  decision::Pose second;
  second.x = 3.0;
  second.y = 4.0;
  second.qw = 1.0;

  EXPECT_FALSE(client.stepExecutorTarget(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    "/tmp/center.csv",
    {first, second},
    false,
    10.0));

  rclcpp::spin_some(node);
  ASSERT_EQ(received_paths.size(), 1U);
  EXPECT_EQ(received_paths.front().header.frame_id, "map");
  ASSERT_EQ(received_paths.front().poses.size(), 2U);
  EXPECT_DOUBLE_EQ(received_paths.front().poses[1].pose.position.x, 3.0);
  ASSERT_EQ(received_files.size(), 1U);
  EXPECT_EQ(received_files.front().data, "/tmp/center.csv");
  EXPECT_EQ(service_calls, 0);

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    "/tmp/center.csv",
    {first, second},
    false,
    10.3));

  for (int i = 0; i < 20 && !client.state().running_target.has_value(); ++i) {
    rclcpp::spin_some(node);
  }

  EXPECT_EQ(service_calls, 1);
  EXPECT_EQ(client.state().running_target, decision::TargetName::Center);
  EXPECT_EQ(client.state().running_mode, decision::TargetMode::ExecutorFollow);
}

TEST(WaypointExecutorClient, ImmediatePreemptPublishesGoalPose)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_preempt_test");
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const auto waypoints_topic = "executor_waypoints_preempt_" + suffix;
  const auto saved_file_topic = "saved_waypoint_file_preempt_" + suffix;
  const auto direct_goal_topic = "direct_goal_preempt_" + suffix;
  const auto follow_service_name = "start_waypoint_following_preempt_" + suffix;
  const auto through_service_name = "start_waypoint_through_preempt_" + suffix;

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();

  std::vector<geometry_msgs::msg::PoseStamped> received_direct_goals;
  int service_calls = 0;

  auto direct_goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    direct_goal_topic, 10,
    [&received_direct_goals](const geometry_msgs::msg::PoseStamped & msg) {
      received_direct_goals.push_back(msg);
    });
  auto follow_service = node->create_service<std_srvs::srv::Trigger>(
    follow_service_name,
    [&service_calls](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++service_calls;
      response->success = true;
      response->message = "started";
    });

  decision::DecisionConfig config;
  config.goal_frame_id = "map";
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 9.0);
  client.setRosInterfaces(
    node->create_publisher<nav_msgs::msg::Path>(waypoints_topic, latched_qos),
    node->create_publisher<std_msgs::msg::String>(saved_file_topic, latched_qos),
    node->create_publisher<geometry_msgs::msg::PoseStamped>(direct_goal_topic, 10),
    node->create_client<std_srvs::srv::Trigger>(follow_service_name),
    node->create_client<std_srvs::srv::Trigger>(through_service_name));

  decision::Pose first;
  first.x = 7.0;
  first.y = 8.0;
  first.qw = 1.0;

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Home,
    decision::TargetMode::ExecutorFollow,
    "/tmp/home.csv",
    {first},
    true,
    10.0));

  for (int i = 0; i < 20 && (received_direct_goals.empty() || service_calls == 0); ++i) {
    rclcpp::spin_some(node);
  }

  ASSERT_EQ(received_direct_goals.size(), 1U);
  EXPECT_DOUBLE_EQ(received_direct_goals.front().pose.position.x, 7.0);
  EXPECT_DOUBLE_EQ(received_direct_goals.front().pose.position.y, 8.0);
  EXPECT_EQ(service_calls, 1);
}

TEST(WaypointExecutorClient, ImmediateRequestsCoalesceWhileServiceRequestIsInFlight)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_coalesce_test");
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const auto waypoints_topic = "executor_waypoints_coalesce_" + suffix;
  const auto saved_file_topic = "saved_waypoint_file_coalesce_" + suffix;
  const auto direct_goal_topic = "direct_goal_coalesce_" + suffix;
  const auto follow_service_name = "start_waypoint_following_coalesce_" + suffix;
  const auto through_service_name = "start_waypoint_through_coalesce_" + suffix;

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();

  std::vector<geometry_msgs::msg::PoseStamped> received_direct_goals;
  int service_calls = 0;

  auto direct_goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    direct_goal_topic, 10,
    [&received_direct_goals](const geometry_msgs::msg::PoseStamped & msg) {
      received_direct_goals.push_back(msg);
    });
  auto follow_service = node->create_service<std_srvs::srv::Trigger>(
    follow_service_name,
    [&service_calls](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++service_calls;
      response->success = true;
      response->message = "started";
    });

  decision::DecisionConfig config;
  config.goal_frame_id = "map";
  decision::WaypointExecutorClient client(config);
  client.setRosInterfaces(
    node->create_publisher<nav_msgs::msg::Path>(waypoints_topic, latched_qos),
    node->create_publisher<std_msgs::msg::String>(saved_file_topic, latched_qos),
    node->create_publisher<geometry_msgs::msg::PoseStamped>(direct_goal_topic, 10),
    node->create_client<std_srvs::srv::Trigger>(follow_service_name),
    node->create_client<std_srvs::srv::Trigger>(through_service_name));

  decision::Pose home;
  home.x = 1.0;
  home.qw = 1.0;
  decision::Pose wait_home;
  wait_home.x = 2.0;
  wait_home.qw = 1.0;

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Home,
    decision::TargetMode::ExecutorFollow,
    "/tmp/home.csv", {home}, true, 10.0));
  EXPECT_FALSE(client.stepExecutorTarget(
    decision::TargetName::Home,
    decision::TargetMode::ExecutorFollow,
    "/tmp/home.csv", {home}, true, 10.1));
  EXPECT_FALSE(client.stepExecutorTarget(
    decision::TargetName::Home,
    decision::TargetMode::ExecutorFollow,
    "/tmp/home.csv", {home}, true, 10.2));
  EXPECT_FALSE(client.stepExecutorTarget(
    decision::TargetName::WaitHome,
    decision::TargetMode::ExecutorFollow,
    "/tmp/wait_home.csv", {wait_home}, true, 10.3));

  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node);
  }

  ASSERT_EQ(received_direct_goals.size(), 2U);
  EXPECT_DOUBLE_EQ(received_direct_goals[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(received_direct_goals[1].pose.position.x, 2.0);
  EXPECT_EQ(service_calls, 1);
  EXPECT_FALSE(client.state().running_target.has_value());

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::WaitHome,
    decision::TargetMode::ExecutorFollow,
    "/tmp/wait_home.csv", {wait_home}, true, 10.4));
  for (int i = 0; i < 20 && !client.state().running_target.has_value(); ++i) {
    rclcpp::spin_some(node);
  }

  EXPECT_EQ(received_direct_goals.size(), 2U);
  EXPECT_EQ(service_calls, 2);
  EXPECT_EQ(client.state().running_target, decision::TargetName::WaitHome);
}

TEST(WaypointExecutorClient, TerminalStatusBeforeStartResponseSettlesInFlightGeneration)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  const auto verify_terminal = [](
      const std::string & status,
      decision::ExecutorResultStatus expected_result,
      decision::TargetName target) {
      SCOPED_TRACE(status);

      const auto suffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
      auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_terminal_" + suffix);
      const auto waypoints_topic = "executor_waypoints_terminal_" + suffix;
      const auto saved_file_topic = "saved_waypoint_file_terminal_" + suffix;
      const auto follow_service_name = "start_waypoint_following_terminal_" + suffix;
      const auto through_service_name = "start_waypoint_through_terminal_" + suffix;

      rclcpp::QoS latched_qos(1);
      latched_qos.reliable().transient_local();

      int service_calls = 0;
      auto follow_service = node->create_service<std_srvs::srv::Trigger>(
        follow_service_name,
        [&service_calls](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ++service_calls;
          response->success = true;
          response->message = "started";
        });

      decision::DecisionConfig config;
      decision::WaypointExecutorClient client(config);
      client.setRosInterfaces(
        node->create_publisher<nav_msgs::msg::Path>(waypoints_topic, latched_qos),
        node->create_publisher<std_msgs::msg::String>(saved_file_topic, latched_qos),
        node->create_publisher<geometry_msgs::msg::PoseStamped>("direct_goal_" + suffix, 10),
        node->create_client<std_srvs::srv::Trigger>(follow_service_name),
        node->create_client<std_srvs::srv::Trigger>(through_service_name));

      decision::Pose waypoint;
      waypoint.x = 1.0;
      waypoint.qw = 1.0;
      ASSERT_TRUE(client.stepExecutorTarget(
        target, decision::TargetMode::ExecutorFollow,
        "/tmp/terminal.csv", {waypoint}, true, 10.0));

      const auto status_target = client.targetForStatus(
        decision::TargetMode::ExecutorFollow, status);
      ASSERT_EQ(status_target, target);
      const auto terminal_state = client.onExecutorStatus(
        *status_target, decision::TargetMode::ExecutorFollow, status, 11.0);
      EXPECT_EQ(terminal_state.result_status, expected_result);
      EXPECT_FALSE(terminal_state.running_target.has_value());

      for (int i = 0; i < 20; ++i) {
        rclcpp::spin_some(node);
      }

      EXPECT_EQ(service_calls, 1);
      EXPECT_EQ(client.state().result_status, expected_result);
      EXPECT_FALSE(client.state().running_target.has_value());
    };

  verify_terminal(
    "COMPLETED", decision::ExecutorResultStatus::Succeeded, decision::TargetName::Home);
  verify_terminal(
    "ABORTED", decision::ExecutorResultStatus::Aborted, decision::TargetName::WaitHome);
}

TEST(WaypointExecutorClient, RunningHandoffAssociatesTerminalWithInFlightTarget)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_handoff_" + suffix);
  const auto waypoints_topic = "executor_waypoints_handoff_" + suffix;
  const auto saved_file_topic = "saved_waypoint_file_handoff_" + suffix;
  const auto follow_service_name = "start_waypoint_following_handoff_" + suffix;
  const auto through_service_name = "start_waypoint_through_handoff_" + suffix;

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();

  int service_calls = 0;
  auto follow_service = node->create_service<std_srvs::srv::Trigger>(
    follow_service_name,
    [&service_calls](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      ++service_calls;
      response->success = true;
      response->message = "started";
    });

  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 9.0);
  client.setRosInterfaces(
    node->create_publisher<nav_msgs::msg::Path>(waypoints_topic, latched_qos),
    node->create_publisher<std_msgs::msg::String>(saved_file_topic, latched_qos),
    node->create_publisher<geometry_msgs::msg::PoseStamped>("direct_goal_" + suffix, 10),
    node->create_client<std_srvs::srv::Trigger>(follow_service_name),
    node->create_client<std_srvs::srv::Trigger>(through_service_name));

  decision::Pose home;
  home.x = 1.0;
  home.qw = 1.0;
  ASSERT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Home, decision::TargetMode::ExecutorFollow,
    "/tmp/home.csv", {home}, true, 10.0));

  EXPECT_EQ(
    client.targetForStatus(decision::TargetMode::ExecutorFollow, "COMPLETED"),
    decision::TargetName::Center);

  const auto running_target = client.targetForStatus(
    decision::TargetMode::ExecutorFollow, "RUNNING");
  ASSERT_EQ(running_target, decision::TargetName::Home);
  client.onExecutorStatus(
    *running_target, decision::TargetMode::ExecutorFollow, "RUNNING", 10.1);

  const auto completed_target = client.targetForStatus(
    decision::TargetMode::ExecutorFollow, "COMPLETED");
  ASSERT_EQ(completed_target, decision::TargetName::Home);
  const auto completed = client.onExecutorStatus(
    *completed_target, decision::TargetMode::ExecutorFollow, "COMPLETED", 10.2);
  EXPECT_EQ(completed.result_status, decision::ExecutorResultStatus::Succeeded);
  EXPECT_FALSE(completed.running_target.has_value());

  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node);
  }

  EXPECT_EQ(service_calls, 1);
  EXPECT_EQ(client.state().result_status, decision::ExecutorResultStatus::Succeeded);
  EXPECT_FALSE(client.state().running_target.has_value());
}
