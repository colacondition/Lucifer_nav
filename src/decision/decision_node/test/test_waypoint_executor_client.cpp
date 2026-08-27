#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <decision_interfaces/action/follow_waypoints.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "decision/waypoint_executor_client.hpp"

namespace
{

using FollowWaypoints = decision_interfaces::action::FollowWaypoints;

void initRclcppIfNeeded()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

decision::Pose makePose(double x, double y)
{
  decision::Pose pose;
  pose.x = x;
  pose.y = y;
  pose.qw = 1.0;
  return pose;
}

// 假执行器：记录收到的目标；按行为立即成功/失败，或挂起不返回结果。
struct MockExecutor
{
  enum class Behavior
  {
    Succeed,
    Abort,
    Hang,
  };

  std::vector<FollowWaypoints::Goal> goals;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowWaypoints>> last_handle;
  std::shared_ptr<rclcpp_action::Server<FollowWaypoints>> server;
  Behavior behavior{Behavior::Succeed};

  explicit MockExecutor(
    rclcpp::Node::SharedPtr node, const std::string & action_name)
  {
    server = rclcpp_action::create_server<FollowWaypoints>(
      node,
      action_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowWaypoints::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowWaypoints>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowWaypoints>> handle) {
        goals.push_back(handle->get_goal() ? *handle->get_goal() : FollowWaypoints::Goal());
        last_handle = handle;
        if (behavior == Behavior::Hang) {
          return;
        }
        auto result = std::make_shared<FollowWaypoints::Result>();
        result->success = (behavior == Behavior::Succeed);
        result->message = result->success ? "completed" : "aborted";
        if (result->success) {
          handle->succeed(result);
        } else {
          handle->abort(result);
        }
      });
  }
};

void spinUntil(
  const rclcpp::Node::SharedPtr & node,
  const std::function<bool()> & predicate,
  int max_iterations = 200)
{
  for (int i = 0; i < max_iterations && !predicate(); ++i) {
    rclcpp::spin_some(node);
  }
}

}  // namespace

TEST(WaypointExecutorClient, HomeAndWaitHomePreemptImmediately)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);

  EXPECT_TRUE(client.shouldPreemptImmediately(decision::TargetName::Home));
  EXPECT_TRUE(client.shouldPreemptImmediately(decision::TargetName::WaitHome));
  EXPECT_FALSE(client.shouldPreemptImmediately(decision::TargetName::Center));
  EXPECT_FALSE(client.shouldPreemptImmediately(decision::TargetName::Patrol));
}

TEST(WaypointExecutorClient, ApplyResultMarksSuccessOrAborted)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 10.0);

  auto succeeded = client.applyExecutorResult(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, true, 11.0);
  EXPECT_EQ(succeeded.result_status, decision::ExecutorResultStatus::Succeeded);
  EXPECT_FALSE(succeeded.running_target.has_value());
  EXPECT_NEAR(succeeded.last_goal_success_time_sec, 11.0, 1e-9);

  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 12.0);
  auto aborted = client.applyExecutorResult(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, false, 13.0);
  EXPECT_EQ(aborted.result_status, decision::ExecutorResultStatus::Aborted);
  EXPECT_FALSE(aborted.running_target.has_value());
}

TEST(WaypointExecutorClient, ApplyResultIgnoresMismatchedTarget)
{
  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 10.0);

  // 别的目标晚到的结果不能覆盖当前状态。
  auto state = client.applyExecutorResult(
    decision::TargetName::Home, decision::TargetMode::ExecutorFollow, true, 11.0);
  EXPECT_EQ(state.result_status, decision::ExecutorResultStatus::None);
  EXPECT_EQ(state.running_target, decision::TargetName::Center);
}

TEST(WaypointExecutorClient, MaintainReissueUsesDistanceAndHoldTime)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  config.maintain_goal.xy_tolerance = 0.35;
  config.maintain_goal.drift_hold_sec = 0.8;
  decision::WaypointExecutorClient client(config);

  const auto robot = makePose(2.0, 0.0);
  const auto target = makePose(0.0, 0.0);

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

  const auto robot = makePose(2.0, 0.0);
  const auto anchor = makePose(0.0, 0.0);

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
  client.applyExecutorResult(
    decision::TargetName::WaitHp, decision::TargetMode::ExecutorFollow, true, 11.0);

  const auto robot = makePose(2.0, 0.0);
  const auto anchor = makePose(0.0, 0.0);

  EXPECT_FALSE(client.shouldRequestTarget(
    decision::TargetName::WaitHp, decision::TargetMode::ExecutorFollow, robot, anchor, 11.1));
  EXPECT_TRUE(client.shouldRequestTarget(
    decision::TargetName::WaitHp, decision::TargetMode::ExecutorFollow, robot, anchor, 12.0));
}

TEST(WaypointExecutorClient, CompletedMoveTargetDoesNotMaintain)
{
  decision::DecisionConfig config;
  config.maintain_goal.enable = true;
  decision::WaypointExecutorClient client(config);

  client.setRunningTarget(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 10.0);
  client.applyExecutorResult(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, true, 11.0);

  const auto robot = makePose(2.0, 0.0);
  const auto anchor = makePose(0.0, 0.0);

  EXPECT_FALSE(client.shouldRequestTarget(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow, robot, anchor, 20.0));
}

TEST(WaypointExecutorClient, DispatchSendsActionGoalAndPublishesForPanel)
{
  initRclcppIfNeeded();
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_dispatch_" + suffix);
  const auto follow_action = "follow_waypoints_dispatch_" + suffix;
  const auto through_action = "through_waypoints_dispatch_" + suffix;
  const auto waypoints_topic = "executor_waypoints_dispatch_" + suffix;
  const auto saved_file_topic = "saved_waypoint_file_dispatch_" + suffix;

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();
  MockExecutor mock(node, follow_action);
  mock.behavior = MockExecutor::Behavior::Succeed;

  std::vector<nav_msgs::msg::Path> received_paths;
  std::vector<std_msgs::msg::String> received_files;
  auto path_sub = node->create_subscription<nav_msgs::msg::Path>(
    waypoints_topic, latched_qos,
    [&received_paths](const nav_msgs::msg::Path & msg) { received_paths.push_back(msg); });
  auto file_sub = node->create_subscription<std_msgs::msg::String>(
    saved_file_topic, latched_qos,
    [&received_files](const std_msgs::msg::String & msg) { received_files.push_back(msg); });

  decision::DecisionConfig config;
  config.goal_frame_id = "map";
  decision::WaypointExecutorClient client(config);
  client.setRosInterfaces(
    node->create_publisher<nav_msgs::msg::Path>(waypoints_topic, latched_qos),
    node->create_publisher<std_msgs::msg::String>(saved_file_topic, latched_qos),
    nullptr,
    rclcpp_action::create_client<FollowWaypoints>(node, follow_action),
    rclcpp_action::create_client<FollowWaypoints>(node, through_action));

  // 先 spin 让 action server 就绪。
  spinUntil(node, [&mock]() { return mock.server != nullptr; }, 10);
  spinUntil(
    node,
    [&]() {
      return rclcpp_action::create_client<FollowWaypoints>(node, follow_action)->action_server_is_ready();
    },
    10);

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Center,
    decision::TargetMode::ExecutorFollow,
    "/tmp/center.csv",
    {makePose(1.0, 2.0), makePose(3.0, 4.0)},
    false,
    10.0));

  spinUntil(node, [&mock]() { return !mock.goals.empty(); });
  ASSERT_EQ(mock.goals.size(), 1U);
  EXPECT_EQ(mock.goals.front().waypoints.header.frame_id, "map");
  ASSERT_EQ(mock.goals.front().waypoints.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(mock.goals.front().waypoints.poses[1].pose.position.x, 3.0);

  // 结果送达后状态应转为成功。
  spinUntil(
    node,
    [&]() { return client.state().result_status == decision::ExecutorResultStatus::Succeeded; });
  EXPECT_FALSE(client.state().running_target.has_value());

  // 面板兼容发布。
  rclcpp::spin_some(node);
  ASSERT_EQ(received_paths.size(), 1U);
  ASSERT_EQ(received_files.size(), 1U);
  EXPECT_EQ(received_files.front().data, "/tmp/center.csv");
}

TEST(WaypointExecutorClient, ImmediatePreemptPublishesDirectGoal)
{
  initRclcppIfNeeded();
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_preempt_" + suffix);
  const auto follow_action = "follow_waypoints_preempt_" + suffix;
  const auto through_action = "through_waypoints_preempt_" + suffix;
  const auto direct_goal_topic = "direct_goal_preempt_" + suffix;

  MockExecutor mock(node, follow_action);
  mock.behavior = MockExecutor::Behavior::Succeed;

  std::vector<geometry_msgs::msg::PoseStamped> received_direct_goals;
  auto direct_goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    direct_goal_topic, 10,
    [&received_direct_goals](const geometry_msgs::msg::PoseStamped & msg) {
      received_direct_goals.push_back(msg);
    });

  decision::DecisionConfig config;
  config.goal_frame_id = "map";
  decision::WaypointExecutorClient client(config);
  client.setRosInterfaces(
    nullptr,
    nullptr,
    node->create_publisher<geometry_msgs::msg::PoseStamped>(direct_goal_topic, 10),
    rclcpp_action::create_client<FollowWaypoints>(node, follow_action),
    rclcpp_action::create_client<FollowWaypoints>(node, through_action));

  spinUntil(node, [&mock]() { return mock.server != nullptr; }, 10);

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Home,
    decision::TargetMode::ExecutorFollow,
    "/tmp/home.csv",
    {makePose(7.0, 8.0)},
    true,
    10.0));

  spinUntil(node, [&received_direct_goals]() { return !received_direct_goals.empty(); });
  ASSERT_EQ(received_direct_goals.size(), 1U);
  EXPECT_DOUBLE_EQ(received_direct_goals.front().pose.position.x, 7.0);
  EXPECT_DOUBLE_EQ(received_direct_goals.front().pose.position.y, 8.0);
}

TEST(WaypointExecutorClient, PreemptedGoalResultDoesNotOverrideNewTarget)
{
  initRclcppIfNeeded();
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_override_" + suffix);
  const auto follow_action = "follow_waypoints_override_" + suffix;
  const auto through_action = "through_waypoints_override_" + suffix;

  MockExecutor mock(node, follow_action);
  mock.behavior = MockExecutor::Behavior::Hang;  // 第一个目标挂起

  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRosInterfaces(
    nullptr, nullptr, nullptr,
    rclcpp_action::create_client<FollowWaypoints>(node, follow_action),
    rclcpp_action::create_client<FollowWaypoints>(node, through_action));

  spinUntil(node, [&mock]() { return mock.server != nullptr; }, 10);

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Home, decision::TargetMode::ExecutorFollow,
    "/tmp/a.csv", {makePose(1.0, 0.0)}, true, 10.0));
  spinUntil(node, [&]() { return client.state().running_target.has_value(); });
  ASSERT_TRUE(client.state().running_target.has_value());

  // 第二个目标抢占；假执行器改为立即成功。
  mock.behavior = MockExecutor::Behavior::Succeed;
  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Center, decision::TargetMode::ExecutorFollow,
    "/tmp/b.csv", {makePose(2.0, 0.0)}, true, 11.0));

  spinUntil(
    node,
    [&]() { return client.state().result_status == decision::ExecutorResultStatus::Succeeded; });
  EXPECT_FALSE(client.state().running_target.has_value());
}

TEST(WaypointExecutorClient, ResultCallbackHookNotifiesTerminalOutcome)
{
  initRclcppIfNeeded();
  const auto suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  auto node = std::make_shared<rclcpp::Node>("waypoint_executor_client_hook_" + suffix);
  const auto follow_action = "follow_waypoints_hook_" + suffix;
  const auto through_action = "through_waypoints_hook_" + suffix;

  MockExecutor mock(node, follow_action);
  mock.behavior = MockExecutor::Behavior::Abort;

  decision::DecisionConfig config;
  decision::WaypointExecutorClient client(config);
  client.setRosInterfaces(
    nullptr, nullptr, nullptr,
    rclcpp_action::create_client<FollowWaypoints>(node, follow_action),
    rclcpp_action::create_client<FollowWaypoints>(node, through_action));

  std::optional<std::pair<decision::TargetName, bool>> notified;
  client.setResultCallback(
    [&notified](decision::TargetName target, bool success) {
      notified = std::make_pair(target, success);
    });

  spinUntil(node, [&mock]() { return mock.server != nullptr; }, 10);

  EXPECT_TRUE(client.stepExecutorTarget(
    decision::TargetName::Patrol, decision::TargetMode::ExecutorFollow,
    "/tmp/p.csv", {makePose(1.0, 0.0)}, true, 10.0));

  spinUntil(node, [&notified]() { return notified.has_value(); });
  ASSERT_TRUE(notified.has_value());
  EXPECT_EQ(notified->first, decision::TargetName::Patrol);
  EXPECT_FALSE(notified->second);
}

TEST(WaypointExecutorClient, TimeoutGuardDisabledByDefault)
{
  decision::DecisionConfig config;
  ASSERT_DOUBLE_EQ(config.executor_result_timeout_sec, 0.0);
  decision::WaypointExecutorClient client(config);

  client.setRunningTarget(decision::TargetName::Patrol, decision::TargetMode::ExecutorFollow, 10.0);
  // 超时关闭时，任意时钟前进都不得合成失败。
  EXPECT_FALSE(client.checkGoalTimeout(1.0e6));
  EXPECT_EQ(client.state().running_target, decision::TargetName::Patrol);
}

// goal 下发后长时间无任何动作事件（executor 崩溃重生的典型后果）：
// 守卫在超过时限后合成 Aborted 并触发 result 回调，状态放行重发。
TEST(WaypointExecutorClient, TimeoutGuardSynthesizesAbortAndAllowsRetry)
{
  decision::DecisionConfig config;
  config.executor_result_timeout_sec = 5.0;
  decision::WaypointExecutorClient client(config);

  std::optional<std::pair<decision::TargetName, bool>> notified;
  client.setResultCallback(
    [&notified](decision::TargetName target, bool success) {
      notified = std::make_pair(target, success);
    });

  // 用 setRunningTarget 直接入「已接受执行」态（绕开 action server 搭建，
  // 该路径的收尾逻辑与真实 result 回调完全一致）。
  client.setRunningTarget(decision::TargetName::Center, decision::TargetMode::ExecutorFollow, 100.0);
  client.markGoalInFlightForTest(100.0);

  EXPECT_FALSE(client.checkGoalTimeout(104.0));   // 未超时：不动状态。
  EXPECT_TRUE(client.state().running_target.has_value());

  EXPECT_TRUE(client.checkGoalTimeout(106.0));    // 超 grace：合成 abort。
  EXPECT_FALSE(client.state().running_target.has_value());
  EXPECT_EQ(client.state().result_status, decision::ExecutorResultStatus::Aborted);
  ASSERT_TRUE(notified.has_value());
  EXPECT_EQ(notified->first, decision::TargetName::Center);
  EXPECT_FALSE(notified->second);

  // 合成终态后 shouldRequestTarget 对同目标重新放行（这正是修复点）。
  EXPECT_TRUE(
    client.shouldRequestTarget(
      decision::TargetName::Center, decision::TargetMode::ExecutorFollow,
      std::nullopt, std::nullopt, 106.0));
}
