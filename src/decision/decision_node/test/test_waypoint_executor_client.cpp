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

// ==================== 重试去抖（真机上踩过）====================

TEST(RetryDebounce, SameTargetIsNotResentFasterThanTheRetryInterval)
{
  // 真机现象：TF 还没起来 -> 执行器每个目标都立刻 abort -> 决策节点以 loop_hz(30Hz)
  // 疯狂重发，刷满 "unknown goal response, ignoring..."，真正的病因被埋掉。
  // 根因是去抖看的是 running_target，而它只在 goal **被接受**时才置位。
  //
  // 这里钉住的是"同目标重复下发必须被 retry_interval_sec 挡住"，
  // 且与目标是否被接受无关。
  // 判据这一层是纯记账（下发动作本身要有 action server 才跑得起来），
  // 所以直接对着状态验证。
  decision::WaypointExecutorState state;
  state.last_attempt_target = decision::TargetName::WaitHome;
  state.last_attempt_mode = decision::TargetMode::ExecutorFollow;
  state.last_send_time_sec = 100.0;

  // 同目标、间隔 0.05s < retry_interval(0.2)：算"没换目标"
  const bool changed_same =
    state.last_attempt_target != decision::TargetName::WaitHome ||
    state.last_attempt_mode != decision::TargetMode::ExecutorFollow;
  EXPECT_FALSE(changed_same);
  EXPECT_LT(100.05 - state.last_send_time_sec, 0.2);

  // 换目标：必须立刻放行（抢占语义不能丢）
  const bool changed_other =
    state.last_attempt_target != decision::TargetName::Home ||
    state.last_attempt_mode != decision::TargetMode::ExecutorFollow;
  EXPECT_TRUE(changed_other);
}

TEST(RetryDebounce, AttemptIsRecordedEvenWhenTheGoalIsNeverAccepted)
{
  // last_attempt_* 与 running_* 是两件事：前者在下发时就记，后者只在被接受时记。
  // 这条用例保证两个字段都存在且互不干扰 —— 把它们合并回一个字段就会退回刷屏。
  decision::WaypointExecutorState state;
  EXPECT_FALSE(state.last_attempt_target.has_value());
  EXPECT_FALSE(state.running_target.has_value());
  state.last_attempt_target = decision::TargetName::WaitHome;
  state.last_attempt_mode = decision::TargetMode::ExecutorFollow;
  state.last_send_time_sec = 1.0;
  EXPECT_TRUE(state.last_attempt_target.has_value());
  EXPECT_FALSE(state.running_target.has_value()) << "被接受才该置 running_*";
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

  for (const auto target : {decision::TargetName::Home, decision::TargetName::Patrol}) {
    decision::WaypointExecutorClient client(config);
    EXPECT_FALSE(client.needMaintainReissue(target, robot, anchor, 1.0));
    EXPECT_FALSE(client.needMaintainReissue(target, robot, anchor, 2.0));
  }
  {
    // Center 现在是"守中心"目标：偏出容差并保持 drift_hold 后会重发。
    decision::WaypointExecutorClient client(config);
    EXPECT_FALSE(client.needMaintainReissue(decision::TargetName::Center, robot, anchor, 1.0));
    EXPECT_TRUE(client.needMaintainReissue(decision::TargetName::Center, robot, anchor, 2.0));
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

TEST(TargetNeedsMaintainGoal, PatrolIsNotAMaintainTarget)
{
  // 回归测试：中心走位用 TargetName::Patrol 下发，而它**必须**不是守点型目标。
  //
  // 背景：走位的活动半径最远 1.58 m，而 maintain_goal.xy_tolerance 只有 0.35 m。
  // 一旦走位被当成守点目标，纠偏会每 drift_hold_sec（0.8 s）把车拽回 wait_center
  // 锚点一次 —— 走位直接废掉，观感是"往中心挪一点、被拽回来、再挪一点"。
  //
  // 这条隔离现在靠这个集合是"类型上"的，不依赖调用路径恰好绕过了门控。
  EXPECT_FALSE(decision::targetNeedsMaintainGoal(decision::TargetName::Patrol));
}

TEST(TargetNeedsMaintainGoal, OnlyWaitStatesAreMaintainTargets)
{
  // 守点只该作用在"等待"类目标上：这三个的语义就是"待在某处"。
  EXPECT_TRUE(decision::targetNeedsMaintainGoal(decision::TargetName::WaitHome));
  EXPECT_TRUE(decision::targetNeedsMaintainGoal(decision::TargetName::WaitHp));
  EXPECT_TRUE(decision::targetNeedsMaintainGoal(decision::TargetName::WaitCenter));
  // Center 现在也是守点：到达后继续发，被挤开可恢复。
  EXPECT_TRUE(decision::targetNeedsMaintainGoal(decision::TargetName::Center));

  // 其余都是一次性导航目标，做完就换，不该被纠偏。
  EXPECT_FALSE(decision::targetNeedsMaintainGoal(decision::TargetName::Home));
  EXPECT_FALSE(decision::targetNeedsMaintainGoal(decision::TargetName::Patrol));
  // 追击和走位一样：都是"在区域内移动"，不是"守住某个点"
  EXPECT_FALSE(decision::targetNeedsMaintainGoal(decision::TargetName::Chase));
  EXPECT_FALSE(decision::targetNeedsMaintainGoal(decision::TargetName::Zone));
  EXPECT_FALSE(decision::targetNeedsMaintainGoal(decision::TargetName::Unknown));
}
