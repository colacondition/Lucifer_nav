#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "decision/decision_config.hpp"
#include "decision/decision_context.hpp"
#include "decision/decision_state_machine.hpp"
#include "decision/types.hpp"
#include "decision/waypoint_executor_client.hpp"
#include "decision/waypoint_store.hpp"
#include "decision_interfaces/msg/game_status.hpp"
#include "decision_interfaces/msg/robot_status.hpp"

namespace
{

constexpr std::size_t kExecutorStatusHistoryDepth = 10;
constexpr double kTfLookupRetrySec = 1.0;

rclcpp::QoS latchedQos()
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  return qos;
}

rclcpp::QoS executorStatusQos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(kExecutorStatusHistoryDepth)};
  qos.reliable().transient_local();
  return qos;
}

}  // namespace

class BtActionReplacementNode : public rclcpp::Node
{
public:
  BtActionReplacementNode()
  : Node("bt_action_replacement")
  {
    decision::declareDecisionParameters(*this);
    config_ = decision::loadDecisionConfig(*this);

    state_machine_ = std::make_unique<decision::DecisionStateMachine>(
      config_.hp_recovery, config_.targets);
    waypoint_store_ = std::make_unique<decision::WaypointStore>(config_);
    waypoint_executor_ = std::make_unique<decision::WaypointExecutorClient>(config_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(config_.topics.goal, 10);
    decision_state_pub_ = create_publisher<std_msgs::msg::String>(
      config_.topics.decision_state, decision::decisionStateQos());
    decision_state_pub_->publish(decision::decisionStateMessage(state_machine_->state()));
    saved_waypoint_file_pub_ = create_publisher<std_msgs::msg::String>(
      config_.waypoint_executor.saved_waypoint_file_topic, latchedQos());
    executor_waypoints_pub_ = create_publisher<nav_msgs::msg::Path>(
      config_.waypoint_executor.executor_waypoints_topic, latchedQos());
    follow_client_ = create_client<std_srvs::srv::Trigger>(
      config_.waypoint_executor.follow_service);
    through_client_ = create_client<std_srvs::srv::Trigger>(
      config_.waypoint_executor.through_service);
    waypoint_executor_->setRosInterfaces(
      executor_waypoints_pub_, saved_waypoint_file_pub_, goal_pub_, follow_client_, through_client_);

    robot_status_sub_ = create_subscription<decision_interfaces::msg::RobotStatus>(
      config_.topics.robot_status, 10,
      [this](const decision_interfaces::msg::RobotStatus & msg) {
        context_.setRobotStatus(msg, nowSec());
      });
    game_status_sub_ = create_subscription<decision_interfaces::msg::GameStatus>(
      config_.topics.game_status, 10,
      [this](const decision_interfaces::msg::GameStatus & msg) {
        context_.setGameStatus(msg, nowSec());
      });
    follow_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.waypoint_executor.follow_status_topic, executorStatusQos(),
      [this](const std_msgs::msg::String & msg) {
        onExecutorStatus(decision::TargetMode::ExecutorFollow, msg.data);
      });
    through_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.waypoint_executor.through_status_topic, latchedQos(),
      [this](const std_msgs::msg::String & msg) {
        onExecutorStatus(decision::TargetMode::ExecutorThrough, msg.data);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / config_.loop_hz),
      [this]() {
        onTimer();
      });

    RCLCPP_INFO(get_logger(), "bt_action_replacement C++ node started");
  }

private:
  double nowSec()
  {
    return get_clock()->now().seconds();
  }

  bool gameActive() const
  {
    const auto status = context_.gameStatus();
    return status.has_value() &&
      static_cast<int>(status->game_progress) == config_.game.progress &&
      static_cast<int>(status->stage_remain_time) >= config_.game.lower_remain_time &&
      static_cast<int>(status->stage_remain_time) <= config_.game.higher_remain_time;
  }

  void onExecutorStatus(decision::TargetMode mode, const std::string & status)
  {
    const auto target = waypoint_executor_->targetForStatus(mode, status);
    if (!target.has_value()) {
      return;
    }

    const auto updated = waypoint_executor_->onExecutorStatus(
      *target, mode, status, nowSec());
    if (updated.result_status == decision::ExecutorResultStatus::Succeeded) {
      pending_executor_event_ = decision::ExecutorEvent{
        *target, decision::ExecutorEventType::Succeeded};
      RCLCPP_INFO(
        get_logger(), "Executor target [%s] completed",
        decision::toString(*target).c_str());
    } else if (updated.result_status == decision::ExecutorResultStatus::Aborted) {
      pending_executor_event_ = decision::ExecutorEvent{
        *target, decision::ExecutorEventType::Aborted};
      RCLCPP_WARN(
        get_logger(), "Executor target [%s] aborted",
        decision::toString(*target).c_str());
    }
  }

  void onTimer()
  {
    const double now_sec = nowSec();
    decision::DecisionInputs inputs;
    inputs.game_active = gameActive();
    inputs.current_hp = context_.effectiveCurrentHp();
    inputs.executor_event = std::exchange(pending_executor_event_, std::nullopt);
    const auto result = state_machine_->tick(inputs, now_sec);

    if (result.transitioned) {
      RCLCPP_INFO(
        get_logger(), "Decision state %s -> %s (%s)",
        decision::toString(result.previous_state).c_str(),
        decision::toString(result.state).c_str(), result.reason.c_str());
      decision_state_pub_->publish(decision::decisionStateMessage(result.state));
    }

    const auto mode = decision::TargetMode::ExecutorFollow;
    stepExecutorTarget(result.target, mode, now_sec);
  }

  void stepExecutorTarget(
    decision::TargetName target,
    decision::TargetMode mode,
    double now_sec)
  {
    const auto state = waypoint_executor_->state();
    if (state.running_target == target && state.running_mode == mode) {
      return;
    }

    std::optional<decision::Pose> robot_pose;
    std::optional<decision::Pose> target_pose;
    if (state.active_target == target &&
      state.result_status == decision::ExecutorResultStatus::Succeeded)
    {
      robot_pose = currentRobotPose(now_sec);
      target_pose = waypoint_store_->anchorPose(target);
    }

    if (!waypoint_executor_->shouldRequestTarget(
        target, mode, robot_pose, target_pose, now_sec))
    {
      return;
    }

    const auto waypoints = loadPreparedWaypoints(target, mode);
    if (!waypoints.has_value() || waypoints->empty()) {
      warnMissingCsv(target, now_sec);
      return;
    }

    const bool immediate_preempt = waypoint_executor_->shouldPreemptImmediately(target);
    const bool request_sent = waypoint_executor_->stepExecutorTarget(
      target, mode, waypoint_store_->waypointFile(target), *waypoints, immediate_preempt, now_sec);
    if (request_sent) {
      RCLCPP_INFO(
        get_logger(), "Requested executor target [%s] in mode [%s]",
        decision::toString(target).c_str(),
        mode == decision::TargetMode::ExecutorFollow ? "follow" : "through");
    }
  }

  std::optional<decision::Pose> currentRobotPose(double now_sec)
  {
    if (now_sec < next_tf_lookup_attempt_sec_) {
      return std::nullopt;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        config_.goal_frame_id,
        config_.maintain_goal.robot_base_frame,
        tf2::TimePointZero);
      decision::Pose pose;
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.z = transform.transform.translation.z;
      pose.qx = transform.transform.rotation.x;
      pose.qy = transform.transform.rotation.y;
      pose.qz = transform.transform.rotation.z;
      pose.qw = transform.transform.rotation.w;
      next_tf_lookup_attempt_sec_ = -1.0e9;
      return pose;
    } catch (const tf2::TransformException & ex) {
      next_tf_lookup_attempt_sec_ = now_sec + kTfLookupRetrySec;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot maintain wait target: transform %s <- %s unavailable: %s",
        config_.goal_frame_id.c_str(),
        config_.maintain_goal.robot_base_frame.c_str(),
        ex.what());
      return std::nullopt;
    }
  }

  std::optional<std::vector<decision::Pose>> loadPreparedWaypoints(
    decision::TargetName target,
    decision::TargetMode mode)
  {
    if (mode == decision::TargetMode::ExecutorFollow) {
      return waypoint_store_->prepareFollowWaypoints(target, std::nullopt);
    }
    return waypoint_store_->loadWaypoints(target);
  }

  void warnMissingCsv(decision::TargetName target, double now_sec)
  {
    auto & last_warn_sec = last_missing_csv_warn_sec_[target];
    if (now_sec - last_warn_sec < 5.0) {
      return;
    }

    last_warn_sec = now_sec;
    RCLCPP_WARN(
      get_logger(), "Target [%s] has no CSV. Fixed-point fallback has been removed.",
      decision::toString(target).c_str());
  }

  decision::DecisionConfig config_;
  decision::DecisionContext context_;
  std::unique_ptr<decision::DecisionStateMachine> state_machine_;
  std::unique_ptr<decision::WaypointStore> waypoint_store_;
  std::unique_ptr<decision::WaypointExecutorClient> waypoint_executor_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executor_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr saved_waypoint_file_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr follow_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr through_client_;
  rclcpp::Subscription<decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<decision_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follow_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr through_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<decision::ExecutorEvent> pending_executor_event_;
  std::map<decision::TargetName, double> last_missing_csv_warn_sec_;
  double next_tf_lookup_attempt_sec_{-1.0e9};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BtActionReplacementNode>());
  rclcpp::shutdown();
  return 0;
}
