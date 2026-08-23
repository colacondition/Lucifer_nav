// 单一参数化航点执行器：follow / patrol 由 mode 参数切换。
//
//   mode: follow | patrol
//     follow：非终点航点距离 <= switch_distance 即切换；终点等待 GOAL_REACHED 或
//             remaining <= final_goal_tolerance；approach 控制器只在终点启用。
//     patrol：每个航点都等 GOAL_REACHED；超时且 remaining < fallback_tolerance 时
//             按距离兜底接受；approach 全程启用。
//
// 只有一个可执行文件 waypoint_executor。默认 node/service/action 名走 follow，
// patrol 由 launch 覆盖 mode 与名称。bringup 只起 follow。
#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <decision_interfaces/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

constexpr std::size_t kExecutorStatusHistoryDepth = 10;

}  // namespace

class WaypointExecutor : public rclcpp::Node
{
public:
  using FollowWaypoints = decision_interfaces::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ServerGoalHandle<FollowWaypoints>;

  WaypointExecutor()
  : Node(defaultNodeName()),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declare_parameter<std::string>("mode", "follow");
    declare_parameter<std::string>("waypoint_file", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("goal_topic", "/goal_pose");
    declare_parameter<std::string>("status_topic", "/navigation2/status");
    declare_parameter<std::string>("saved_waypoint_file_topic", "/waypoint_editor/saved_waypoint_file");
    declare_parameter<std::string>("current_waypoints_topic", "/waypoint_editor/current_waypoints");
    declare_parameter<std::string>("override_waypoints_topic", "/waypoint_editor/executor_waypoints");
    declare_parameter<double>("override_waypoints_fresh_sec", 1.0);
    declare_parameter<std::string>("approach_enabled_topic", "/goal_approach_controller/enabled");
    declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<double>("switch_distance", 0.6);
    declare_parameter<double>("final_goal_tolerance", 0.35);
    // patrol 超时后的距离兜底阈值（旧实现硬编码 0.35）。
    declare_parameter<double>("fallback_tolerance", 0.35);
    declare_parameter<double>("goal_republish_interval", 1.0);
    declare_parameter<double>("goal_republish_stall_sec", 2.5);
    declare_parameter<double>("goal_progress_epsilon", 0.05);
    // 服务名与状态话题名保持旧版默认，decision 客户端无感。
    declare_parameter<std::string>("service_name", defaultServiceName());
    // action 名：decision 走 action（目标自带航点），RViz 面板仍走 Trigger 服务。
    declare_parameter<std::string>("action_name", defaultActionName());
    declare_parameter<std::string>("executor_status_topic", defaultStatusTopic());
    declare_parameter<double>("status_timeout", defaultStatusTimeout());

    const std::string mode = get_parameter("mode").as_string();
    if (mode == "patrol") {
      mode_ = Mode::Patrol;
    } else if (mode == "follow") {
      mode_ = Mode::Follow;
    } else {
      throw std::invalid_argument("mode must be 'follow' or 'patrol', got: " + mode);
    }

    goal_topic_ = get_parameter("goal_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    switch_distance_ = get_parameter("switch_distance").as_double();
    final_goal_tolerance_ = get_parameter("final_goal_tolerance").as_double();
    fallback_tolerance_ = get_parameter("fallback_tolerance").as_double();
    status_timeout_ = get_parameter("status_timeout").as_double();
    goal_republish_interval_ = get_parameter("goal_republish_interval").as_double();
    goal_republish_stall_sec_ = get_parameter("goal_republish_stall_sec").as_double();
    goal_progress_epsilon_ = get_parameter("goal_progress_epsilon").as_double();
    saved_waypoint_file_topic_ = get_parameter("saved_waypoint_file_topic").as_string();
    current_waypoints_topic_ = get_parameter("current_waypoints_topic").as_string();
    override_waypoints_topic_ = get_parameter("override_waypoints_topic").as_string();
    override_waypoints_fresh_sec_ = get_parameter("override_waypoints_fresh_sec").as_double();
    executor_status_topic_ = get_parameter("executor_status_topic").as_string();
    approach_enabled_topic_ = get_parameter("approach_enabled_topic").as_string();
    const std::string service_name = get_parameter("service_name").as_string();
    const std::string action_name = get_parameter("action_name").as_string();

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(10));
    executor_status_pub_ = create_publisher<std_msgs::msg::String>(
      executor_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(kExecutorStatusHistoryDepth)).transient_local().reliable());
    approach_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
      approach_enabled_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_status_ = msg->data;
        last_status_time_ = now();
      });
    waypoint_file_sub_ = create_subscription<std_msgs::msg::String>(
      saved_waypoint_file_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const std_msgs::msg::String::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_waypoint_file_ = msg->data;
          RCLCPP_INFO(
            get_logger(), "Updated waypoint file from topic: %s", latest_waypoint_file_.c_str());
        }
      });
    current_waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
      current_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_waypoints_ = msg->poses;
      });
    override_waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
      override_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        override_waypoints_ = msg->poses;
        override_waypoints_stamp_ = msg->header.stamp;
      });

    start_service_ = create_service<std_srvs::srv::Trigger>(
      service_name,
      std::bind(&WaypointExecutor::handleStart, this, std::placeholders::_1, std::placeholders::_2));

    // Action 版入口：goal 自带航点，客户端不再需要 override 话题 + 服务响应的
    // 异步关联。Trigger 服务保留给 RViz 面板。
    action_server_ = rclcpp_action::create_server<FollowWaypoints>(
      this,
      action_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowWaypoints::Goal> goal) {
        return (goal && !goal->waypoints.poses.empty()) ?
          rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandleFollowWaypoints>) {
        stop_requested_ = true;  // 执行循环看到即停
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle) {
        preemptAndRunAction(goal_handle);
      });

    RCLCPP_INFO(
      get_logger(), "Waypoint executor ready (mode=%s): publish %s, watch %s, service %s",
      mode.c_str(), goal_topic_.c_str(), status_topic_.c_str(), service_name.c_str());
    publishExecutorStatus("IDLE");
  }

  ~WaypointExecutor() override
  {
    // shutdown_ 不可逆：析构期间任何新的 start 请求都被拒绝，杜绝
    // 「析构置 stop 后 handleStart 又置回 false 并新起 worker_」的 UAF 面
    // （当前单线程 spin 下安全，换 MultiThreadedExecutor 时这是致命竞态）。
    shutdown_ = true;
    stop_requested_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  enum class Mode
  {
    Follow,
    Patrol,
  };

  static const char * defaultNodeName()
  {
    return "waypoint_executor";
  }

  static const char * defaultServiceName()
  {
    return "start_waypoint_following";
  }

  static const char * defaultStatusTopic()
  {
    return "/waypoint_editor/follow_status";
  }

  static double defaultStatusTimeout()
  {
    return 30.0;
  }

  static const char * defaultActionName()
  {
    return "/waypoint_editor/follow_waypoints";
  }

  bool loadWaypointsFromCSV(
    const std::string & filename,
    std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      return false;
    }

    const std::string frame_id = get_parameter("frame_id").as_string();

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }

      std::istringstream ss(line);
      std::string token;
      std::vector<std::string> tokens;
      while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
      }

      if (tokens.size() < 8) {
        RCLCPP_WARN(get_logger(), "Skipping invalid waypoint line: %s", line.c_str());
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = now();

      try {
        pose.pose.position.x = std::stod(tokens[1]);
        pose.pose.position.y = std::stod(tokens[2]);
        pose.pose.position.z = std::stod(tokens[3]);
        pose.pose.orientation.x = std::stod(tokens[4]);
        pose.pose.orientation.y = std::stod(tokens[5]);
        pose.pose.orientation.z = std::stod(tokens[6]);
        pose.pose.orientation.w = std::stod(tokens[7]);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(
          get_logger(), "Failed to parse waypoint line: %s (%s)", line.c_str(), ex.what());
        continue;
      }

      waypoints.push_back(pose);
    }

    return !waypoints.empty();
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
      pose.header.frame_id = global_frame_;
      pose.header.stamp = now();
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot get robot pose %s -> %s: %s",
        global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  double distanceToGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      return std::numeric_limits<double>::infinity();
    }
    return std::hypot(
      goal.pose.position.x - robot_pose.pose.position.x,
      goal.pose.position.y - robot_pose.pose.position.y);
  }

  std::string getLatestStatus()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_status_;
  }

  std::string getLatestWaypointFile()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_waypoint_file_;
  }

  std::vector<geometry_msgs::msg::PoseStamped> getCurrentWaypoints()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_waypoints_;
  }

  std::vector<geometry_msgs::msg::PoseStamped> getOverrideWaypointsIfFresh()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (override_waypoints_.empty()) {
      return {};
    }
    const auto stamp = rclcpp::Time(override_waypoints_stamp_);
    if ((now() - stamp).seconds() > override_waypoints_fresh_sec_) {
      return {};
    }
    return override_waypoints_;
  }

  void publishGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    auto stamped_goal = goal;
    stamped_goal.header.stamp = now();
    goal_pub_->publish(stamped_goal);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Publishing through waypoint goal to %s: x=%.2f y=%.2f frame=%s",
      goal_topic_.c_str(), stamped_goal.pose.position.x, stamped_goal.pose.position.y,
      stamped_goal.header.frame_id.c_str());
  }

  void publishApproachEnabled(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    approach_enabled_pub_->publish(msg);
  }

  void publishExecutorStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    executor_status_pub_->publish(msg);
  }

  bool waitUntilWaypoint(
    const geometry_msgs::msg::PoseStamped & goal,
    std::size_t index,
    bool is_final_goal)
  {
    const auto start_time = now();
    auto last_publish_time = now() - rclcpp::Duration::from_seconds(goal_republish_interval_);
    auto last_progress_time = now();
    double best_remaining = std::numeric_limits<double>::infinity();
    rclcpp::Rate rate(10.0);

    while (rclcpp::ok() && !stop_requested_) {
      const double remaining = distanceToGoal(goal);
      if (std::isfinite(remaining) && remaining + goal_progress_epsilon_ < best_remaining) {
        best_remaining = remaining;
        last_progress_time = now();
      }

      if ((now() - last_publish_time).seconds() >= goal_republish_interval_) {
        const bool stalled =
          !std::isfinite(best_remaining) ||
          (now() - last_progress_time).seconds() >= goal_republish_stall_sec_;
        if (stalled) {
          publishGoal(goal);
          last_publish_time = now();
        }
      }

      const auto latest_status = getLatestStatus();

      if (latest_status == "BLOCKED" || latest_status == "PATH_TIMEOUT") {
        RCLCPP_WARN(
          get_logger(), "Waypoint %zu failed with status %s", index, latest_status.c_str());
        return false;
      }

      if (mode_ == Mode::Follow) {
        // follow：非终点距离到了就切下一个；终点等 GOAL_REACHED 或距离容差。
        if (is_final_goal) {
          if (latest_status == "GOAL_REACHED" || remaining <= final_goal_tolerance_) {
            RCLCPP_INFO(get_logger(), "Final waypoint reached");
            return true;
          }
        } else if (remaining <= switch_distance_) {
          RCLCPP_INFO(
            get_logger(), "Waypoint %zu close enough, switch to next (%.2f m)", index, remaining);
          return true;
        }
      } else {
        // patrol：每个航点都等 GOAL_REACHED，超时后按距离兜底。
        if (latest_status == "GOAL_REACHED") {
          RCLCPP_INFO(get_logger(), "Waypoint %zu reached", index);
          return true;
        }
      }

      if (status_timeout_ > 0.0 && (now() - start_time).seconds() > status_timeout_) {
        if (mode_ == Mode::Patrol && remaining < fallback_tolerance_) {
          RCLCPP_INFO(
            get_logger(), "Waypoint %zu accepted by distance fallback (%.2f m)", index, remaining);
          return true;
        }
        RCLCPP_WARN(get_logger(), "Waypoint %zu timed out", index);
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  void runWaypoints(std::vector<geometry_msgs::msg::PoseStamped> waypoints)
  {
    const bool approach_always_on = (mode_ == Mode::Patrol);
    if (approach_always_on) {
      publishApproachEnabled(true);
    }

    for (std::size_t i = 0; i < waypoints.size() && rclcpp::ok() && !stop_requested_; ++i) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_status_.clear();
      }

      const bool is_final_goal = (i + 1 == waypoints.size());
      publishApproachEnabled(approach_always_on || is_final_goal);
      publishGoal(waypoints[i]);
      RCLCPP_INFO(
        get_logger(), "Sent waypoint %zu / %zu%s: x=%.2f y=%.2f frame=%s",
        i + 1, waypoints.size(), is_final_goal ? " (final)" : "",
        waypoints[i].pose.position.x, waypoints[i].pose.position.y,
        waypoints[i].header.frame_id.c_str());

      if (!waitUntilWaypoint(waypoints[i], i + 1, is_final_goal)) {
        publishApproachEnabled(true);
        worker_running_ = false;
        publishExecutorStatus(stop_requested_ ? "IDLE" : "ABORTED");
        if (!stop_requested_) {
          RCLCPP_WARN(get_logger(), "Waypoint execution aborted");
        }
        return;
      }
    }

    publishApproachEnabled(true);
    worker_running_ = false;
    publishExecutorStatus(stop_requested_ ? "IDLE" : "COMPLETED");
    if (!stop_requested_) {
      RCLCPP_INFO(get_logger(), "Waypoint execution completed");
    }
  }

  // ===== Action 路径 =====

  // rclcpp_action 的 goal handle 在目标已被终止（抢占/取消竞态）时，终端调用
  // 与 publish_feedback 会抛异常；这里不是容器，但未捕获异常会杀掉执行器进程。
  static void safeAbort(
    const std::shared_ptr<GoalHandleFollowWaypoints> & handle,
    const std::shared_ptr<FollowWaypoints::Result> & result,
    rclcpp::Logger logger)
  {
    try { handle->abort(result); } catch (const std::exception & ex) {
      RCLCPP_WARN(logger, "abort() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  static void safeCancel(
    const std::shared_ptr<GoalHandleFollowWaypoints> & handle,
    const std::shared_ptr<FollowWaypoints::Result> & result,
    rclcpp::Logger logger)
  {
    try { handle->canceled(result); } catch (const std::exception & ex) {
      RCLCPP_WARN(logger, "canceled() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  static void safeSucceed(
    const std::shared_ptr<GoalHandleFollowWaypoints> & handle,
    const std::shared_ptr<FollowWaypoints::Result> & result,
    rclcpp::Logger logger)
  {
    try { handle->succeed(result); } catch (const std::exception & ex) {
      RCLCPP_WARN(logger, "succeed() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  static void safeFeedback(
    const std::shared_ptr<GoalHandleFollowWaypoints> & handle,
    const std::shared_ptr<FollowWaypoints::Feedback> & feedback,
    rclcpp::Logger logger)
  {
    try { handle->publish_feedback(feedback); } catch (const std::exception & ex) {
      RCLCPP_WARN(logger, "publish_feedback() on terminated goal ignored: %s", ex.what());
    } catch (...) {}
  }

  // 抢占当前任务并执行新 action 目标（与 Trigger 路径共用同一个 worker_）。
  void preemptAndRunAction(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
  {
    if (shutdown_) {
      auto result = std::make_shared<FollowWaypoints::Result>();
      result->success = false;
      result->message = "executor shutting down";
      safeAbort(goal_handle, result, get_logger());
      return;
    }
    if (worker_running_) {
      RCLCPP_INFO(get_logger(), "Preempting current task for a new action goal");
      stop_requested_ = true;
      if (worker_.joinable()) {
        worker_.join();
      }
      worker_running_ = false;
    }

    const auto goal = goal_handle->get_goal();
    std::vector<geometry_msgs::msg::PoseStamped> waypoints(
      goal->waypoints.poses.begin(), goal->waypoints.poses.end());
    if (waypoints.empty()) {
      auto result = std::make_shared<FollowWaypoints::Result>();
      result->success = false;
      result->message = "empty waypoints";
      safeAbort(goal_handle, result, get_logger());
      return;
    }

    stop_requested_ = false;
    worker_running_ = true;
    publishExecutorStatus("RUNNING");
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle, waypoints = std::move(waypoints)]() mutable {
      runWaypointsAction(goal_handle, std::move(waypoints));
    });
  }

  void runWaypointsAction(
    const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle,
    std::vector<geometry_msgs::msg::PoseStamped> waypoints)
  {
    const bool approach_always_on = (mode_ == Mode::Patrol);
    if (approach_always_on) {
      publishApproachEnabled(true);
    }

    auto result = std::make_shared<FollowWaypoints::Result>();
    auto feedback = std::make_shared<FollowWaypoints::Feedback>();

    for (std::size_t i = 0; i < waypoints.size() && rclcpp::ok(); ++i) {
      if (stop_requested_ || goal_handle->is_canceling()) {
        result->success = false;
        result->message = "canceled";
        publishApproachEnabled(true);
        worker_running_ = false;
        publishExecutorStatus("IDLE");
        safeCancel(goal_handle, result, get_logger());
        return;
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_status_.clear();
      }

      const bool is_final_goal = (i + 1 == waypoints.size());
      publishApproachEnabled(approach_always_on || is_final_goal);
      publishGoal(waypoints[i]);
      feedback->current_index = static_cast<int32_t>(i + 1);
      feedback->status = "RUNNING";
      safeFeedback(goal_handle, feedback, get_logger());
      RCLCPP_INFO(
        get_logger(), "Action waypoint %zu / %zu%s: x=%.2f y=%.2f",
        i + 1, waypoints.size(), is_final_goal ? " (final)" : "",
        waypoints[i].pose.position.x, waypoints[i].pose.position.y);

      if (!waitUntilWaypoint(waypoints[i], i + 1, is_final_goal)) {
        result->success = false;
        result->message = stop_requested_ ? "canceled" : "aborted";
        publishApproachEnabled(true);
        worker_running_ = false;
        publishExecutorStatus(stop_requested_ ? "IDLE" : "ABORTED");
        safeAbort(goal_handle, result, get_logger());
        return;
      }
    }

    result->success = true;
    result->message = "completed";
    publishApproachEnabled(true);
    worker_running_ = false;
    publishExecutorStatus(stop_requested_ ? "IDLE" : "COMPLETED");
    safeSucceed(goal_handle, result, get_logger());
  }

  void handleStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (shutdown_) {
      response->success = false;
      response->message = "Waypoint executor is shutting down";
      return;
    }
    if (worker_running_) {
      RCLCPP_INFO(get_logger(), "Restarting waypoint execution on new start request");
      stop_requested_ = true;
      if (worker_.joinable()) {
        worker_.join();
      }
      worker_running_ = false;
      publishExecutorStatus("IDLE");
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints = getOverrideWaypointsIfFresh();
    if (!waypoints.empty()) {
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from fresh executor override", waypoints.size());
    } else {
      waypoints = getCurrentWaypoints();
    }
    if (waypoints.empty()) {
      std::string waypoint_file = getLatestWaypointFile();
      if (waypoint_file.empty()) {
        waypoint_file = get_parameter("waypoint_file").as_string();
      }
      if (waypoint_file.empty()) {
        response->success = false;
        response->message = "No current waypoints; add waypoints in RViz or load/save a CSV first";
        return;
      }
      if (!loadWaypointsFromCSV(waypoint_file, waypoints)) {
        response->success = false;
        response->message =
          "No current waypoints, and failed to load CSV: " + waypoint_file;
        return;
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", waypoints.size(), waypoint_file.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from current editor state", waypoints.size());
    }

    if (waypoints.empty()) {
      response->success = false;
      response->message = "No waypoints to execute";
      return;
    }

    const auto waypoint_count = waypoints.size();
    stop_requested_ = false;
    worker_running_ = true;
    publishExecutorStatus("RUNNING");
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(
      [this, waypoints = std::move(waypoints)]() mutable {runWaypoints(std::move(waypoints));});

    response->success = true;
    response->message = "Waypoint execution started with " + std::to_string(waypoint_count) + " waypoints";
  }

  Mode mode_{Mode::Follow};
  std::string goal_topic_;
  std::string status_topic_;
  std::string saved_waypoint_file_topic_;
  std::string current_waypoints_topic_;
  std::string override_waypoints_topic_;
  double override_waypoints_fresh_sec_{1.0};
  std::string executor_status_topic_;
  std::string approach_enabled_topic_;
  std::string robot_base_frame_;
  std::string global_frame_;
  double switch_distance_{0.6};
  double final_goal_tolerance_{0.35};
  double fallback_tolerance_{0.35};
  double status_timeout_{30.0};
  double goal_republish_interval_{1.0};
  double goal_republish_stall_sec_{2.5};
  double goal_progress_epsilon_{0.05};
  std::string latest_status_;
  std::string latest_waypoint_file_;
  std::vector<geometry_msgs::msg::PoseStamped> current_waypoints_;
  std::vector<geometry_msgs::msg::PoseStamped> override_waypoints_;
  builtin_interfaces::msg::Time override_waypoints_stamp_;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  std::mutex state_mutex_;
  std::thread worker_;
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr executor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr approach_enabled_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr current_waypoints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr override_waypoints_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp_action::Server<FollowWaypoints>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointExecutor>());
  rclcpp::shutdown();
  return 0;
}
