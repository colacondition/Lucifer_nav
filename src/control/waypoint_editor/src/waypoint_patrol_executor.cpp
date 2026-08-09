#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class WaypointPatrolExecutor : public rclcpp::Node
{
public:
  WaypointPatrolExecutor()
  : Node("waypoint_patrol_executor"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declare_parameter<std::string>("waypoint_file", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("goal_topic", "/goal_pose");
    declare_parameter<std::string>("status_topic", "/navigation2/status");
    declare_parameter<std::string>("saved_waypoint_file_topic", "/waypoint_editor/saved_waypoint_file");
    declare_parameter<std::string>("current_waypoints_topic", "/waypoint_editor/current_waypoints");
    declare_parameter<std::string>("override_waypoints_topic", "/waypoint_editor/executor_waypoints");
    declare_parameter<double>("override_waypoints_fresh_sec", 1.0);
    declare_parameter<std::string>("executor_status_topic", "/waypoint_editor/through_status");
    declare_parameter<std::string>("approach_enabled_topic", "/goal_approach_controller/enabled");
    declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<double>("status_timeout", 20.0);
    declare_parameter<double>("goal_republish_interval", 1.0);
    declare_parameter<double>("goal_republish_stall_sec", 2.5);
    declare_parameter<double>("goal_progress_epsilon", 0.05);

    goal_topic_ = get_parameter("goal_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
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

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(10));
    executor_status_pub_ = create_publisher<std_msgs::msg::String>(
      executor_status_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    approach_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
      approach_enabled_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_status_ = msg->data;
        last_status_time_ = now();
      });
    waypoint_file_sub_ = create_subscription<std_msgs::msg::String>(
      saved_waypoint_file_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg->data.empty()) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_waypoint_file_ = msg->data;
          RCLCPP_INFO(
            get_logger(), "Updated waypoint file from topic: %s", latest_waypoint_file_.c_str());
        }
      });
    current_waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
      current_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_waypoints_ = msg->poses;
      });
    override_waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
      override_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        override_waypoints_ = msg->poses;
        override_waypoints_stamp_ = msg->header.stamp;
      });

    start_service_ = create_service<std_srvs::srv::Trigger>(
      "start_waypoint_through",
      std::bind(
        &WaypointPatrolExecutor::handleStartPatrol, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Waypoint-patrol executor ready: publish %s, watch %s",
      goal_topic_.c_str(), status_topic_.c_str());
    publishExecutorStatus("IDLE");
  }

  ~WaypointPatrolExecutor() override
  {
    stop_requested_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
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

  bool waitForWaypointResult(const geometry_msgs::msg::PoseStamped & goal, std::size_t index)
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
          auto republished_goal = goal;
          republished_goal.header.stamp = now();
          goal_pub_->publish(republished_goal);
          last_publish_time = now();
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Publishing waypoint %zu goal to %s: x=%.2f y=%.2f frame=%s",
            index, goal_topic_.c_str(), republished_goal.pose.position.x,
            republished_goal.pose.position.y, republished_goal.header.frame_id.c_str());
        }
      }

      const auto latest_status = getLatestStatus();
      if (latest_status == "GOAL_REACHED") {
        RCLCPP_INFO(get_logger(), "Waypoint %zu reached", index);
        return true;
      }

      if (latest_status == "BLOCKED" || latest_status == "PATH_TIMEOUT") {
        RCLCPP_WARN(
          get_logger(), "Waypoint %zu failed with status %s", index, latest_status.c_str());
        return false;
      }

      if (status_timeout_ > 0.0 && (now() - start_time).seconds() > status_timeout_) {
        if (remaining < 0.35) {
          RCLCPP_INFO(
            get_logger(), "Waypoint %zu accepted by distance fallback (%.2f m)", index, remaining);
          return true;
        }
        RCLCPP_WARN(get_logger(), "Waypoint %zu timed out waiting for status", index);
        return false;
      }

      rate.sleep();
    }
    return false;
  }

  void publishExecutorStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    executor_status_pub_->publish(msg);
  }

  void runPatrol(std::vector<geometry_msgs::msg::PoseStamped> waypoints)
  {
    publishApproachEnabled(true);
    for (std::size_t i = 0; i < waypoints.size() && rclcpp::ok() && !stop_requested_; ++i) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_status_.clear();
      }

      auto goal = waypoints[i];
      goal.header.stamp = now();
      goal_pub_->publish(goal);
      RCLCPP_INFO(
        get_logger(), "Sent waypoint %zu / %zu to %s: x=%.2f y=%.2f frame=%s",
        i + 1, waypoints.size(), goal_topic_.c_str(), goal.pose.position.x,
        goal.pose.position.y, goal.header.frame_id.c_str());

      if (!waitForWaypointResult(goal, i + 1)) {
        publishApproachEnabled(true);
        worker_running_ = false;
        publishExecutorStatus(stop_requested_ ? "IDLE" : "ABORTED");
        if (!stop_requested_) {
          RCLCPP_WARN(get_logger(), "Waypoint-patrol aborted");
        }
        return;
      }
    }

    publishApproachEnabled(true);
    worker_running_ = false;
    publishExecutorStatus(stop_requested_ ? "IDLE" : "COMPLETED");
    if (!stop_requested_) {
      RCLCPP_INFO(get_logger(), "Waypoint-patrol completed");
    }
  }

  void publishApproachEnabled(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    approach_enabled_pub_->publish(msg);
  }

  void handleStartPatrol(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (worker_running_) {
      RCLCPP_INFO(get_logger(), "Restarting waypoint-patrol on new start request");
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
      response->message = "No waypoints to patrol";
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
      [this, waypoints = std::move(waypoints)]() mutable {runPatrol(std::move(waypoints));});

    response->success = true;
    response->message = "Waypoint patrol started with " + std::to_string(waypoint_count) + " waypoints";
  }

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
  double status_timeout_{20.0};
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

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr executor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr approach_enabled_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr current_waypoints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr override_waypoints_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointPatrolExecutor>());
  rclcpp::shutdown();
  return 0;
}
