#include "grid_utils.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace navigation2
{

class RmNav2Compat : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit RmNav2Compat(const rclcpp::NodeOptions & options)
  : Node("rm_nav2_compat", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    // 只做单向桥接。
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(10));
    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        latest_status_ = msg->data;
      });

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name_,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handleGoal(goal);
      },
      [this](const std::shared_ptr<GoalHandleNavigateToPose>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        std::thread{std::bind(&RmNav2Compat::executeGoal, this, goal_handle)}.detach();
      });

    RCLCPP_INFO(
      get_logger(), "rm_nav2_compat ready: action=%s -> goal_topic=%s",
      action_name_.c_str(), goal_topic_.c_str());
  }

private:
  void loadParameters()
  {
    // 目标输入和状态输出的话题名。
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    action_name_ = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    status_topic_ = declare_parameter<std::string>("status_topic", "/navigation2/status");
    feedback_frequency_ = declare_parameter<double>("feedback_frequency", 10.0);
    action_timeout_ = declare_parameter<double>("action_timeout", 0.0);
  }

  rclcpp_action::GoalResponse handleGoal(std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (goal->pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting NavigateToPose goal with empty frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "Preempting active NavigateToPose goal with a new goal");
    }
    active_goal_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::string getLatestStatus()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_status_;
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
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void clearActiveGoal()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_goal_ = false;
  }

  void executeGoal(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto goal_pose = goal->pose;
    // 没时间戳就补当前时间。
    if (goal_pose.header.stamp.sec == 0 && goal_pose.header.stamp.nanosec == 0) {
      goal_pose.header.stamp = now();
    }
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      latest_status_.clear();
    }
    goal_pub_->publish(goal_pose);

    RCLCPP_INFO(
      get_logger(), "Accepted NavigateToPose goal at (%.2f, %.2f) in %s",
      goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.header.frame_id.c_str());

    const auto start_time = now();
    rclcpp::Rate rate(std::max(1.0, feedback_frequency_));
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(std::make_shared<NavigateToPose::Result>());
        clearActiveGoal();
        RCLCPP_INFO(get_logger(), "NavigateToPose goal canceled");
        return;
      }

      const auto status = getLatestStatus();
      if (status == "GOAL_REACHED") {
        goal_handle->succeed(std::make_shared<NavigateToPose::Result>());
        clearActiveGoal();
        RCLCPP_INFO(get_logger(), "NavigateToPose goal succeeded");
        return;
      }
      if (status == "BLOCKED" || status == "PATH_TIMEOUT") {
        goal_handle->abort(std::make_shared<NavigateToPose::Result>());
        clearActiveGoal();
        RCLCPP_WARN(get_logger(), "NavigateToPose goal aborted: %s", status.c_str());
        return;
      }
      if (action_timeout_ > 0.0 && (now() - start_time).seconds() > action_timeout_) {
        goal_handle->abort(std::make_shared<NavigateToPose::Result>());
        clearActiveGoal();
        RCLCPP_WARN(get_logger(), "NavigateToPose goal timed out");
        return;
      }

      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      geometry_msgs::msg::PoseStamped current_pose;
      if (getRobotPose(current_pose)) {
        feedback->current_pose = current_pose;
        feedback->distance_remaining = static_cast<float>(
          std::hypot(
            goal_pose.pose.position.x - current_pose.pose.position.x,
            goal_pose.pose.position.y - current_pose.pose.position.y));
      }
      const int64_t navigation_time_ns = (now() - start_time).nanoseconds();
      feedback->navigation_time.sec = static_cast<int32_t>(navigation_time_ns / 1000000000LL);
      feedback->navigation_time.nanosec =
        static_cast<uint32_t>(navigation_time_ns % 1000000000LL);
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    clearActiveGoal();
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string action_name_;
  std::string goal_topic_;
  std::string status_topic_;
  double feedback_frequency_{10.0};
  double action_timeout_{0.0};

  std::mutex status_mutex_;
  std::string latest_status_;

  std::mutex active_goal_mutex_;
  bool active_goal_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmNav2Compat)
