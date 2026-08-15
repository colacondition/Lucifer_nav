#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

namespace goal_approach_controller
{

class GoalApproachControllerNode : public rclcpp::Node
{
public:
  explicit GoalApproachControllerNode(const rclcpp::NodeOptions & options)
  : Node("goal_approach_controller", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    input_cmd_vel_topic_ = declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel_nav_raw");
    output_cmd_vel_topic_ = declare_parameter<std::string>("output_cmd_vel_topic", "/cmd_vel_nav");
    path_topic_ = declare_parameter<std::string>("path_topic", "/plan");
    approach_enabled_topic_ = declare_parameter<std::string>(
      "approach_enabled_topic", "/goal_approach_controller/enabled");
    approach_distance_ = declare_parameter<double>("approach_distance", 1.5);
    approach_velocity_ = declare_parameter<double>("approach_velocity", 0.5);
    direct_approach_distance_ = declare_parameter<double>("direct_approach_distance", 0.5);
    direct_approach_kp_ = declare_parameter<double>("direct_approach_kp", 1.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.25);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&GoalApproachControllerNode::cmdCallback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        if (!msg || msg->poses.empty()) {
          goal_.reset();
          goal_transform_dirty_ = true;
          return;
        }
        goal_ = msg->poses.back();
        if (goal_->header.frame_id.empty()) {
          goal_->header.frame_id = msg->header.frame_id.empty() ? global_frame_ : msg->header.frame_id;
        }
        // 目标变了，下次 cmd 回调时重算一次全局系目标。
        goal_transform_dirty_ = true;
      });
    approach_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      approach_enabled_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        approach_enabled_ = msg->data;
        RCLCPP_INFO(
          get_logger(), "Goal approach controller %s",
          approach_enabled_ ? "enabled" : "disabled");
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "goal_approach_controller ready: %s -> %s, path=%s, enabled_topic=%s",
      input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(), path_topic_.c_str(),
      approach_enabled_topic_.c_str());
  }

private:
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

  bool transformGoal(const geometry_msgs::msg::PoseStamped & in, geometry_msgs::msg::PoseStamped & out)
  {
    const std::string frame = in.header.frame_id.empty() ? global_frame_ : in.header.frame_id;
    if (frame == global_frame_) {
      out = in;
      out.header.frame_id = global_frame_;
      return true;
    }

    try {
      auto stamped = in;
      stamped.header.frame_id = frame;
      out = tf_buffer_->transform(stamped, global_frame_, tf2::durationFromSec(0.2));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform goal %s -> %s: %s",
        frame.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd = *msg;
    if (!approach_enabled_) {
      cmd_pub_->publish(cmd);
      return;
    }
    if (!goal_) {
      cmd_pub_->publish(cmd);
      return;
    }

    // 目标变换只在换路径/换目标时重算一次（此前每条 cmd_vel 都做两次 TF 查询，
    // 30Hz 下是纯浪费；目标在 map 系时 transformGoal 本就是恒等，但省掉缓存
    // 失效后的重复查询在 100Hz 底盘指令下仍有意义）。
    if (goal_transform_dirty_) {
      goal_transform_dirty_ = false;
      if (!transformGoal(*goal_, cached_goal_in_global_)) {
        cmd_pub_->publish(cmd);
        return;
      }
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      cmd_pub_->publish(cmd);
      return;
    }

    const double dx = cached_goal_in_global_.pose.position.x - robot_pose.pose.position.x;
    const double dy = cached_goal_in_global_.pose.position.y - robot_pose.pose.position.y;
    const double dist = std::hypot(dx, dy);

    if (dist <= goal_tolerance_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double local_heading = normalizeAngle(std::atan2(dy, dx) - robot_yaw);
    const double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);

    if (dist < direct_approach_distance_) {
      cmd.linear.x = target_speed * std::cos(local_heading);
      cmd.linear.y = target_speed * std::sin(local_heading);
      cmd.angular.z = 0.0;
    } else if (dist < approach_distance_) {
      const double speed = std::hypot(cmd.linear.x, cmd.linear.y);
      if (speed > approach_velocity_ && speed > 1e-6) {
        const double scale = approach_velocity_ / speed;
        cmd.linear.x *= scale;
        cmd.linear.y *= scale;
        cmd.angular.z *= scale;
      }
    }

    cmd_pub_->publish(cmd);
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string path_topic_;
  std::string approach_enabled_topic_;
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double direct_approach_distance_{0.5};
  double direct_approach_kp_{1.0};
  double goal_tolerance_{0.25};
  bool approach_enabled_{true};

  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  // 全局系目标缓存：只在换目标时重算（cmd 回调只查一次机器人位姿）。
  geometry_msgs::msg::PoseStamped cached_goal_in_global_;
  bool goal_transform_dirty_{true};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr approach_enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace goal_approach_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(goal_approach_controller::GoalApproachControllerNode)
