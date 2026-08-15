#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
const std::string CMD_VEL_TOPIC = "/cmd_vel";
const std::string AFTER_TF_CMD_VEL = "/cmd_vel_chassis";
const std::string TRAJECTORY_TOPIC = "/local_plan";
const int TF_PUBLISH_FREQUENCY = 100;  // base_link to base_link_fake. Frequency in Hz.
const std::string DEFAULT_PLANNER_FRAME = "map";

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // Declare and get the spin speed parameter
  this->declare_parameter<float>("spin_speed", -6.0);
  this->declare_parameter<float>("angular_deadband", 0.05);
  this->declare_parameter<float>("min_translate_speed_for_spin", 0.15);
  this->get_parameter("spin_speed", spin_speed_);
  this->get_parameter("angular_deadband", angular_deadband_);
  this->get_parameter("min_translate_speed_for_spin", min_translate_speed_for_spin_);

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Create Publisher and Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_TOPIC, 1, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1,
    std::bind(&FakeVelTransform::localPoseCallback, this, std::placeholders::_1));

  // Create a timer to publish the transform regularly
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
    std::bind(&FakeVelTransform::publishTransform, this));
}

// Get the local pose from planner
void FakeVelTransform::localPoseCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid Path message");
    return;
  }

  // Choose the pose based on the size of the poses array
  size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;
  target_frame_ = msg->header.frame_id.empty() ? msg->poses[index].header.frame_id : msg->header.frame_id;
  if (target_frame_.empty()) {
    target_frame_ = DEFAULT_PLANNER_FRAME;
  }

  // Update current angle based on the difference between path yaw and base_link yaw.
  double path_yaw = tf2::getYaw(selected_pose.orientation);
  if (!std::isfinite(path_yaw)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received non-finite path heading, skip base_link_fake orientation update.");
    return;
  }

  // base_link_angle_ 只在 cmdVelCallback 里更新；/local_plan 先于第一条 /cmd_vel
  // 到达（或 TF 刚恢复）时它还是 0，直接相减会把真实航向当成 0，底盘朝错误方向
  // 运动。这里与 cmdVelCallback 走同一条 TF 查询路径更新它。
  try {
    const std::string planner_frame =
      target_frame_.empty() ? DEFAULT_PLANNER_FRAME : target_frame_;
    const auto transform_stamped = tf2_buffer_->lookupTransform(
      planner_frame, "base_link", tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot look up %s -> base_link for path heading, skip update: %s",
      (target_frame_.empty() ? DEFAULT_PLANNER_FRAME : target_frame_).c_str(), ex.what());
    return;
  }

  if (!std::isfinite(base_link_angle_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Lookup returned a non-finite base_link yaw, skip base_link_fake orientation update.");
    return;
  }

  current_angle_ = path_yaw - base_link_angle_;
  if (!std::isfinite(current_angle_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Computed non-finite base_link_fake yaw offset, reset to zero.");
    current_angle_ = 0.0;
  }
}

// Transform the velocity from base_link to base_link_fake
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  constexpr double ZERO_VELOCITY_EPS = 1e-6;
  const bool is_zero_cmd = std::abs(msg->linear.x) < ZERO_VELOCITY_EPS &&
    std::abs(msg->linear.y) < ZERO_VELOCITY_EPS &&
    std::abs(msg->angular.z) < ZERO_VELOCITY_EPS;
  if (is_zero_cmd) {
    cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    const std::string planner_frame =
      target_frame_.empty() ? DEFAULT_PLANNER_FRAME : target_frame_;
    transform_stamped = tf2_buffer_->lookupTransform(
      planner_frame, "base_link", tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);
    if (!std::isfinite(base_link_angle_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lookup returned a non-finite %s -> base_link yaw, skip cmd_vel transform.",
        planner_frame.c_str());
      return;
    }

    double angle_diff = -current_angle_;
    if (!std::isfinite(angle_diff)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Current base_link_fake yaw offset is non-finite, skip cmd_vel transform.");
      return;
    }

    geometry_msgs::msg::Twist aft_tf_vel;
    const double linear_speed = std::hypot(msg->linear.x, msg->linear.y);
    const bool should_spin =
      linear_speed >= min_translate_speed_for_spin_ &&
      std::abs(msg->angular.z) >= angular_deadband_;
    aft_tf_vel.angular.z = should_spin ? spin_speed_ : 0.0;
    aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);

    cmd_vel_chassis_pub_->publish(aft_tf_vel);
    // 诊断锚点：线上出现过「MPC 判定 commanding velocity 但车不动」的故障，
    // 需要知道指令到底在哪一环断掉。这条日志能证明本节点确实收到了 /cmd_vel
    // 并把非零指令发到了 /cmd_vel_chassis；若 gzserver 仍然没动，剩下的断点
    // 就在插件/物理侧（配合 ros2 topic echo /cmd_vel_chassis 一起看）。
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "forwarded cmd_vel -> cmd_vel_chassis: in=(%.2f, %.2f, %.2f) out=(%.2f, %.2f, %.2f)",
      msg->linear.x, msg->linear.y, msg->angular.z,
      aft_tf_vel.linear.x, aft_tf_vel.linear.y, aft_tf_vel.angular.z);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Could not transform planner frame to base_link, skip non-zero cmd_vel: %s", ex.what());
  }
}

// Publish transform from base_link to base_link_fake
void FakeVelTransform::publishTransform()
{
  const auto stamp = get_clock()->now();
  if (!tf_stamp_gate_.accept(stamp.nanoseconds())) {
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_link_fake";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  if (!std::isfinite(current_angle_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "base_link -> base_link_fake yaw offset is non-finite, skip publishing.");
    return;
  }
  tf2::Quaternion q;
  q.setRPY(0, 0, current_angle_);
  t.transform.rotation = tf2::toMsg(q);

  const auto & translation = t.transform.translation;
  const auto & rotation = t.transform.rotation;
  const bool has_non_finite_value =
    !std::isfinite(translation.x) || !std::isfinite(translation.y) || !std::isfinite(translation.z) ||
    !std::isfinite(rotation.x) || !std::isfinite(rotation.y) || !std::isfinite(rotation.z) ||
    !std::isfinite(rotation.w);
  const bool is_empty_transform =
    translation.x == 0.0 && translation.y == 0.0 && translation.z == 0.0 &&
    rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0 && rotation.w == 0.0;
  if (has_non_finite_value || is_empty_transform) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "base_link -> base_link_fake transform is empty or invalid, skip publishing.");
    return;
  }
  tf_broadcaster_->sendTransform(t);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
