// Copyright (c) 2026 colacondition
// Licensed under the MIT License. See LICENSE in the project root.
#include "fake_vel_transform/fake_vel_transform.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
namespace
{
double wrapToPi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}
}  // namespace

const char CMD_VEL_TOPIC[] = "/cmd_vel";
const char AFTER_TF_CMD_VEL[] = "/cmd_vel_chassis";
const char TRAJECTORY_TOPIC[] = "/plan";
const int TF_PUBLISH_FREQUENCY = 100;  // base_link to base_link_fake. Frequency in Hz.
const char DEFAULT_PLANNER_FRAME[] = "map";

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // 本工程模型：**一个可在地面平移的运动云台**，不做底盘小陀螺/物理轮组 yaw。
  // 本节点只把"虚拟云台系"速度换算到"实际云台/雷达系"，角速度原样透传。
  this->declare_parameter<double>("path_lookahead_distance", 0.8);
  this->declare_parameter<double>("yaw_filter_alpha", 0.25);
  path_lookahead_distance_ = this->get_parameter("path_lookahead_distance").as_double();
  yaw_filter_alpha_ = this->get_parameter("yaw_filter_alpha").as_double();

  // —— 旧小陀螺参数：已废弃，仅声明以兼容旧 yaml，设置时告警并忽略 ——
  // 模型不建模底盘自旋，任何 spin 参数都不再生效；保留声明只是避免旧配置触发
  // "parameter not declared" 而启动失败。
  const bool legacy_spin_enable = this->declare_parameter<bool>("spin_enable", false);
  const double legacy_spin_speed = this->declare_parameter<double>("spin_speed", -1.0);
  const double legacy_spin_min_speed = this->declare_parameter<double>("spin_min_speed", -1.0);
  const double legacy_min_translate_speed =
    this->declare_parameter<double>("min_translate_speed_for_spin", -1.0);
  const double legacy_angular_deadband = this->declare_parameter<double>("angular_deadband", -1.0);
  if (
    legacy_spin_enable || legacy_spin_speed >= 0.0 || legacy_spin_min_speed >= 0.0 ||
    legacy_min_translate_speed >= 0.0 || legacy_angular_deadband >= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "spin_* / angular_deadband / "
      "min_translate_speed_for_spin are obsolete and "
      "ignored: the vehicle model is a purely "
      "translating virtual gimbal (no chassis "
      "spin). Remove them from the params file.");
  }

  path_lookahead_distance_ = std::max(0.05, path_lookahead_distance_);
  yaw_filter_alpha_ = std::clamp(yaw_filter_alpha_, 0.0, 1.0);
  RCLCPP_INFO(
    get_logger(),
    "fake_vel_transform (translating-gimbal model): lookahead=%.2fm "
    "yaw_alpha=%.2f, "
    "angular passthrough, no chassis spin",
    path_lookahead_distance_, yaw_filter_alpha_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_TOPIC, 1, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1,
    std::bind(&FakeVelTransform::localPoseCallback, this, std::placeholders::_1));

  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
    std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::localPoseCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid Path message");
    return;
  }

  target_frame_ =
    msg->header.frame_id.empty() ? msg->poses.front().header.frame_id : msg->header.frame_id;
  if (target_frame_.empty()) {
    target_frame_ = DEFAULT_PLANNER_FRAME;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  try {
    const auto transform_stamped =
      tf2_buffer_->lookupTransform(target_frame_, "base_link", tf2::TimePointZero);
    robot_x = transform_stamped.transform.translation.x;
    robot_y = transform_stamped.transform.translation.y;
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot look up %s -> base_link for path heading, skip update: %s", target_frame_.c_str(),
      ex.what());
    return;
  }
  if (!std::isfinite(base_link_angle_) || !std::isfinite(robot_x) || !std::isfinite(robot_y)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Lookup returned a non-finite base_link pose, skip "
      "base_link_fake orientation update.");
    return;
  }

  // /plan 的 orientation 被 MINCO 写成单位四元数，不能当航向用。
  // 取车体最近路径点再沿弧长前瞻，用 XY 切向当 map 系航向。
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto & p = msg->poses[i].pose.position;
    const double d = std::hypot(p.x - robot_x, p.y - robot_y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }

  size_t ahead = closest;
  double acc = 0.0;
  for (size_t i = closest + 1; i < msg->poses.size(); ++i) {
    const auto & a = msg->poses[i - 1].pose.position;
    const auto & b = msg->poses[i].pose.position;
    acc += std::hypot(b.x - a.x, b.y - a.y);
    ahead = i;
    if (acc >= path_lookahead_distance_) {
      break;
    }
  }
  if (ahead == closest) {
    if (closest + 1 < msg->poses.size()) {
      ahead = closest + 1;
    } else if (closest > 0) {
      ahead = closest;
      closest = closest - 1;
    } else {
      return;
    }
  }

  constexpr double kMinSep = 0.05;
  const auto & p0 = msg->poses[closest].pose.position;
  const auto & p1 = msg->poses[ahead].pose.position;
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  if (!(std::hypot(dx, dy) >= kMinSep)) {
    return;
  }
  const double path_yaw = std::atan2(dy, dx);
  if (!std::isfinite(path_yaw)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Computed non-finite path heading, skip "
      "base_link_fake orientation update.");
    return;
  }

  if (!have_path_yaw_) {
    filtered_path_yaw_ = path_yaw;
    have_path_yaw_ = true;
  } else {
    filtered_path_yaw_ =
      wrapToPi(filtered_path_yaw_ + yaw_filter_alpha_ * wrapToPi(path_yaw - filtered_path_yaw_));
  }
  current_angle_ = wrapToPi(filtered_path_yaw_ - base_link_angle_);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  constexpr double ZERO_VELOCITY_EPS = 1e-6;
  const Twist2D fake_cmd{msg->linear.x, msg->linear.y, msg->angular.z};

  // 零速指令不必查 TF：直接转发零速，避免定位/TF 刚启动时对每条零速都刷告警。
  if (isZeroCommand(fake_cmd, ZERO_VELOCITY_EPS)) {
    cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  FrameTransform2D frame_tf;
  try {
    const std::string planner_frame = target_frame_.empty() ? DEFAULT_PLANNER_FRAME : target_frame_;
    const auto transform_stamped =
      tf2_buffer_->lookupTransform(planner_frame, "base_link", tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);
    if (!std::isfinite(base_link_angle_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lookup returned a non-finite %s -> base_link yaw, "
        "publishing zero cmd_vel.",
        planner_frame.c_str());
    } else {
      if (have_path_yaw_) {
        current_angle_ = wrapToPi(filtered_path_yaw_ - base_link_angle_);
      }
      if (std::isfinite(current_angle_)) {
        frame_tf.valid = true;
        frame_tf.fake_from_base_yaw = current_angle_;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Current base_link_fake yaw offset is non-finite, "
          "publishing zero cmd_vel.");
      }
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Could not transform planner frame to base_link, "
      "publishing zero cmd_vel: %s",
      ex.what());
  }

  // TF 失败/数据坏 => frame_tf.valid=false => computeChassisTwist 返回零速
  // （fail-stop：宁可这一拍停车，也不带着过期偏置继续走）。
  if (!frame_tf.valid) {
    cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  // 纯函数：v_B = R(fake_from_base_yaw) · v_F；角速度原样透传（无小陀螺）。
  const Twist2D chassis_cmd = computeChassisTwist(fake_cmd, frame_tf);
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.linear.x = chassis_cmd.linear_x;
  aft_tf_vel.linear.y = chassis_cmd.linear_y;
  aft_tf_vel.angular.z = chassis_cmd.angular_z;

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "forwarded cmd_vel -> cmd_vel_chassis: in=(%.2f, %.2f, "
    "%.2f) out=(%.2f, %.2f, %.2f)",
    msg->linear.x, msg->linear.y, msg->angular.z, aft_tf_vel.linear.x, aft_tf_vel.linear.y,
    aft_tf_vel.angular.z);
}

void FakeVelTransform::publishTransform()
{
  const auto stamp = get_clock()->now();
  if (!tf_stamp_gate_.accept(stamp.nanoseconds())) {
    return;
  }

  if (have_path_yaw_) {
    // TF 短失败/非有限测量时保持上一拍偏置（纯函数 offsetAfterMeasurement
    // 有单测）： 归零会让 fake 航向瞬间跳变，100 Hz 广播时表现为轨迹/偏置抖动。
    bool valid_measurement = false;
    double measured_offset = current_angle_;
    try {
      const std::string planner_frame =
        target_frame_.empty() ? DEFAULT_PLANNER_FRAME : target_frame_;
      const auto transform_stamped =
        tf2_buffer_->lookupTransform(planner_frame, "base_link", tf2::TimePointZero);
      const double yaw = tf2::getYaw(transform_stamped.transform.rotation);
      if (std::isfinite(yaw)) {
        base_link_angle_ = yaw;
        measured_offset = wrapToPi(filtered_path_yaw_ - base_link_angle_);
        valid_measurement = std::isfinite(measured_offset);
      }
    } catch (const tf2::TransformException &) {
      // 保持上一拍偏置，避免 TF 短暂失败时停发。
    }
    current_angle_ = offsetAfterMeasurement(valid_measurement, measured_offset, current_angle_);
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
      "base_link -> base_link_fake yaw offset is "
      "non-finite, skip publishing.");
    return;
  }
  tf2::Quaternion q;
  q.setRPY(0, 0, current_angle_);
  t.transform.rotation = tf2::toMsg(q);

  const auto & translation = t.transform.translation;
  const auto & rotation = t.transform.rotation;
  const bool has_non_finite_value =
    !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
    !std::isfinite(translation.z) || !std::isfinite(rotation.x) || !std::isfinite(rotation.y) ||
    !std::isfinite(rotation.z) || !std::isfinite(rotation.w);
  const bool is_empty_transform = translation.x == 0.0 && translation.y == 0.0 &&
                                  translation.z == 0.0 && rotation.x == 0.0 && rotation.y == 0.0 &&
                                  rotation.z == 0.0 && rotation.w == 0.0;
  if (has_non_finite_value || is_empty_transform) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "base_link -> base_link_fake transform is empty or "
      "invalid, skip publishing.");
    return;
  }
  tf_broadcaster_->sendTransform(t);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
