#pragma once
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navigation2::utils {

// 测量 odom 相对 now 的实际延迟，clamp 到 [0, max_dt]。状态外推与参考前移必须
// 用同一个延迟量（sentry 的契约：延迟补偿同时前移状态和参考），所以把它抽出来
// 供 predict_pose 与 MPC 参考构造共用，避免两处各自算、量还不一致。
inline double odom_latency(
  const nav_msgs::msg::Odometry & odom, const rclcpp::Time & now, double max_dt = 0.5)
{
  const rclcpp::Time stamp(odom.header.stamp, now.get_clock_type());
  double dt = (now - stamp).seconds();
  if (!std::isfinite(dt) || dt < 0.0) {
    dt = 0.0;
  }
  if (max_dt < 0.0) {
    max_dt = 0.0;
  }
  if (dt > max_dt) {
    dt = max_dt;
  }
  return dt;
}

// 按时间差外推位姿，最长只补到 max_dt。
inline geometry_msgs::msg::PoseStamped predict_pose(
  const nav_msgs::msg::Odometry & odom, const rclcpp::Time & now, double max_dt = 0.5)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = now;
  out.header.frame_id = odom.header.frame_id;

  const double px = odom.pose.pose.position.x;
  const double py = odom.pose.pose.position.y;
  const double pz = odom.pose.pose.position.z;

  Eigen::Quaterniond q(
    odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  Eigen::Vector3d v(
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
  Eigen::Vector3d w(
    odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);

  const double dt = odom_latency(odom, now, max_dt);

  const double wn = w.norm();
  Eigen::Quaterniond dq(Eigen::AngleAxisd(
      dt * wn, (wn > 1e-6) ? w.normalized() : Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_pred = q * dq;
  q_pred.normalize();

  Eigen::Vector3d p_pred(px, py, pz);
  // ROS 的 Odometry.twist 按 REP 103 约定表达在 child_frame（车体系），外推前
  // 必须先旋转到 header.frame_id 系。旧实现把车体速度直接加到世界位置，yaw 非零
  // 时位置预测错误；该函数只在 TF 不可用的里程计回退路径使用，此处修正不影响
  // 正常 TF 路径。
  p_pred += (q * v) * dt;

  out.pose.position.x = p_pred.x();
  out.pose.position.y = p_pred.y();
  out.pose.position.z = p_pred.z();
  out.pose.orientation.w = q_pred.w();
  out.pose.orientation.x = q_pred.x();
  out.pose.orientation.y = q_pred.y();
  out.pose.orientation.z = q_pred.z();
  return out;
}

}  // namespace navigation2::utils
