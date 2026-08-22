#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fake_vel_transform/monotonic_stamp_gate.hpp"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void localPoseCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);

  void publishTransform();

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  // Broadcast tf from base_link to base_link_fake
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  MonotonicStampGate tf_stamp_gate_;

  double current_angle_{0.0};
  double base_link_angle_{0.0};
  // map 系下的路径切向（滤波后）。TF 发布时再减 base_link yaw，得到相对偏置。
  // 不能把偏置本身当状态：/plan 的 orientation 是单位四元数，旧逻辑会把 fake
  // 锁到 map +X，并在每次重规划时跟着底盘航向反打，轨迹跟着跳。
  double filtered_path_yaw_{0.0};
  bool have_path_yaw_{false};
  double path_lookahead_distance_{0.8};
  double yaw_filter_alpha_{0.25};
  float angular_deadband_{0.05F};
  float min_translate_speed_for_spin_{0.15F};
  float spin_speed_{0.0F};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
