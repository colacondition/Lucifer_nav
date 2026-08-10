#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Minimal in-tree replacement for the upstream common_libs <common_utils/convert.hpp>
// (that package is not part of this workspace); only the overloads used by
// small_glim_node.cpp are provided.
namespace small_glim::utils {

template<typename T>
T convert_to(const geometry_msgs::msg::Vector3& v);

template<>
inline Eigen::Vector3d convert_to<Eigen::Vector3d>(const geometry_msgs::msg::Vector3& v) {
    return {v.x, v.y, v.z};
}

inline void convert(const Eigen::Vector3d& v, geometry_msgs::msg::Vector3& msg) {
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
}

inline void convert(const Eigen::Isometry3d& T, geometry_msgs::msg::Transform& msg) {
    const Eigen::Quaterniond q(T.linear());
    msg.translation.x = T.translation().x();
    msg.translation.y = T.translation().y();
    msg.translation.z = T.translation().z();
    msg.rotation.x = q.x();
    msg.rotation.y = q.y();
    msg.rotation.z = q.z();
    msg.rotation.w = q.w();
}

inline void convert(const Eigen::Isometry3d& T, geometry_msgs::msg::Pose& msg) {
    const Eigen::Quaterniond q(T.linear());
    msg.position.x = T.translation().x();
    msg.position.y = T.translation().y();
    msg.position.z = T.translation().z();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
}

}
