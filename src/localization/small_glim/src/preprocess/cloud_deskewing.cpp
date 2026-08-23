#include <small_glim/preprocess/cloud_deskewing.hpp>
#include <small_glim/common/logger.hpp>
#include <algorithm>
#include <gtsam/geometry/Pose3.h>

namespace small_glim {

std::vector<Eigen::Vector4d> CloudDeskewing::deskew(
    const Eigen::Isometry3d& T_imu_lidar,
    const Eigen::Vector3d& linear_vel,
    const Eigen::Vector3d& angular_vel,
    const std::vector<double>& times,
    const std::vector<Eigen::Vector4d>& points
) {
    if (times.empty()) {
        return std::vector<Eigen::Vector4d>();
    }
    if (!std::isfinite(times.front()) || !std::isfinite(times.back()) || times.back() < times.front() || times.back() - times.front() > 1.0) {
        logger::warn(
            "cloud_deskewing",
            "skip deskewing due to invalid timestamp range front={:.6f} back={:.6f}",
            times.front(),
            times.back()
        );
        return points;
    }

    const Eigen::Isometry3d T_lidar_imu = T_imu_lidar.inverse();
    const gtsam::Vector6 vel = (gtsam::Vector6() << angular_vel, linear_vel).finished();

    const double time_eps = 1e-4; // 0.1msec
    std::vector<double> time_table;
    std::vector<size_t> time_indices;

    time_table.reserve(static_cast<size_t>((times.back() - times.front()) / time_eps * 1.2));
    time_indices.reserve(points.size());

    for (const double time : times) {
        if (time_table.empty() || time - time_table.back() > time_eps) {
            time_table.push_back(time);
        }
        time_indices.push_back(time_table.size() - 1);
    }

    std::vector<Eigen::Isometry3d> T_lidar0_lidar1(time_table.size());
    for (size_t i = 0; i < time_table.size(); i++) {
        const double dt = time_table[i];
        // Maybe this is not correct. Need to check.
        const Eigen::Isometry3d T_imu1_imu0(gtsam::Pose3::Expmap(dt * vel).matrix());
        T_lidar0_lidar1[i] = T_lidar_imu * T_imu1_imu0.inverse() * T_imu_lidar;
    }

    std::vector<Eigen::Vector4d> deskewed(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        const auto& T_l0_l1 = T_lidar0_lidar1[time_indices[i]];
        deskewed[i] = T_l0_l1 * points[i];
    }

    return deskewed;
}

std::vector<Eigen::Vector4d> CloudDeskewing::deskew(
    const Eigen::Isometry3d& T_imu_lidar,
    const std::vector<double>& imu_times,
    const std::vector<Eigen::Isometry3d>& imu_poses,
    const double stamp,
    const std::vector<double>& times,
    const std::vector<Eigen::Vector4d>& points
) {
    if (times.empty()) {
        return std::vector<Eigen::Vector4d>();
    }
    if (!std::isfinite(times.front()) || !std::isfinite(times.back()) || times.back() < times.front() || times.back() - times.front() > 1.0) {
        logger::warn(
            "cloud_deskewing",
            "skip deskewing due to invalid timestamp range front={:.6f} back={:.6f}",
            times.front(),
            times.back()
        );
        return points;
    }

    if (imu_poses.empty() || imu_times.size() != imu_poses.size() ||
        !std::isfinite(stamp) || !std::isfinite(imu_times.front()) ||
        imu_times.front() > stamp + 1e-6) {
        return {};
    }

    const double time_eps = 1e-4;
    std::vector<double> time_table;
    std::vector<size_t> time_indices;
    time_table.reserve(static_cast<size_t>((times.back() - times.front()) / time_eps * 1.2));
    time_indices.reserve(times.size());

    // create time table
    for (const double t: times) {
        if (time_table.empty() || t - time_table.back() > time_eps) {
            time_table.push_back(t);
        }
        time_indices.push_back(time_table.size() - 1);
    }

    const Eigen::Isometry3d T_lidar_imu = T_imu_lidar.inverse();
    std::vector<Eigen::Isometry3d> T_lidar0_lidar1(time_table.size());

    // 绝对时间轨迹插值。区间外 fail-closed，不把缺失 IMU 数据静默钳到端点；exact hit
    // 直接返回对应姿态，避免 '<' 游标在边界仍使用旧区间。
    auto pose_at = [&](const double query, Eigen::Isometry3d & pose) -> bool {
        if (!std::isfinite(query) || query < imu_times.front() - 1e-9 ||
            query > imu_times.back() + 1e-9) {
            return false;
        }
        const auto upper = std::lower_bound(imu_times.begin(), imu_times.end(), query);
        if (upper == imu_times.end()) {
            pose = imu_poses.back();
            return std::abs(query - imu_times.back()) <= 1e-9;
        }
        const size_t right = static_cast<size_t>(upper - imu_times.begin());
        if (std::abs(*upper - query) <= 1e-9 || right == 0) {
            pose = imu_poses[right];
            return true;
        }
        const size_t left = right - 1;
        const double dt = imu_times[right] - imu_times[left];
        if (!(dt > 0.0) || !std::isfinite(dt)) {
            return false;
        }
        const double p = (query - imu_times[left]) / dt;
        const Eigen::Quaterniond ql(imu_poses[left].linear());
        const Eigen::Quaterniond qr(imu_poses[right].linear());
        pose = Eigen::Isometry3d::Identity();
        pose.translation() =
            (1.0 - p) * imu_poses[left].translation() + p * imu_poses[right].translation();
        pose.linear() = ql.slerp(p, qr).normalized().toRotationMatrix();
        return pose.matrix().allFinite();
    };

    Eigen::Isometry3d T_world_imu0;
    if (!pose_at(stamp, T_world_imu0)) {
        return {};
    }
    const Eigen::Isometry3d T_imu0_world = T_world_imu0.inverse();

    // Calculate T_lidar0_lidar1 for each time in time table
    for (size_t i = 0; i < time_table.size(); i++) {
        const double time = stamp + time_table[i];
        Eigen::Isometry3d T_world_imu1;
        if (!pose_at(time, T_world_imu1)) {
            return {};
        }
        const Eigen::Isometry3d T_imu0_imu1 = T_imu0_world * T_world_imu1;
        T_lidar0_lidar1[i] = T_lidar_imu * T_imu0_imu1 * T_imu_lidar;
    }

    // Transform points
    std::vector<Eigen::Vector4d> deskewed(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        deskewed[i] = T_lidar0_lidar1[time_indices[i]] * points[i];
    }

    return deskewed;
}

}
