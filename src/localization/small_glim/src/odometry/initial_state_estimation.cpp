#include <small_glim/odometry/initial_state_estimation.hpp>
#include <small_glim/common/logger.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext_params.hpp>

namespace {

Eigen::Matrix3d align_vector_to_z_axis(const Eigen::Vector3d& vector) {
    constexpr double min_norm = 1e-6;
    if (vector.norm() < min_norm) {
        return Eigen::Matrix3d::Identity();
    }

    return Eigen::Quaterniond::FromTwoVectors(vector.normalized(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d align_x_axis_to_heading(const Eigen::Vector3d& x_axis_world) {
    Eigen::Vector3d heading = x_axis_world;
    heading.z() = 0.0;

    constexpr double min_norm = 1e-6;
    if (heading.norm() < min_norm) {
        return Eigen::Matrix3d::Identity();
    }

    heading.normalize();
    const double yaw_correction = -std::atan2(heading.y(), heading.x());
    return Eigen::AngleAxisd(yaw_correction, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

} // namespace

namespace small_glim {

InitialStateEstimation::InitialStateEstimation(
    const Config::Ptr config,
    const Eigen::Isometry3d& T_lidar_imu,
    const Eigen::Matrix<double, 6, 1>& imu_bias
): imu_bias(imu_bias), T_lidar_imu(T_lidar_imu) {
    num_threads = config->param<int>("odometry_estimation.num_threads");
    window_size = config->param<double>("odometry_estimation.initialization_window_size");
    align_initial_odom_to_imu = config->param<bool>("odometry_estimation.align_initial_odom_to_imu");
    naive_mode = config->param<bool>("odometry_estimation.naive_initialization");
    ready = false;
    init_stamp = 0.0;
    stamp = 0.0;
    sum_acc.setZero();
    force_init = false;
    init_v_world_imu.setZero();
    init_T_world_imu.setIdentity();
    target_ivox = std::make_unique<gtsam_points::iVox>(1.0);
    covariance_estimation = std::make_unique<CloudCovarianceEstimation>(num_threads);
    imu_integration = std::make_unique<IMUIntegration>(config);
}

void InitialStateEstimation::insert_frame(const PreprocessedFrame::ConstPtr raw_frame) {
    if (raw_frame->size() < 50) {
        logger::warn("initial_state_estimation", "skip initial state estimation for a frame with too few points ({} points)", raw_frame->size());
        return;
    }

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(raw_frame->points);
    frame->add_covs(covariance_estimation->estimate(raw_frame->points, raw_frame->neighbor_indices));

    gtsam::Pose3 estimated_T_odom_lidar = gtsam::Pose3(T_lidar_imu.inverse().matrix());

    if (!T_odom_lidar.empty()) {
        gtsam::Pose3 init_T_odom_lidar(T_odom_lidar.back().second.matrix());

        if (T_odom_lidar.size() >= 2) {
            // Linear twist motion assumption
            Eigen::Isometry3d delta = T_odom_lidar[T_odom_lidar.size() - 2].second.inverse() * T_odom_lidar[T_odom_lidar.size() - 1].second;
            delta.linear() = Eigen::Quaterniond(delta.linear()).normalized().toRotationMatrix();
            init_T_odom_lidar = init_T_odom_lidar * gtsam::Pose3(delta.matrix());
        }

        gtsam::Values values;
        values.insert(0, init_T_odom_lidar);

        gtsam::NonlinearFactorGraph graph;
        auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(gtsam::Pose3::Identity(), 0, target_ivox, frame, target_ivox);
        factor->set_num_threads(num_threads);
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        // lm_params.set_verbose();
        lm_params.setMaxIterations(10);
        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        estimated_T_odom_lidar = values.at<gtsam::Pose3>(0);
    }

    auto transformed = gtsam_points::transform(frame, Eigen::Isometry3d(estimated_T_odom_lidar.matrix()));
    target_ivox->insert(*transformed);

    T_odom_lidar.emplace_back(raw_frame->stamp, Eigen::Isometry3d(estimated_T_odom_lidar.matrix()));
}

void InitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    imu_integration->insert_imu(stamp, linear_acc, angular_vel);

    if (naive_mode && !ready && !force_init) {
        if (init_stamp <= 0.0) {
            init_stamp = stamp;
            sum_acc.setZero();
        }
        const Eigen::Vector3d corrected_acc = linear_acc - imu_bias.head<3>();
        if (corrected_acc.norm() > 1e-6) {
            sum_acc += corrected_acc.normalized();
        }
        this->stamp = stamp;
        if (stamp - init_stamp >= window_size) {
            ready = true;
        }
    }
}

EstimationFrame::ConstPtr InitialStateEstimation::initial_pose() {
    // a caller may have explicitly provided an initial state
    if (force_init) {
        EstimationFrame::Ptr estimated = std::make_shared<EstimationFrame>();
        estimated->id = static_cast<size_t>(-1);
        estimated->stamp = stamp;
        estimated->T_lidar_imu = T_lidar_imu;
        estimated->v_world_imu = init_v_world_imu;
        estimated->imu_bias = imu_bias;
        estimated->T_world_imu = init_T_world_imu;
        estimated->T_world_lidar = init_T_world_imu * T_lidar_imu.inverse();
        return estimated;
    }

    if (naive_mode) {
        if (!ready) {
            return nullptr;
        }

        // estimate initial orientation by gravity leveling using accumulated accel
        Eigen::Isometry3d init_T_world_imu = Eigen::Isometry3d::Identity();
        init_T_world_imu.linear() = align_vector_to_z_axis(sum_acc);

        if (align_initial_odom_to_imu) {
            // rotate around z so that imu x-axis horizontal projection aligns with world x-axis
            const Eigen::Vector3d imu_x_world = init_T_world_imu.linear() * Eigen::Vector3d::UnitX();
            init_T_world_imu.linear() = align_x_axis_to_heading(imu_x_world) * init_T_world_imu.linear();
        }

        // Anchor the odom origin at the power-on lidar center instead of the IMU:
        // downstream (ground_segmentation, costmap height logic) assumes z=0 is the lidar plane.
        init_T_world_imu.translation() = -(init_T_world_imu.linear() * T_lidar_imu.inverse().translation());

        EstimationFrame::Ptr estimated = std::make_shared<EstimationFrame>();
        estimated->id = static_cast<size_t>(-1);
        estimated->stamp = stamp;
        estimated->T_lidar_imu = T_lidar_imu;
        estimated->v_world_imu = Eigen::Vector3d::Zero();
        estimated->imu_bias = imu_bias;
        estimated->T_world_imu = init_T_world_imu;
        estimated->T_world_lidar = init_T_world_imu * T_lidar_imu.inverse();
        return estimated;
    }

    return nullptr;
}


void InitialStateEstimation::set_init_state(
    const Eigen::Isometry3d& init_T_world_imu_,
    const Eigen::Vector3d& init_v_world_imu_
) {
    force_init = true;
    init_T_world_imu = init_T_world_imu_;
    init_v_world_imu = init_v_world_imu_;
    ready = true;
}

}