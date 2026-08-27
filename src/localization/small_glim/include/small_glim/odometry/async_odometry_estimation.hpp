#pragma once

#include <thread>
#include <atomic>
#include <small_glim/odometry/odometry_estimation.hpp>
#include <small_glim/common/concurrent_queue.hpp>

namespace small_glim {

/**
* @brief Odometry estimation executor to wrap and asynchronously run OdometryEstimationCPU
* @note  All the exposed public methods are thread-safe
*/
class AsyncOdometryEstimation {
public:
    /**
    * @brief Construct a new Async Odometry Estimation object
    * @param odometry_estimation  Odometry estimation to be wrapped
    */
    explicit AsyncOdometryEstimation(const Config::Ptr config);

    /**
    * @brief Destroy the Async Odometry Estimation object
    */
    ~AsyncOdometryEstimation();

    /**
    * @brief Insert an IMU data into the odometry estimation
    * @param stamp         Timestamp
    * @param linear_acc    Linear acceleration
    * @param angular_vel   Angular velocity
    */
    void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
    /// 最新一帧原始陀螺（body 系）。内部 IMU 队列为空时返回 false。
    /// 线程安全性同 insert_imu（经内部队列锁/单写者纪律）。
    bool latest_angular_velocity(Eigen::Vector3d* out) const;
    /// 最新一帧估计出的 IMU 偏置 [gyro(3), acc(3)]。尚无帧时返回 false。
    bool latest_imu_bias(Eigen::Matrix<double, 6, 1>* out) const;

    /**
    * @brief Insert a preprocessed point cloud into odometry estimation
    * @param frame  Preprocessed point cloud
    */
    void insert_frame(const PreprocessedFrame::Ptr frame);

    /**
    * @brief Wait for the odometry estimation thread
    */
    void join();

    /**
    * @brief   Get the size of the input queue
    */
    size_t workload() const;

    /**
    * @brief Get the estimation results
    * @param estimation_results    Estimation results
    * @param marginalized_frames   Marginalized frames
    */
    void get_results(
        std::vector<EstimationFrame::ConstPtr>& estimation_results,
        std::vector<EstimationFrame::ConstPtr>& target_ivox_frames,
        std::vector<EstimationFrame::ConstPtr>& marginalized_frames
    );

    /**
     * @brief 开关 ivox 帧输出。没有订阅者时（RViz 没开 ivox_cloud 显示）关闭，
     *        省掉每帧把整张体素图拷成点云的成本。默认开，保持旧行为。
     */
    void set_output_ivox(bool enabled);

private:
    void run();

private:
    double max_scan_duration;
    double max_imu_wait_time_gap;
    double max_imu_wait_wall_time;
    int max_internal_frame_queue;

    std::atomic_bool output_ivox_enabled{true};
    std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
    std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
    std::thread thread;

    // Input queues
    ConcurrentQueue<Eigen::Matrix<double, 7, 1>> input_imu_queue {DataStorePolicy::UPTO(100)};
    ConcurrentQueue<PreprocessedFrame::Ptr> input_frame_queue {DataStorePolicy::UPTO(100)};

    // Output queues
    ConcurrentQueue<EstimationFrame::ConstPtr> output_estimation_results {DataStorePolicy::UPTO(10)};
    ConcurrentQueue<EstimationFrame::ConstPtr> output_target_ivox_frames {DataStorePolicy::UPTO(10)};
    ConcurrentQueue<EstimationFrame::ConstPtr> output_marginalized_frames {DataStorePolicy::UPTO(10)};

    std::atomic_size_t internal_frame_queue_size;
    std::shared_ptr<OdometryEstimationCPU> odometry_estimation;
};

}
