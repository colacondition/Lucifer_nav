#pragma once

#include <memory>
#include <small_glim/common/raw_points.hpp>

namespace small_glim {

/**
* @brief Preprocessed point cloud
*/
struct PreprocessedFrame {
public:
    using Ptr = std::shared_ptr<PreprocessedFrame>;
    using ConstPtr = std::shared_ptr<const PreprocessedFrame>;
    /**
    * @brief Number of points
    * @return Number of points
    */
    size_t size() const {
        return points.size();
    }

public:
    double stamp; // Timestamp at the beginning of the scan
    double scan_end_time; // Timestamp at the end of the scan

    std::vector<double> times; // Point timestamps w.r.t. the first pt
    std::vector<double> intensities; // Point intensities
    std::vector<Eigen::Vector4d> points; // Points (homogeneous coordinates)

    // 与本帧严格同时间戳的定位稠密云。只挂在送入里程计队列的主 frame 上，避免节点用
    // “最新稠密云 + 当前完成的估计帧”松散配对；定位线程积压时二者仍保持一一对应。
    // 稠密子帧自身不再挂子帧，不形成引用环。
    ConstPtr localization_frame;

    size_t k_neighbors; // Number of neighbors of each point
    std::vector<size_t> neighbor_indices; // k-nearest neighbor indices of each point
};

}