#include <small_glim/odometry/estimation_frame.hpp>

namespace small_glim {

const Eigen::Isometry3d EstimationFrame::T_world_frame() const {
    switch (frame_type) {
        case FrameType::WORLD: return Eigen::Isometry3d::Identity();
        case FrameType::IMU: return T_world_imu;
    }
    return Eigen::Isometry3d::Identity();
}

}
