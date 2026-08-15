#include <algorithm>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <small_glim/common/config.hpp>
#include <small_glim/common/logger.hpp>
#include <small_glim/common/ros_convert.hpp>
#include <small_glim/preprocess/cloud_preprocessor.hpp>
#include <small_glim/preprocess/time_keeper.hpp>
#include <small_glim/odometry/async_odometry_estimation.hpp>
#include <small_glim/odometry/imu_integration.hpp>
#include <small_glim/mapping/async_mapping.hpp>

namespace small_glim {

class SmallGlimNode: public rclcpp::Node {
public:
    explicit SmallGlimNode(const rclcpp::NodeOptions& options);
    ~SmallGlimNode() override;

    void timer_callback();
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    size_t lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    // 高频里程计输出：在 10Hz 雷达帧之间用最新估计的 bias 与 IMU 数据前向传播，
    // 以 high_rate_odom_hz 发布 /Odometry 与 lidar_odom->base_link TF。
    // /lio/robo/odom 始终保留 10Hz 校正流，供 fast_location 与点云同源使用。
    void high_rate_timer_callback();

    void pub_odometry(const EstimationFrame::ConstPtr frame);
    void pub_propagated_odometry(
        const Eigen::Isometry3d & T_world_imu, const Eigen::Vector3d & v_world_imu,
        const Eigen::Isometry3d & T_lidar_imu, const rclcpp::Time & stamp);
    void pub_odometry_impl(
        const Eigen::Isometry3d & T_odom_lidar, const Eigen::Vector3d & v_body,
        const rclcpp::Time & stamp, bool publish_tf, bool publish_primary_odom,
        bool publish_robo_odom);
    void pub_cloud(const EstimationFrame::ConstPtr frame, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
    void pub_localization_cloud(const EstimationFrame::ConstPtr frame, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
    void wait();

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    std::shared_ptr<Config> config;
    std::unique_ptr<CloudPreprocessor> preprocessor;
    std::unique_ptr<TimeKeeper> time_keeper;
    std::unique_ptr<AsyncOdometryEstimation> odometry_estimation;
    std::unique_ptr<AsyncMapping> mapping;

    double imu_time_offset;
    double lidar_time_offset;
    double acc_scale;
    bool enable_mapping;
    bool enable_tf_publish;

    std::string intensity_field, ring_field;
    std::string base_frame_id, odometry_frame_id, cloud_frame_id;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr high_rate_timer;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robo_odometry_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ivox_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localization_cloud_pub;

    // ---- 高频里程计前向传播状态 ----
    // 只用于在雷达帧之间补帧。传播状态始终“向前”推进，但每拍都从最近一次
    // 校正基准重新预积分：基准之前保留的 IMU 数据在下一帧校正到来时被用于
    // 把旧基准外推到校正时间戳，再与校正位姿做同时间戳的指数平滑。这样既能
    // 压低 10Hz 校正跳变，又不会像旧实现那样把两帧雷达之间的运动一起“抹掉”。
    // IMU 独立存一份给这个传播器（与里程计线程的输入队列解耦）。所有访问都
    // 发生在节点的 executor 单线程里，无需加锁。
    double high_rate_odom_hz{0.0};
    double high_rate_smoothing_tau{0.1};
    double high_rate_smoothing_tau_z{1.0};
    double high_rate_max_window{0.2};
    int high_rate_max_imu_queue{2000};
    std::unique_ptr<IMUIntegration> imu_propagation;
    EstimationFrame::ConstPtr latest_estimation_frame;
    double latest_imu_stamp{0.0};
    bool have_latest_imu{false};

    // 平滑后的传播基准（位姿/速度都定义在 propagation_base_stamp 时刻）。
    bool have_propagation_base{false};
    Eigen::Isometry3d propagation_base_T{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d propagation_base_v{Eigen::Vector3d::Zero()};
    double propagation_base_stamp{0.0};
    // 最近一次已处理的校正帧时间戳。传播基准可能因队列安全阀被推进到比校正帧
    // 更新的时刻，所以不能用 propagation_base_stamp 判断“是否来了新校正帧”。
    double last_correction_stamp{0.0};
    bool have_last_correction{false};
    // 上一次发布的时间戳，只用于保证 TF 时间戳严格单调。
    double last_high_rate_stamp{0.0};
    bool have_last_high_rate_stamp{false};

    // Latest denser localization cloud (LiDAR frame), produced alongside the odometry
    // cloud and transformed by the odometry pose at publish time. Written in
    // lidar_callback, read in timer_callback; both run on the executor's single thread.
    PreprocessedFrame::ConstPtr latest_localization_frame;
};

}

namespace small_glim {

SmallGlimNode::SmallGlimNode(const rclcpp::NodeOptions& options): Node("small_glim", options) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    config = std::make_shared<Config>(this);
    bool debug = config->param<bool>("node.debug");
    if (debug) {
        get_logger().set_level(rclcpp::Logger::Level::Debug);
        logger::debug("node", "enable debug printing");
    }

    enable_tf_publish = config->param<bool>("node.enable_tf_publish");
    enable_mapping = config->param<bool>("node.enable_mapping");
    high_rate_odom_hz = config->param<double>("node.high_rate_odom_hz");
    if (!(high_rate_odom_hz >= 0.0)) {
        logger::warn("node", "invalid high_rate_odom_hz, falling back to 0 Hz (disabled)");
        high_rate_odom_hz = 0.0;
    }
    high_rate_smoothing_tau = std::max(0.0, config->param<double>("node.high_rate_smoothing_tau"));
    high_rate_smoothing_tau_z = std::max(0.0, config->param<double>("node.high_rate_smoothing_tau_z"));
    high_rate_max_window = std::max(0.01, config->param<double>("node.high_rate_max_window"));
    high_rate_max_imu_queue = std::max(64, config->param<int>("node.high_rate_max_imu_queue"));
    if (high_rate_odom_hz > 0.0) {
        imu_propagation = std::make_unique<IMUIntegration>(config);
        // 高频传播每 tick 从过去的估计帧时间开始积分，首段 gap 超限是正常现象，
        // 静音该路径的 gap 警告；主里程计线程仍保留告警。
        imu_propagation->set_suppress_gap_warnings(true);
    }

    imu_time_offset = config->param<double>("node.imu_time_offset");
    lidar_time_offset = config->param<double>("node.lidar_time_offset");
    acc_scale = config->param<double>("node.acc_scale");

    intensity_field = config->param<std::string>("sensors.intensity_field");
    ring_field = config->param<std::string>("sensors.ring_field");
    base_frame_id = config->param<std::string>("node.base_frame_id");
    odometry_frame_id = config->param<std::string>("node.odometry_frame_id");
    cloud_frame_id = config->param<std::string>("node.cloud_frame_id");

    // Preprocessing
    time_keeper = std::make_unique<TimeKeeper>(config);
    preprocessor = std::make_unique<CloudPreprocessor>(config);

    // Odometry estimation
    odometry_estimation = std::make_unique<AsyncOdometryEstimation>(config);

    // Mapping (accumulates from startup; the final map is saved on node destruction, i.e. Ctrl-C)
    if (enable_mapping) {
        mapping = std::make_unique<AsyncMapping>(config);
    }

    // ROS-related
    const std::string imu_sub_topic = config->param<std::string>("node.imu_sub_topic");
    const std::string lidar_sub_topic = config->param<std::string>("node.lidar_sub_topic");
    const std::string odometry_pub_topic = config->param<std::string>("node.odometry_pub_topic");
    const std::string robo_odometry_pub_topic = config->param<std::string>("node.robo_odometry_pub_topic");
    const std::string registered_cloud_pub_topic = config->param<std::string>("node.registered_cloud_pub_topic");
    const std::string localization_cloud_pub_topic = config->param<std::string>("node.localization_cloud_pub_topic");
    const std::string ivox_cloud_pub_topic = config->param<std::string>("node.ivox_cloud_pub_topic");

    // Subscribers
    // SensorDataQoS（best-effort）与 mid360_driver 的发布端一致：传感器流丢帧
    // 比可靠重传更健康。仿真的 ros2_livox_simulation 发布端同为 best-effort 兼容。
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        imu_sub_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); }
    );
    lidar_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_sub_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidar_callback(msg); }
    );
    odometry_pub = create_publisher<nav_msgs::msg::Odometry>(odometry_pub_topic, rclcpp::QoS(1));
    robo_odometry_pub = create_publisher<nav_msgs::msg::Odometry>(robo_odometry_pub_topic, rclcpp::QoS(1));
    registered_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(registered_cloud_pub_topic, rclcpp::QoS(1));
    localization_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(localization_cloud_pub_topic, rclcpp::QoS(1));
    ivox_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(ivox_cloud_pub_topic, rclcpp::QoS(1));

    // Start timer
    timer = create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });

    // 高频里程计补帧：10Hz 雷达帧之间用 IMU 前向传播，降低 MPC/TF 查询看到的
    // 相位滞后（最坏 100ms → ~20ms）。0 关闭。发布走 wall timer 保证均匀节拍。
    if (high_rate_odom_hz > 0.0) {
        high_rate_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / high_rate_odom_hz),
            [this]() { high_rate_timer_callback(); });
        logger::info(
            "node", "high-rate odometry propagation enabled at {:.1f} Hz", high_rate_odom_hz);
    }
}

SmallGlimNode::~SmallGlimNode() {
    logger::info("node", "waiting for odometry estimation");
    odometry_estimation->join();
    if (mapping) {
        std::vector<EstimationFrame::ConstPtr> estimation_results;
        std::vector<EstimationFrame::ConstPtr> target_ivox_frames;
        std::vector<EstimationFrame::ConstPtr> marginalized_frames;
        odometry_estimation->get_results(estimation_results, target_ivox_frames, marginalized_frames);
        for (const auto& marginalized_frame: marginalized_frames) {
            mapping->insert_frame(marginalized_frame);
        }
        logger::info("node", "waiting for mapping");
        mapping->request_finish();
        mapping->join();
    }
}

void SmallGlimNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
    const Eigen::Vector3d linear_acc = acc_scale * utils::convert_to<Eigen::Vector3d>(msg->linear_acceleration);
    const Eigen::Vector3d angular_vel = utils::convert_to<Eigen::Vector3d>(msg->angular_velocity);
    if (!std::isfinite(imu_stamp) || !linear_acc.allFinite() || !angular_vel.allFinite()) {
        logger::warn("node", "skip invalid IMU data (stamp={:.6f})", imu_stamp);
        return;
    }
    if (!time_keeper->validate_imu_stamp(imu_stamp)) {
        logger::warn("node", "skip an invalid IMU data (stamp={})", imu_stamp);
        return;
    }
    odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
    if (imu_propagation) {
        imu_propagation->insert_imu(imu_stamp, linear_acc, angular_vel);
        latest_imu_stamp = imu_stamp;
        have_latest_imu = true;
    }
}

size_t SmallGlimNode::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    RawPoints::Ptr raw_points;
    try {
        raw_points = std::make_shared<RawPoints>(*msg, intensity_field, ring_field);
    } catch (const std::exception& e) {
        logger::warn("node", "failed to extract points from message: {}", e.what());
        return 0;
    }
    raw_points->stamp += lidar_time_offset;
    if (!time_keeper->process(raw_points)) {
        logger::warn("node", "skip invalid LiDAR frame (stamp={:.6f})", raw_points->stamp);
        return 0;
    }
    auto preprocessed = preprocessor->preprocess(raw_points);
    if (!preprocessed) {
        logger::warn("node", "skip LiDAR frame rejected by preprocessing (stamp={:.6f})", raw_points->stamp);
        return 0;
    }
    // Denser cloud for fast_location, decoupled from the odometry downsampling. Produced
    // here (once per accepted frame) and transformed at publish time with the odometry pose.
    // 两档分辨率相同（默认都是 0.05）且 odometry 路径没有额外的 cropbox/outlier/
    // random-grid 过滤时，直接复用里程计的降采样结果：同一帧不再做第二次体素降采样
    // 和全量拷贝（deskew 是拷出新的，不会原地改 raw 点，复用安全）。只有配置了更细
    // 的 localization 分辨率或额外过滤时才单独算。
    const double loc_res = config->param<double>("preprocess.localization_downsample_resolution");
    const double odom_res = config->param<double>("preprocess.downsample_resolution");
    const bool odom_has_extra_filters =
      config->param<bool>("preprocess.enable_cropbox_filter") ||
      config->param<bool>("preprocess.enable_outlier_removal") ||
      config->param<bool>("preprocess.use_random_grid_downsampling");
    if (loc_res <= 0.0) {
        latest_localization_frame = nullptr;
    } else if (std::abs(loc_res - odom_res) < 1e-9 && !odom_has_extra_filters) {
        latest_localization_frame = preprocessed;
    } else {
        latest_localization_frame = preprocessor->preprocess_for_localization(raw_points);
    }
    odometry_estimation->insert_frame(preprocessed);
    const size_t workload = odometry_estimation->workload();
    logger::debug("node", "workload={}", workload);
    return workload;
}

void SmallGlimNode::timer_callback() {
    // 每拍同步一次 ivox 订阅者状态：没人在看就不让里程计线程构建 ivox 帧
    // （构建本身是全量点云拷贝，与发布无关）。
    odometry_estimation->set_output_ivox(ivox_cloud_pub->get_subscription_count() > 0);

    std::vector<EstimationFrame::ConstPtr> estimation_frames;
    std::vector<EstimationFrame::ConstPtr> target_ivox_frames;
    std::vector<EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_frames, target_ivox_frames, marginalized_frames);
    if (!estimation_frames.empty()) {
        latest_estimation_frame = estimation_frames.back();
        pub_odometry(estimation_frames.back());
        // 点云只在有订阅者时才拼装/发布：/Laser_map（RViz 调试）常处于无人订阅
        // 状态，之前每帧都要把几万点拷成 PointCloud2 再序列化，纯浪费。
        if (registered_cloud_pub->get_subscription_count() > 0) {
            pub_cloud(estimation_frames.back(), registered_cloud_pub);
        }
        if (localization_cloud_pub->get_subscription_count() > 0) {
            pub_localization_cloud(estimation_frames.back(), localization_cloud_pub);
        }
    }
    if (!target_ivox_frames.empty() && ivox_cloud_pub->get_subscription_count() > 0) {
        pub_cloud(target_ivox_frames.back(), ivox_cloud_pub);
    }
    if (mapping) {
        for (const auto& marginalized_frame: marginalized_frames) {
            mapping->insert_frame(marginalized_frame);
        }
    }
}

void SmallGlimNode::pub_odometry(const EstimationFrame::ConstPtr frame) {
    const rclcpp::Time stamp(static_cast<int64_t>(frame->stamp * 1e9));
    // Workspace convention inherited from super_lio: the "base_link" odometry actually
    // carries the lidar pose, keeping the odom origin z anchored at the power-on lidar plane.
    const Eigen::Isometry3d T_odom_lidar = frame->T_world_imu * frame->T_lidar_imu.inverse();
    const Eigen::Vector3d v_body = T_odom_lidar.linear().transpose() * frame->v_world_imu;
    // 高频模式下的数据流分工：
    //   /Odometry          → 高频传播流（MPC 的 odom 回退与 TF 同一条流）
    //   /lio/robo/odom     → 10Hz 校正流（fast_location 专用，必须与
    //                        /Laser_map_dense 的校正位姿同流，否则 scan 和 odom
    //                        不一致会让 map→odom 漂移，点云看起来跟着车走）
    //   lidar_odom→base_link TF → 高频传播流
    // 10Hz 硬校正值绝不进入 TF 和 /Odometry，传播值绝不进入 /lio/robo/odom。
    const bool high_rate_enabled = high_rate_odom_hz > 0.0;
    pub_odometry_impl(
        T_odom_lidar, v_body, stamp,
        enable_tf_publish && !high_rate_enabled,   // TF: 关闭高频时由校正流发
        !high_rate_enabled,                        // /Odometry: 关闭高频时由校正流发
        true);                                     // /lio/robo/odom: 始终发校正值
}

void SmallGlimNode::pub_propagated_odometry(
    const Eigen::Isometry3d & T_world_imu, const Eigen::Vector3d & v_world_imu,
    const Eigen::Isometry3d & T_lidar_imu, const rclcpp::Time & stamp
) {
    const Eigen::Isometry3d T_odom_lidar = T_world_imu * T_lidar_imu.inverse();
    const Eigen::Vector3d v_body = T_odom_lidar.linear().transpose() * v_world_imu;
    // 传播流只发 TF 和 /Odometry，不发 /lio/robo/odom。
    pub_odometry_impl(T_odom_lidar, v_body, stamp, enable_tf_publish, true, false);
}

void SmallGlimNode::pub_odometry_impl(
    const Eigen::Isometry3d & T_odom_lidar, const Eigen::Vector3d & v_body,
    const rclcpp::Time & stamp, bool publish_tf, bool publish_primary_odom,
    bool publish_robo_odom
) {
    // Dynamic lidar_odom -> base_link TF. This is the only edge connecting the odom tree
    // (map->odom->lidar_odom) to the robot tree (base_link->livox_frame/imu_link/...);
    // dropping it splits the TF graph and breaks costmap/amcl localization, not just RViz.
    if (publish_tf) {
        geometry_msgs::msg::TransformStamped tf_base_to_odom;
        tf_base_to_odom.header.frame_id = odometry_frame_id;
        tf_base_to_odom.child_frame_id = base_frame_id;
        tf_base_to_odom.header.stamp = stamp;
        utils::convert(T_odom_lidar, tf_base_to_odom.transform);
        tf_broadcaster->sendTransform(tf_base_to_odom);
    }

    if (!publish_primary_odom && !publish_robo_odom) {
        return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odometry_frame_id;
    odom.child_frame_id = base_frame_id;
    odom.header.stamp = stamp;
    utils::convert(T_odom_lidar, odom.pose.pose);
    utils::convert(v_body, odom.twist.twist.linear);
    if (publish_primary_odom) {
        odometry_pub->publish(odom);
    }
    if (publish_robo_odom) {
        robo_odometry_pub->publish(odom);
    }
}

void SmallGlimNode::high_rate_timer_callback() {
    if (!imu_propagation || !latest_estimation_frame || !have_latest_imu) {
        return;
    }

    const auto & frame = latest_estimation_frame;
    const gtsam::imuBias::ConstantBias bias(frame->imu_bias);

    // ---- 校正帧状态机：首帧 / 新帧 / 乱序帧 / 迟到帧 ----
    // 新校正帧是否出现，以 last_correction_stamp 判断，而不是 propagation_base_stamp：
    // 队列安全阀可能已经把传播基准推进到比最新校正帧更晚的时刻。
    auto reanchor = [&](const double stamp, const Eigen::Isometry3d & T,
                        const Eigen::Vector3d & v) {
        propagation_base_T = T;
        propagation_base_v = v;
        propagation_base_stamp = stamp;
        have_propagation_base = true;

        // 丢弃基准时间之前的 IMU，后续预积分从新基准时间开始。
        size_t num_stale = 0;
        imu_propagation->integrate_imu(stamp, stamp, bias, &num_stale);
        if (num_stale > 0) {
            imu_propagation->erase_imu_data(num_stale);
        }
    };

    if (!have_propagation_base || !have_last_correction) {
        // 正常首帧直接作为基准。
        reanchor(frame->stamp, frame->T_world_imu, frame->v_world_imu);
        last_correction_stamp = frame->stamp;
        have_last_correction = true;
    } else if (frame->stamp < last_correction_stamp) {
        // 校正帧时间戳回退说明估计线程被重置过，直接吸附新值。
        logger::warn(
            "node",
            "high-rate estimation frame timestamp rewind ({:.6f} -> {:.6f}), re-anchoring",
            last_correction_stamp,
            frame->stamp);
        reanchor(frame->stamp, frame->T_world_imu, frame->v_world_imu);
        last_correction_stamp = frame->stamp;
    } else if (frame->stamp > last_correction_stamp) {
        const double new_correction_stamp = frame->stamp;
        last_correction_stamp = new_correction_stamp;

        if (new_correction_stamp < propagation_base_stamp) {
            // 迟到校正：队列安全阀已经把基准推到比校正帧更晚，IMU 历史已丢，
            // 无法向后外推，这一帧只能硬吸附，下一帧恢复正常的同时间戳平滑。
            logger::warn(
                "node",
                "late high-rate correction (correction={:.6f} < base={:.6f}), hard re-anchoring",
                new_correction_stamp,
                propagation_base_stamp);
            reanchor(new_correction_stamp, frame->T_world_imu, frame->v_world_imu);
        } else {
            // ---- 新校正帧：同一时间戳上的预测值 vs 校正值 ----
            // 关键修复：不能把“旧基准时刻的位姿”直接和“新校正时刻的位姿”插值。
            // 先用保留在传播队列里的 IMU 把旧基准外推到 new_correction_stamp，得到
            // pred_at_correction，再与校正值平滑；帧间运动由 IMU 完整保留，
            // 平滑只吸收校正残差。
            const double old_base_stamp = propagation_base_stamp;
            const gtsam::NavState old_base_state(
                gtsam::Pose3(propagation_base_T.matrix()), propagation_base_v);

            Eigen::Isometry3d pred_T = propagation_base_T;
            Eigen::Vector3d pred_v = propagation_base_v;
            size_t num_integrated = 0;
            imu_propagation->integrate_imu(
                old_base_stamp, new_correction_stamp, bias, &num_integrated);
            const auto predicted_at_correction =
                imu_propagation->integrated_measurements().predict(old_base_state, bias);
            if (predicted_at_correction.pose().matrix().allFinite()
                && predicted_at_correction.velocity().allFinite()) {
                pred_T = Eigen::Isometry3d(predicted_at_correction.pose().matrix());
                pred_v = predicted_at_correction.velocity();
            }

            // 已经积分到 new_correction_stamp 的 IMU 不再需要：后续传播从新基准时刻重新开始。
            if (num_integrated > 0) {
                imu_propagation->erase_imu_data(num_integrated);
            }

            // 平滑系数按“两次校正的时间间隔”计算，与 high_rate_odom_hz 解耦。
            // 旧实现用单个高频 tick 的墙钟 dt，100Hz 时每个校正只吸收 ~9.5%，
            // 相当于把 10Hz 校正跳变几乎原样送进 TF。
            // z 轴单独用更慢的 tau：地面机器人的 z 本来只该缓慢变化，而 LIO 在
            // 运动/旋转时高度校正常有 cm 级抖动；若与 xy/yaw 共用 0.1s，这些抖动
            // 会被逐拍吸收进高频 TF，表现为“反复下沉又弹回地面”。
            const double correction_dt =
                std::max(new_correction_stamp - old_base_stamp, 1e-3);
            const double alpha = high_rate_smoothing_tau > 0.0
                ? 1.0 - std::exp(-correction_dt / high_rate_smoothing_tau)
                : 1.0;
            const double alpha_z = high_rate_smoothing_tau_z > 0.0
                ? 1.0 - std::exp(-correction_dt / high_rate_smoothing_tau_z)
                : 1.0;
            const double a = std::clamp(alpha, 0.0, 1.0);
            const double a_z = std::clamp(alpha_z, 0.0, 1.0);

            Eigen::Quaterniond q_pred(pred_T.linear());
            Eigen::Quaterniond q_corr(frame->T_world_imu.linear());
            if (q_pred.dot(q_corr) < 0.0) {
                q_corr.coeffs() *= -1.0;
            }

            Eigen::Isometry3d blended = Eigen::Isometry3d::Identity();
            blended.linear() = q_pred.slerp(a, q_corr).normalized().toRotationMatrix();
            blended.translation() =
                pred_T.translation() * (1.0 - a) +
                frame->T_world_imu.translation() * a;
            blended.translation().z() =
                pred_T.translation().z() * (1.0 - a_z) +
                frame->T_world_imu.translation().z() * a_z;

            propagation_base_T = blended;
            propagation_base_v = pred_v * (1.0 - a) + frame->v_world_imu * a;
            propagation_base_v.z() =
                pred_v.z() * (1.0 - a_z) + frame->v_world_imu.z() * a_z;
            propagation_base_stamp = new_correction_stamp;
        }
    }

    const gtsam::NavState base_state(
        gtsam::Pose3(propagation_base_T.matrix()), propagation_base_v);

    // ---- 积分外推到最新 IMU 时间 ----
    // 外推窗口设上限：估计处理延迟大时，长窗口纯 IMU 外推会放大漂移/抖动。
    double end_time = latest_imu_stamp;
    if (end_time - propagation_base_stamp > high_rate_max_window) {
        end_time = propagation_base_stamp + high_rate_max_window;
    }

    Eigen::Isometry3d out_T_world_imu = propagation_base_T;
    Eigen::Vector3d out_v_world_imu = propagation_base_v;
    double out_stamp = propagation_base_stamp;

    if (end_time > propagation_base_stamp) {
        size_t num_integrated = 0;
        imu_propagation->integrate_imu(
            propagation_base_stamp, end_time, bias, &num_integrated);
        const auto predicted =
            imu_propagation->integrated_measurements().predict(base_state, bias);
        if (predicted.pose().matrix().allFinite() && predicted.velocity().allFinite()) {
            out_T_world_imu = Eigen::Isometry3d(predicted.pose().matrix());
            out_v_world_imu = predicted.velocity();
            out_stamp = end_time;
        } else {
            logger::warn(
                "node",
                "high-rate odometry prediction non-finite, falling back to smoothed base");
        }
        // 注意：这里刻意不出队。当前基准到下一校正帧之间的 IMU 必须留在队列里，
        // 下一帧校正到来时才能把基准重新外推到校正时间戳；校正分支里会统一出队。
    }

    // 安全阀：正常 10Hz 校正会让队列在校正分支持续出队，体积有界；但如果定位
    // 卡住（长时间没有新校正帧）而 IMU 还在进来，队列会无上限增长。此时把基准
    // 重锚到最近一次传播输出并丢掉更早的 IMU——若之后有迟到校正帧，rewind 分支
    // 会硬吸附恢复。
    if (out_stamp > propagation_base_stamp
        && imu_propagation->imu_queue_size() > static_cast<size_t>(high_rate_max_imu_queue)) {
        logger::warn(
            "node",
            "high-rate IMU propagation queue exceeded {} samples, re-anchoring at t={:.6f}",
            high_rate_max_imu_queue,
            out_stamp);
        propagation_base_T = out_T_world_imu;
        propagation_base_v = out_v_world_imu;
        propagation_base_stamp = out_stamp;

        size_t num_stale = 0;
        imu_propagation->integrate_imu(
            propagation_base_stamp, propagation_base_stamp, bias, &num_stale);
        if (num_stale > 0) {
            imu_propagation->erase_imu_data(num_stale);
        }
    }

    // 时间戳必须严格单调：IMU 时间戳偶发乱序/重复时直接跳过这一拍，tf2 会
    // 继续使用上一帧 TF，而不是拿到乱序时间戳后做错误插值。
    if (have_last_high_rate_stamp && !(out_stamp > last_high_rate_stamp)) {
        return;
    }
    last_high_rate_stamp = out_stamp;
    have_last_high_rate_stamp = true;

    pub_propagated_odometry(
        out_T_world_imu, out_v_world_imu, frame->T_lidar_imu,
        rclcpp::Time(static_cast<int64_t>(out_stamp * 1e9)));
}

void SmallGlimNode::pub_cloud(
    const EstimationFrame::ConstPtr frame,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher
) {
    const size_t num_points = frame->frame->num_points;
    const auto& points = frame->frame->points;
    sensor_msgs::msg::PointCloud2 msg;
    // Points are in the odom frame; the workspace publishes a static identity odom->world,
    // and downstream (fast_location) expects the cloud stamped with the world frame.
    msg.header.frame_id = cloud_frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->stamp * 1e9));
    msg.height = 1;
    msg.width = static_cast<uint32_t>(num_points);
    msg.is_dense = true;
    msg.point_step = 16;
    msg.row_step = static_cast<uint32_t>(16 * num_points);
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    sensor_msgs::msg::PointField field_intensity;
    field_intensity.name = "intensity";
    field_intensity.offset = 12;
    field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_intensity.count = 1;
    msg.fields = {field_x, field_y, field_z, field_intensity};
    msg.data.resize(msg.row_step * msg.height);
    // Downstream PCL consumers (fast_location, rviz) convert these clouds into
    // pcl::PointCloudXYZI and PCL warns every frame when the intensity field is missing.
    // Emit real intensity when available (mid360 real hardware), otherwise zeros (sim
    // plugin has no intensity).
    const bool has_intensity = frame->frame->has_intensities();
    for (size_t i = 0; i < num_points; i++) {
        Eigen::Vector3f pt = points[i](Eigen::seq(0, 2)).cast<float>();
        if (frame->frame_type != FrameType::WORLD) pt = frame->T_world_frame().cast<float>() * pt;
        const float intensity = has_intensity ? static_cast<float>(frame->frame->intensities[i]) : 0.0f;
        std::memcpy(msg.data.data() + i * 16, pt.data(), sizeof(pt));
        std::memcpy(msg.data.data() + i * 16 + 12, &intensity, sizeof(intensity));
    }
    publisher->publish(msg);
}

void SmallGlimNode::pub_localization_cloud(
    const EstimationFrame::ConstPtr frame,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher
) {
    const auto& dense = latest_localization_frame;
    if (!dense || dense->points.empty()) {
        return;
    }
    const size_t num_points = dense->points.size();
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = cloud_frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->stamp * 1e9));
    msg.height = 1;
    msg.width = static_cast<uint32_t>(num_points);
    msg.is_dense = true;
    msg.point_step = 16;
    msg.row_step = static_cast<uint32_t>(16 * num_points);
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    sensor_msgs::msg::PointField field_intensity;
    field_intensity.name = "intensity";
    field_intensity.offset = 12;
    field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_intensity.count = 1;
    msg.fields = {field_x, field_y, field_z, field_intensity};
    msg.data.resize(msg.row_step * msg.height);
    // Localization cloud is kept in the LiDAR frame; transform with the (smoothed) odometry
    // pose. No per-point deskew here: fast_location voxel-downsamples to 0.2 m and stacks
    // several frames, so the whole-frame approximation (a "global shutter" cloud) is well
    // within its tolerance at nav speeds.
    const Eigen::Isometry3f T_world_lidar = frame->T_world_lidar.cast<float>();
    const bool has_intensity = !dense->intensities.empty();
    for (size_t i = 0; i < num_points; i++) {
        Eigen::Vector3f pt = dense->points[i].head<3>().cast<float>();
        pt = T_world_lidar * pt;
        const float intensity = has_intensity ? static_cast<float>(dense->intensities[i]) : 0.0f;
        std::memcpy(msg.data.data() + i * 16, pt.data(), sizeof(pt));
        std::memcpy(msg.data.data() + i * 16 + 12, &intensity, sizeof(intensity));
    }
    publisher->publish(msg);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_glim::SmallGlimNode)
