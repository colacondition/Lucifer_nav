/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "mid360_driver/mid360_driver.hpp"
#include <limits>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>

namespace mid360_driver {

    constexpr std::size_t MAX_PENDING_POINTS = 200000;
    constexpr std::size_t MAX_PENDING_IMU = 1000;

    /// 一颗雷达"待发布点云"的统计（供驱动内部双雷达融合判断对齐用）
    struct PendingFrameInfo {
        std::size_t count = 0;
        double min_timestamp = std::numeric_limits<double>::infinity();
        double max_timestamp = -std::numeric_limits<double>::infinity();
    };

    class LidarPublisher {
    private:
        std::vector<Point> points_wait_to_publish;
        std::vector<ImuMsg> imu_wait_to_publish;
        std::vector<Point> points_to_publish;
        std::vector<ImuMsg> imu_to_publish;
        bool is_initialized = false;      // 点云发布器已创建
        bool is_imu_initialized = false;  // IMU 发布器已创建（与点云解耦：融合模式下
                                          // 只发合并点云时，单路点云发布器不该被创建）
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;
        DriverRobustnessConfig robustness_config;

    public:
        LidarPublisher() = default;
        void configure_robustness(const DriverRobustnessConfig &config);
        void ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic);
        void ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip);
        /// 只创建 IMU 发布器。suffixed=false 时用不带 IP 后缀的话题
        /// （融合模式下的主雷达走这条，下游 small_glim 订的 /livox/imu 才拿得到数据）。
        void ensure_imu_initialized(rclcpp::Node &node, const std::string &imu_topic,
                                    const asio::ip::address &lidar_ip, bool suffixed);
        bool is_imu_initialized_suffixed = false;
        void on_receive_pointcloud(const std::vector<Point> &points);
        void on_receive_imu(const ImuMsg &imu_msg);
        void prepare_pointcloud_to_publish();
        void prepare_imu_to_publish();
        void publish_pointcloud(const std::string &frame_id) const;
        void publish_imu(const std::string &frame_id) const;

        // ---- 供"驱动内部融合"使用的接口（默认路径完全不受影响） ----
        const std::vector<Point> &pending_pointcloud() const { return points_to_publish; }
        PendingFrameInfo pending_info() const;
        void discard_pending_pointcloud() { points_to_publish.clear(); }
        bool has_publisher() const { return static_cast<bool>(pointcloud_publisher); }
        void publish_pointcloud_msg(const sensor_msgs::msg::PointCloud2 &msg) const {
            pointcloud_publisher->publish(msg);
        }
    };

    class Mid360DriverNode : public rclcpp::Node {
    private:
        asio::io_context io_context;
        std::mutex multi_lidar_mutex_;
        std::thread io_thread;
        std::unique_ptr<mid360_driver::Mid360Driver> mid360_driver;
        LidarPublisher lidar_publisher;
        std::unordered_map<asio::ip::address, LidarPublisher, IpAddressHasher> multi_lidar_publishers;
        rclcpp::TimerBase::SharedPtr publish_pointcloud_timer;
        rclcpp::TimerBase::SharedPtr publish_imu_timer;

        // ================= 驱动内部双雷达融合（可选，默认关闭） =================
        // 设计原则：只在 enable_internal_lidar_merge=true 时生效；为 false 时
        // 代码路径与原来逐位一致，绝不改变既有导航链路的行为。
        DriverRobustnessConfig robustness_config_{};
        bool enable_internal_lidar_merge = false;
        std::string merge_primary_ip;      // 主雷达 IP（空 = 取 IP 数值最小的那台）
        std::string merge_output_topic;    // 合并后话题（建议 /livox/lidar，下游零改动）
        std::string merge_frame_id;        // 合并后 frame_id（建议 livox_frame）
        // 2 台雷达帧头时间差上限。注意我们不是按"自然帧边界"发布，而是共用
        // 一个定时器把两路各自累积的 100 ms 缓冲拼起来，所以两路最小时间戳天然
        // 会差几毫秒；5 ms 这种按帧边界设计的窗口在这里会误杀一半的帧。
        // 20 ms ≪ 100 ms 帧周期，绝不会配错帧，又足够容忍定时器抖动。
        double merge_max_interval_ms = 20.0;
        double merge_extrinsic[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x,y,z,roll,pitch,yaw（副→主）
        // 融合时裁掉"离主雷达太近"的点（自车/工装回波）。实测我们的点云里
        // 2/3 的点在 30 cm 内，不裁的话合并帧 38k 点里大部分是自车，既浪费
        // RViz 渲染，也白送给 LIO/代价地图。参考 HERO_2026 的 blind_distance。
        // 默认 0.0 = 不裁（保持原有行为）。
        double merge_min_point_range = 0.0;
        /// 判断某台设备是不是融合的主雷达（主雷达的 IMU 用不带 IP 后缀的话题）
        bool is_primary_device(const asio::ip::address &lidar_ip) const;
        bool merge_publish_individual = true; // 是否仍按设备发单帧话题（标定需要 true）
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merge_publisher;
        std::size_t merge_published_frames = 0;
        std::size_t merge_dropped_frames = 0;
        double last_merge_dt_ms = 0.0;
        // 降级：某颗雷达掉线超过这个时长，就只用还活着的数据继续发（否则全局失明）
        double merge_degraded_timeout_ms = 1000.0;
        std::chrono::steady_clock::time_point last_full_merge_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_merge_log = std::chrono::steady_clock::now();

        /// 返回 true 表示这一拍已经发过合并帧（调用方决定是否还要发单帧）
        bool try_publish_merged_pointcloud(
                std::vector<std::pair<asio::ip::address, LidarPublisher *>> &snapshot);

    public:
        explicit Mid360DriverNode(const rclcpp::NodeOptions &options);
        ~Mid360DriverNode() override;
    };

}// namespace mid360_driver
