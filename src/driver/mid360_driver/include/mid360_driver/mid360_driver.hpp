/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
// asio/awaitable.hpp uses std::exchange without including <utility>; C++23/gcc-13
// no longer pulls it in transitively. Keep this include BEFORE asio.hpp.
#include <utility>
#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace mid360_driver {

    struct DriverRobustnessConfig {
        bool validate_crc;
        double max_packet_time_jump;
        double max_packet_time_span;
        double max_point_range;
        double max_imu_acc;
        double max_imu_gyro;
        double min_drop_log_interval;
    };

    struct Point {
        double timestamp;
        float x, y, z;
        float intensity;
    };

    struct ImuMsg {
        double timestamp;
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct IpAddressHasher {
        std::size_t operator()(const asio::ip::address &addr) const noexcept;
    };

    class Mid360Driver {
    private:
        std::atomic<bool> is_running = true;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        DriverRobustnessConfig robustness_config;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> last_lidar_timestamp_map;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> last_imu_timestamp_map;
        // 连续被判 implausible 的起始时刻（按流、按源地址计，空表 = 无待重锚）。
        // 旧实现里参考时间戳只在收包成功时更新，一旦某包 diff 越界，参考值就
        // 永久冻结在旧时刻：雷达 PTP 重同步 / NTP 步进 / 雷达重启换基准后，
        // 后续每个包都越界，点云+IMU 无声全灭直到进程重启。现在坏包连续持续
        // 满时长阈值（kReanchorAfter）后把锚重置到当前包。用墙钟时长而不是
        // 包数——Mid-360s 的 IMU 输出率可配成 50~500Hz、ESC 慢速模式会降低
        // 点云包率，包数阈值会随速率配置变形。
        std::unordered_map<asio::ip::address, std::chrono::steady_clock::time_point, IpAddressHasher> lidar_implausible_since;
        std::unordered_map<asio::ip::address, std::chrono::steady_clock::time_point, IpAddressHasher> imu_implausible_since;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud;
        std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu;

    public:
        Mid360Driver(asio::io_context &io_context,
                     const asio::ip::address &host_ip,
                     DriverRobustnessConfig robustness_config,
                     std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud,
                     std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu);

        ~Mid360Driver();

        void stop();

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace mid360_driver
