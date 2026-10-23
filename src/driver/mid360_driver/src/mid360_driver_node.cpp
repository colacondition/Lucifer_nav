/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "mid360_driver/mid360_driver_node.hpp"
#include <cerrno>
#include <system_error>
#include <algorithm>
#include <cmath>

namespace mid360_driver {

    void LidarPublisher::configure_robustness(const DriverRobustnessConfig &config) {
        robustness_config = config;
    }

    void LidarPublisher::ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic) {
        if (!is_initialized) {
            // 传感器流用 SensorDataQoS（best-effort）：丢一帧点云比 TCP 式重传堆队列
            // 更健康。下游 small_glim / cpp_lidar_filter 的订阅同为 SensorDataQoS。
            pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SensorDataQoS());
            imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
            is_imu_initialized = true;
            is_initialized = true;
        }
    }

    void LidarPublisher::ensure_imu_initialized(rclcpp::Node &node, const std::string &imu_topic,
                                                const asio::ip::address &lidar_ip, bool suffixed) {
        if (is_imu_initialized) {
            return;
        }
        std::string topic = imu_topic;
        if (suffixed) {
            const auto lidar_ip_bytes = lidar_ip.to_v4().to_bytes();
            for (const auto byte: lidar_ip_bytes) {
                topic.push_back('_');
                topic.append(std::to_string(static_cast<int>(byte)));
            }
        }
        imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS());
        is_imu_initialized = true;
        is_imu_initialized_suffixed = suffixed;
    }

    bool Mid360DriverNode::is_primary_device(const asio::ip::address &lidar_ip) const {
        if (!enable_internal_lidar_merge) {
            return false;
        }
        if (merge_primary_ip.empty()) {
            return false;   // 没显式指定就算了，保持原行为
        }
        return lidar_ip == asio::ip::make_address(merge_primary_ip);
    }

    void LidarPublisher::ensure_initialized(rclcpp::Node &node, const std::string &lidar_topic, const std::string &imu_topic, const asio::ip::address &lidar_ip) {
        if (!is_initialized) {
            auto lidar_ip_bytes = lidar_ip.to_v4().to_bytes();
            std::string lidar_ip_str;
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[0])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[1])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[2])));
            lidar_ip_str.push_back('_');
            lidar_ip_str.append(std::to_string(static_cast<int>(lidar_ip_bytes[3])));
            pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic + lidar_ip_str, rclcpp::SensorDataQoS());
            imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic + lidar_ip_str, rclcpp::SensorDataQoS());
            is_imu_initialized = true;
            is_initialized = true;
        }
    }

    void LidarPublisher::on_receive_pointcloud(const std::vector<Point> &points) {
        if (points_wait_to_publish.size() + points.size() > MAX_PENDING_POINTS) {
            points_wait_to_publish.clear();
        }
        points_wait_to_publish.reserve(points_wait_to_publish.size() + points.size());
        std::copy(points.begin(), points.end(), std::back_inserter(points_wait_to_publish));
    }

    void LidarPublisher::on_receive_imu(const ImuMsg &imu_msg) {
        if (imu_wait_to_publish.size() >= MAX_PENDING_IMU) {
            imu_wait_to_publish.clear();
        }
        imu_wait_to_publish.push_back(imu_msg);
    }

    void LidarPublisher::prepare_pointcloud_to_publish() {
        std::swap(points_wait_to_publish, points_to_publish);
        points_wait_to_publish.clear();
    }

    void LidarPublisher::prepare_imu_to_publish() {
        std::swap(imu_wait_to_publish, imu_to_publish);
        imu_wait_to_publish.clear();
    }

    namespace {

        /// RPY(ZYX) → 3x3 旋转矩阵，行主序。与标定工具的输出约定一致。
        void rpy_to_matrix(const double rpy[3], double out[3][3]) {
            const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
            const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
            const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
            out[0][0] = cy * cp;
            out[0][1] = cy * sp * sr - sy * cr;
            out[0][2] = cy * sp * cr + sy * sr;
            out[1][0] = sy * cp;
            out[1][1] = sy * sp * sr + cy * cr;
            out[1][2] = sy * sp * cr - cy * sr;
            out[2][0] = -sp;
            out[2][1] = cp * sr;
            out[2][2] = cp * cr;
        }

        /// 按驱动既有的字段布局生成 PointCloud2。
        /// 返回 false 表示这一帧该丢（空帧 / 时间戳跨度非法 / 无有效点）。
        bool build_pointcloud_message(
                const std::vector<Point> &points,
                const std::string &frame_id,
                const DriverRobustnessConfig &config,
                sensor_msgs::msg::PointCloud2 &msg,
                const char *tag,
                double span_scale) {
            if (points.empty()) {
                return false;
            }
            msg = sensor_msgs::msg::PointCloud2();
            msg.header.frame_id = frame_id;
            msg.height = 1;
            msg.fields.reserve(5);
            sensor_msgs::msg::PointField field;
            field.name = "x";
            field.offset = 0;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            msg.fields.push_back(field);
            field.name = "y";
            field.offset = 4;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            msg.fields.push_back(field);
            field.name = "z";
            field.offset = 8;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            msg.fields.push_back(field);
            field.name = "intensity";
            field.offset = 12;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            msg.fields.push_back(field);
            field.name = "timestamp";
            field.offset = 16;
            field.datatype = sensor_msgs::msg::PointField::FLOAT64;
            field.count = 1;
            msg.fields.push_back(field);
            msg.is_bigendian = false;
            msg.point_step = 24;
            msg.data.resize(points.size() * msg.point_step);

            const double max_range2 = config.max_point_range * config.max_point_range;
            double min_timestamp = std::numeric_limits<double>::max();
            double max_timestamp = std::numeric_limits<double>::lowest();
            std::size_t valid_count = 0;
            for (const auto &point: points) {
                if (std::isfinite(point.timestamp)) {
                    min_timestamp = std::min(min_timestamp, point.timestamp);
                    max_timestamp = std::max(max_timestamp, point.timestamp);
                }
                const double range2 = point.x * point.x + point.y * point.y + point.z * point.z;
                if (!std::isfinite(point.timestamp)
                    || !std::isfinite(point.x)
                    || !std::isfinite(point.y)
                    || !std::isfinite(point.z)
                    || !std::isfinite(point.intensity)
                    || range2 > max_range2) {
                    continue;
                }
                const std::size_t offset = valid_count * msg.point_step;
                std::memcpy(msg.data.data() + offset, &point.x, 4 * sizeof(float));
                std::memcpy(msg.data.data() + offset + 16, &point.timestamp, sizeof(double));
                ++valid_count;
            }

            if (!std::isfinite(min_timestamp) || !std::isfinite(max_timestamp)) {
                return false;
            }
            const double frame_time_span = max_timestamp - min_timestamp;
            if (frame_time_span < 0.0 || frame_time_span > config.max_packet_time_span * span_scale) {
                RCLCPP_WARN(
                    rclcpp::get_logger("mid360_driver"),
                    "drop point cloud (%s) with invalid timestamp span: min=%.6f max=%.6f span=%.6f points=%zu",
                    tag,
                    min_timestamp,
                    max_timestamp,
                    frame_time_span,
                    points.size()
                );
                return false;
            }
            if (valid_count == 0) {
                return false;
            }
            msg.header.stamp.sec = static_cast<int32_t>(std::floor(min_timestamp));
            msg.header.stamp.nanosec = static_cast<uint32_t>((min_timestamp - msg.header.stamp.sec) * 1e9);
            msg.width = static_cast<uint32_t>(valid_count);
            msg.row_step = msg.width * msg.point_step;
            msg.data.resize(msg.row_step * msg.height);
            msg.is_dense = true;
            return true;
        }

    }// namespace

    PendingFrameInfo LidarPublisher::pending_info() const {
        PendingFrameInfo info;
        info.count = points_to_publish.size();
        for (const auto &point: points_to_publish) {
            if (!std::isfinite(point.timestamp)) {
                continue;
            }
            info.min_timestamp = std::min(info.min_timestamp, point.timestamp);
            info.max_timestamp = std::max(info.max_timestamp, point.timestamp);
        }
        return info;
    }

    void LidarPublisher::publish_pointcloud(const std::string &frame_id) const {
        if (points_to_publish.empty() || !pointcloud_publisher) {
            return;   // 发布器可能没创建（融合模式下不发单路），别解空指针
        }

        sensor_msgs::msg::PointCloud2 msg;
        if (!build_pointcloud_message(points_to_publish, frame_id, robustness_config, msg, "single", 2.0)) {
            return;
        }
        pointcloud_publisher->publish(msg);
    }

    void LidarPublisher::publish_imu(const std::string &frame_id) const {
        if (!imu_publisher) {
            return;
        }
        for (const auto &imu: imu_to_publish) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp.sec = static_cast<int32_t>(std::floor(imu.timestamp));
            msg.header.stamp.nanosec = static_cast<uint32_t>((imu.timestamp - msg.header.stamp.sec) * 1e9);
            msg.header.frame_id = frame_id;
            msg.angular_velocity.x = imu.angular_velocity_x;
            msg.angular_velocity.y = imu.angular_velocity_y;
            msg.angular_velocity.z = imu.angular_velocity_z;
            msg.linear_acceleration.x = imu.linear_acceleration_x;
            msg.linear_acceleration.y = imu.linear_acceleration_y;
            msg.linear_acceleration.z = imu.linear_acceleration_z;
            imu_publisher->publish(msg);
        }
    }

    bool Mid360DriverNode::try_publish_merged_pointcloud(
            std::vector<std::pair<asio::ip::address, LidarPublisher *>> &snapshot) {
        if (!enable_internal_lidar_merge || snapshot.size() < 2U) {
            return false;
        }

        // ---- 选主雷达：显式指定优先，否则取 IP 数值最小的那台（确定性） ----
        std::size_t primary = 0;
        if (!merge_primary_ip.empty()) {
            const auto want = asio::ip::make_address(merge_primary_ip);
            bool found = false;
            for (std::size_t i = 0; i < snapshot.size(); ++i) {
                if (snapshot[i].first == want) {
                    primary = i;
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "内部融合：找不到主雷达 %s，本拍不融合", merge_primary_ip.c_str());
                return false;
            }
        } else {
            for (std::size_t i = 1; i < snapshot.size(); ++i) {
                if (snapshot[i].first < snapshot[primary].first) {
                    primary = i;
                }
            }
        }

        // ---- 收集"这一拍有数据"的设备 ----
        std::vector<std::size_t> available;
        available.reserve(snapshot.size());
        for (std::size_t i = 0; i < snapshot.size(); ++i) {
            if (!snapshot[i].second->pending_pointcloud().empty()) {
                available.push_back(i);
            }
        }
        if (available.empty()) {
            return false;   // 全世界都没数据，等下一拍
        }
        const bool all_available = available.size() == snapshot.size();

        // 时间对齐：帧头（最小点时间）差必须在窗口内
        const PendingFrameInfo primary_info = snapshot[primary].second->pending_info();
        double worst_dt_ms = 0.0;
        if (all_available) {
            for (const auto i: available) {
                if (i == primary) {
                    continue;
                }
                const PendingFrameInfo info = snapshot[i].second->pending_info();
                worst_dt_ms = std::max(worst_dt_ms,
                                       std::abs(info.min_timestamp - primary_info.min_timestamp) * 1e3);
            }
        }

        // ---- 降级模式：某颗雷达掉线时，不能一帧都不发（否则 LIO/代价地图全盲）。
        // 短暂抖动（< merge_degraded_timeout_ms）仍然不发半帧；超过就只用"还活着"的
        // ones 拼一帧出去，并节流告警。实测触发场景：A 的网线抖了一下，推流器卡住，
        // 设备侧报 HMS 0x0401（360S 专用"链路断开过"）。
        const auto merge_now = std::chrono::steady_clock::now();
        const bool window_ok = all_available && worst_dt_ms <= merge_max_interval_ms;
        if (!window_ok) {
            const auto degraded_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    merge_now - last_full_merge_time).count();
            if (degraded_ms < static_cast<long>(merge_degraded_timeout_ms)) {
                // 还在宽限期内：不发半帧，但把更旧的那一路丢掉（避免死循环）
                if (all_available) {
                    std::size_t newest = primary;
                    for (const auto i: available) {
                        if (snapshot[i].second->pending_info().max_timestamp
                            > snapshot[newest].second->pending_info().max_timestamp) {
                            newest = i;
                        }
                    }
                    for (const auto i: available) {
                        if (i != newest) {
                            snapshot[i].second->discard_pending_pointcloud();
                        }
                    }
                }
                return false;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "内部融合降级：%s，只用还活着的 %zu/%zu 路发布"
                                 "（已持续 %ld ms；检查网线/供电，设备侧 HMS 可能报 0x0401）",
                                 all_available ? "两路帧头差超窗" : "有雷达没数据",
                                 available.size(), snapshot.size(), degraded_ms);
        } else {
            last_full_merge_time = merge_now;
        }

        if (false) {
            // 超窗：宁可丢一帧也不发半帧。但**只丢更旧的那一路**——若把新鲜的那路
            // 也丢了，下一拍它就会落后整整一帧，形成"丢一帧、错一拍"的死循环
            // （实测会把合并帧率压到 3~5 Hz）。上游 navi_minco_bit 也是丢旧帧。
            std::size_t newest = primary;
            for (std::size_t i = 0; i < snapshot.size(); ++i) {
                if (snapshot[i].second->pending_info().max_timestamp
                    > snapshot[newest].second->pending_info().max_timestamp) {
                    newest = i;
                }
            }
            for (std::size_t i = 0; i < snapshot.size(); ++i) {
                if (i != newest) {
                    snapshot[i].second->discard_pending_pointcloud();
                }
            }
            ++merge_dropped_frames;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "内部融合丢弃一帧：两台雷达帧头差 %.2f ms > %.2f ms（累计丢 %zu）",
                                 worst_dt_ms, merge_max_interval_ms, merge_dropped_frames);
            return false;
        }

        // ---- 合并：主雷达点原样，其余设备变换到主雷达系，再按每点时间排序 ----
        double rot[3][3];
        const double rpy[3] = {merge_extrinsic[3], merge_extrinsic[4], merge_extrinsic[5]};
        rpy_to_matrix(rpy, rot);
        std::vector<Point> merged;
        merged.reserve(primary_info.count * snapshot.size());
        for (const auto i: available) {
            const auto &points = snapshot[i].second->pending_pointcloud();
            if (i == primary) {
                if (merge_min_point_range <= 0.0) {
                    merged.insert(merged.end(), points.begin(), points.end());
                } else {
                    const double r2 = merge_min_point_range * merge_min_point_range;
                    for (const auto &point: points) {
                        if (static_cast<double>(point.x) * point.x
                            + static_cast<double>(point.y) * point.y
                            + static_cast<double>(point.z) * point.z >= r2) {
                            merged.push_back(point);
                        }
                    }
                }
                continue;
            }
            for (const auto &point: points) {
                const double x = static_cast<double>(point.x);
                const double y = static_cast<double>(point.y);
                const double z = static_cast<double>(point.z);
                Point transformed = point;
                transformed.x = static_cast<float>(
                        rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + merge_extrinsic[0]);
                transformed.y = static_cast<float>(
                        rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + merge_extrinsic[1]);
                transformed.z = static_cast<float>(
                        rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + merge_extrinsic[2]);
                if (merge_min_point_range > 0.0) {
                    const double tx = static_cast<double>(transformed.x);
                    const double ty = static_cast<double>(transformed.y);
                    const double tz = static_cast<double>(transformed.z);
                    if (tx * tx + ty * ty + tz * tz
                        < merge_min_point_range * merge_min_point_range) {
                        continue;
                    }
                }
                merged.push_back(transformed);
            }
        }
        std::sort(merged.begin(), merged.end(), [](const Point &lhs, const Point &rhs) {
            return lhs.timestamp < rhs.timestamp;
        });

        sensor_msgs::msg::PointCloud2 msg;
        // 合并帧跨度按"两台各一帧"放宽到 4 倍（单帧仍是 2 倍，行为不变）
        if (!build_pointcloud_message(merged, merge_frame_id, robustness_config_, msg, "merged", 4.0)) {
            for (auto &entry: snapshot) {
                entry.second->discard_pending_pointcloud();
            }
            return false;
        }
        if (!merge_publisher) {
            return false;
        }
        merge_publisher->publish(msg);
        ++merge_published_frames;
        last_merge_dt_ms = worst_dt_ms;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_merge_log > std::chrono::seconds(5)) {
            last_merge_log = now;
            RCLCPP_INFO(get_logger(),
                        "内部融合统计：发布 %zu 帧，丢弃 %zu 帧，最近帧头差 %.2f ms（窗口 %.1f ms）",
                        merge_published_frames, merge_dropped_frames, last_merge_dt_ms,
                        merge_max_interval_ms);
        }
        return true;
    }

    Mid360DriverNode::Mid360DriverNode(const rclcpp::NodeOptions &options) : Node("mid360_driver", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string imu_frame = declare_parameter<std::string>("imu_frame");
        std::string host_ip = declare_parameter<std::string>("host_ip");
        double lidar_publish_time_interval = declare_parameter<double>("lidar_publish_time_interval");
        bool is_topic_name_with_lidar_ip = declare_parameter<bool>("is_topic_name_with_lidar_ip");
        DriverRobustnessConfig robustness_config;
        robustness_config.validate_crc = declare_parameter<bool>("validate_crc");
        robustness_config.max_packet_time_jump = declare_parameter<double>("max_packet_time_jump");
        robustness_config.max_packet_time_span = declare_parameter<double>("max_packet_time_span");
        robustness_config.max_point_range = declare_parameter<double>("max_point_range");
        robustness_config.max_imu_acc = declare_parameter<double>("max_imu_acc");
        robustness_config.max_imu_gyro = declare_parameter<double>("max_imu_gyro");
        robustness_config.min_drop_log_interval = declare_parameter<double>("min_drop_log_interval");
        robustness_config_ = robustness_config;

        // ---- 驱动内部融合参数（默认关闭；关闭时代码路径与原来完全一致） ----
        enable_internal_lidar_merge = declare_parameter<bool>("enable_internal_lidar_merge", false);
        merge_primary_ip = declare_parameter<std::string>("merge_primary_ip", "");
        merge_output_topic = declare_parameter<std::string>("merge_output_topic", "/livox/lidar");
        merge_frame_id = declare_parameter<std::string>("merge_frame_id", "livox_frame");
        merge_max_interval_ms = declare_parameter<double>("merge_max_interval_ms", 5.0);
        merge_publish_individual = declare_parameter<bool>("merge_publish_individual", true);
        merge_min_point_range = declare_parameter<double>("merge_min_point_range", 0.0);
        {
            const auto extrinsic = declare_parameter<std::vector<double>>(
                    "merge_extrinsic_secondary_to_primary",
                    std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            if (extrinsic.size() == 6U) {
                std::copy(extrinsic.begin(), extrinsic.end(), merge_extrinsic);
            } else {
                RCLCPP_ERROR(get_logger(),
                             "merge_extrinsic_secondary_to_primary 需要 6 个数（x,y,z,roll,pitch,yaw），已忽略");
                enable_internal_lidar_merge = false;
            }
        }
        if (enable_internal_lidar_merge) {
            if (!is_topic_name_with_lidar_ip) {
                RCLCPP_ERROR(get_logger(),
                             "enable_internal_lidar_merge=true 需要 is_topic_name_with_lidar_ip=true"
                             "（否则无法按 IP 区分设备），已忽略该开关");
                enable_internal_lidar_merge = false;
            } else if (merge_publish_individual) {
                // 让下游还能拿到单路话题（离线标定、诊断都要用）
            }
            if (enable_internal_lidar_merge) {
                merge_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
                        merge_output_topic, rclcpp::SensorDataQoS());
                RCLCPP_INFO(get_logger(),
                            "驱动内部双雷达融合已启用：主雷达 %s → 输出 %s（window %.1f ms，外参 "
                            "[%.4f %.4f %.4f | %.2f° %.2f° %.2f°]）",
                            merge_primary_ip.empty() ? "IP 最小者" : merge_primary_ip.c_str(),
                            merge_output_topic.c_str(),
                            merge_max_interval_ms,
                            merge_extrinsic[0], merge_extrinsic[1], merge_extrinsic[2],
                            merge_extrinsic[3] * 180.0 / M_PI, merge_extrinsic[4] * 180.0 / M_PI,
                            merge_extrinsic[5] * 180.0 / M_PI);
                RCLCPP_INFO(get_logger(),
                            "主雷达的 IMU 将发到 %s（不带 IP 后缀），其余设备仍带后缀",
                            imu_topic.c_str());
            }
        }

        if (!is_topic_name_with_lidar_ip) {
            lidar_publisher.configure_robustness(robustness_config);
            lidar_publisher.ensure_initialized(*this, lidar_topic, imu_topic);
        }
        // 端口被占用（另一个驱动实例 / Livox Viewer2 / 上一次没退干净）时，
        // 构造函数里的 asio bind 会抛 std::system_error。原来它会直接 terminate，
        // 配合 launch 的 respawn=True 就是无限重启刷屏。这里给出可操作的提示。
        try {
        mid360_driver = std::make_unique<mid360_driver::Mid360Driver>(
                io_context,
                asio::ip::make_address(host_ip),
                robustness_config,
                [this, is_topic_name_with_lidar_ip, robustness_config](const asio::ip::address &lidar_ip, const std::vector<Point> &points) {
                    std::lock_guard lock(multi_lidar_mutex_);
                    if (is_topic_name_with_lidar_ip) {
                        auto iter = multi_lidar_publishers.try_emplace(lidar_ip).first;
                        iter->second.configure_robustness(robustness_config);
                        iter->second.on_receive_pointcloud(points);
                    } else {
                        lidar_publisher.on_receive_pointcloud(points);
                    }
                },
                [this, is_topic_name_with_lidar_ip, robustness_config](const asio::ip::address &lidar_ip, const ImuMsg &imu_msg) {
                    std::lock_guard lock(multi_lidar_mutex_);
                    if (is_topic_name_with_lidar_ip) {
                        auto iter = multi_lidar_publishers.try_emplace(lidar_ip).first;
                        iter->second.configure_robustness(robustness_config);
                        iter->second.on_receive_imu(imu_msg);
                    } else {
                        lidar_publisher.on_receive_imu(imu_msg);
                    }
                });
        } catch (const std::system_error &error) {
            // 两种 bind 失败要分开说，排查方向完全不同：
            //   EADDRINUSE   = 端口被别人占了（另一个驱动 / Viewer2）
            //   EADDRNOTAVAIL= 网卡上没有 host_ip 这个地址（没插线 / 没配 IP / 网卡 down）
            const int code = error.code().value();
            if (code == EADDRINUSE) {
                RCLCPP_FATAL(get_logger(),
                             "起不了：UDP 端口 %s 已被占用。多半是另一个 mid360_driver 还在跑，"
                             "或者 Livox Viewer 2 开着。先 `ss -ulpn | grep 56301` 看是谁，"
                             "再 `pkill -9 -x mid360_driver_node` / 关掉 Viewer2 后重试。",
                             host_ip.c_str());
            } else if (code == EADDRNOTAVAIL) {
                RCLCPP_FATAL(get_logger(),
                             "起不了：网卡上没有 %s 这个地址（bind: %s）。检查：网线插了吗？"
                             "`ip -br addr show <网口>` 里有没有这个 IP？载波是不是 UP？"
                             "驱动会绑 host_ip，网卡 DOWN 时必然失败。",
                             host_ip.c_str(), error.what());
            } else {
                RCLCPP_FATAL(get_logger(), "起不了：bind 失败（%s，errno=%d）",
                             error.what(), code);
            }
            throw;
        }
        if (is_topic_name_with_lidar_ip) {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this, lidar_topic, imu_topic, lidar_frame]() {
                std::vector<std::pair<asio::ip::address, LidarPublisher*>> snapshot;
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    snapshot.reserve(multi_lidar_publishers.size());
                    for (auto &[lidar_ip, publisher] : multi_lidar_publishers) {
                        publisher.prepare_pointcloud_to_publish();
                        snapshot.emplace_back(lidar_ip, &publisher);
                    }
                }
                // 只为"确实要发单路"的设备创建发布器：融合开启且
                // merge_publish_individual=false 时，单路话题根本不该出现。
                if (!enable_internal_lidar_merge || merge_publish_individual) {
                    for (auto &[lidar_ip, publisher] : snapshot) {
                        publisher->ensure_initialized(*this, lidar_topic, imu_topic, lidar_ip);
                    }
                }
                // 融合开启时先试合并帧；单帧是否照发由 merge_publish_individual 决定
                const bool merged = try_publish_merged_pointcloud(snapshot);
                // 融合开启且明确不要单路话题时：**这一拍不管合并成没成功都到此为止**。
                // 之前写成 `if (merged && !merge_publish_individual) return;`，于是合并
                // 失败的那一拍（启动瞬间某台雷达还没数据）会掉进下面的单帧发布，而那时
                // 单路发布器根本没创建 → 空指针 → SIGSEGV（respawn 一次崩一次）。
                if (!merge_publish_individual && enable_internal_lidar_merge) {
                    (void) merged;
                    return;
                }
                for (auto &[lidar_ip, publisher] : snapshot) {
                    publisher->publish_pointcloud(lidar_frame);
                }
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, lidar_topic, imu_topic, imu_frame]() {
                std::vector<std::pair<asio::ip::address, LidarPublisher*>> snapshot;
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    snapshot.reserve(multi_lidar_publishers.size());
                    for (auto &[lidar_ip, publisher] : multi_lidar_publishers) {
                        publisher.prepare_imu_to_publish();
                        snapshot.emplace_back(lidar_ip, &publisher);
                    }
                }
                for (auto &[lidar_ip, publisher] : snapshot) {
                    // 融合模式下主雷达的 IMU 走**不带 IP 后缀**的话题：下游
                    // small_glim / fake_vel_transform 订的都是配置里的 imu_topic
                    // （/livox/imu），带了后缀它们一帧都收不到。
                    const bool suffixed = !is_primary_device(lidar_ip);
                    publisher->ensure_imu_initialized(*this, imu_topic, lidar_ip, suffixed);
                    publisher->publish_imu(imu_frame);
                }
            });
        } else {
            publish_pointcloud_timer = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double, std::ratio<1, 1>>(lidar_publish_time_interval), [this, lidar_frame]() {
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    lidar_publisher.prepare_pointcloud_to_publish();
                }
                lidar_publisher.publish_pointcloud(lidar_frame);
            });
            publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1), [this, imu_frame]() {
                {
                    std::lock_guard lock(multi_lidar_mutex_);
                    lidar_publisher.prepare_imu_to_publish();
                }
                lidar_publisher.publish_imu(imu_frame);
            });
        }
        io_thread = std::thread([this]() {
            io_context.run();
        });
    }

    Mid360DriverNode::~Mid360DriverNode() {
        if (mid360_driver) {
            mid360_driver->stop();
        }
        io_context.stop();
        if (io_thread.joinable()) {
            io_thread.join();
        }
    }

}// namespace mid360_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mid360_driver::Mid360DriverNode)
