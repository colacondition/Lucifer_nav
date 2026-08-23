#pragma once

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>
#include "fast_location/frame_transforms.hpp"
#include "fast_location/global_search.hpp"
#include "fast_location/alignment_quality.hpp"
using Point = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<Point>;




class RobotLocalizationNode : public rclcpp::Node
{
public:
    RobotLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RobotLocalizationNode();
private:
    // 此前是 .cpp 文件级全局：单实例侥幸安全，但生命周期脱离节点、多实例必串状态。
    // 收编为成员（默认值与原全局初始化一致）。
    std::mutex data_mutex_;
    std::mutex tf_mutex_;
    PointCloudXYZI::Ptr global_map_{std::make_shared<PointCloudXYZI>()};
    PointCloudXYZI::Ptr cur_scan_{std::make_shared<PointCloudXYZI>()};
    nav_msgs::msg::Odometry::SharedPtr cur_odom_{std::make_shared<nav_msgs::msg::Odometry>()};
    Eigen::Matrix4f T_pcd_to_odom_{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f initial_pcd_to_odom_{Eigen::Matrix4f::Identity()};
    // 默认使用代码内置零位姿作为初始位姿
    std::atomic<bool> initial_pose_received_{true};
    // 这三个标志在订阅回调（SubScan/subInitPose）与定位定时器回调
    // （locationThread/globalLocalization）之间跨线程共享，必须用 atomic 消除
    // 数据竞争（旧实现是普通 bool，MultiThreadedExecutor(2) 下属 UB）。
    std::atomic<bool> initialized_{false};  // 系统是否已初始化

    Eigen::Matrix4f poseToMat(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    Eigen::Matrix4f inverseSE3(const Eigen::Matrix4f &T);
    Eigen::Matrix4f runICP(PointCloudXYZI::Ptr src, PointCloudXYZI::Ptr tgt, const Eigen::Matrix4f &initial_guess, float voxel_scale, int max_iterations, float &fitness_score);
    bool globalLocalization(const Eigen::Matrix4f &pose_guess);
    bool performGlobalSearch();
    // 检查暂存的全局搜索结果是否在新一帧扫描下依然成立。
    bool verifyPendingGlobalResult();
    bool snapshotLocalizationInput(
        PointCloudXYZI::Ptr &scan_for_icp,
        nav_msgs::msg::Odometry &odom_snapshot);
    // 把 scan_buffer_ 里最近 frames 帧合并出来,专门给精化阶段用。
    PointCloudXYZI::Ptr snapshotAccumulatedScan(int frames);
    bool acceptLocalizationResult(
        const Eigen::Matrix4f &result,
        float fitness,
        const Eigen::Matrix4f &pose_guess,
        const PointCloudXYZI::Ptr &scan_for_icp,
        const std::chrono::high_resolution_clock::time_point &start_time,
        bool global_search_result);
    void handleTrackingFailure();
    void enterLost(const char * reason);
    void abandonPendingGlobalResult();
    void clearDownsampleCaches();
    PointCloudXYZI::Ptr cachedOrDownsample(
        std::unordered_map<int, PointCloudXYZI::Ptr> & cache,
        int scale_key,
        const PointCloudXYZI::Ptr & cloud,
        float voxel_size);
    void publishIntegrityState(fast_location::IntegrityState state, const char * reason);
    void handleInconsistentObservation(double nis);
    void handleFrontendDiverged();
    void handleDirtyObservation();
    bool loadGlobalMap(const std::string &pcd_file_path);
    PointCloudXYZI::Ptr voxelDownSample(PointCloudXYZI::Ptr cloud, float voxel_size);
    PointCloudXYZI::Ptr gropGlobalMapInFOV(
        const Eigen::Matrix4f &pose_set,
        const nav_msgs::msg::Odometry &odom,
        float search_radius = -1.0f,
        bool publish_debug_cloud = true);

    void SubScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void SubOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void subInitPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void subInitPoseWithCovariance(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void publishGlobalMap();  // 定时器回调函数
    void locationThread();
    void publishMapToOdomTf();
    bool hasPointCloudSubscribers(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const;
    void publishPointCloudIfSubscribed(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
        const PointCloudXYZI & cloud,
        const std::string & frame_id,
        const builtin_interfaces::msg::Time & stamp);
    PointCloudXYZI::Ptr transformCloud(
        const PointCloudXYZI & cloud,
        const Eigen::Matrix4f & transform) const;
    void publishPcdCloudInMap(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
        const PointCloudXYZI & cloud,
        const builtin_interfaces::msg::Time & stamp);


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap;  
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odometry;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_downsampled;   // 下采样后的扫描点云
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_downsampled;    // 下采样后的地图点云
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF广播器

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scam;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_init_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_covariance;
    
    rclcpp::TimerBase::SharedPtr map_publish_timer;  // 定时器
    rclcpp::TimerBase::SharedPtr localization_timer;
    rclcpp::TimerBase::SharedPtr tf_publish_timer;
    // 定位与 TF 各占 executor 一条线程；订阅留在默认组，
    // 全场搜索占住定位周期时 SubScan 仍能进缓冲（时序校验要新帧）。
    rclcpp::CallbackGroup::SharedPtr loc_callback_group_;
    rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
    pcl::KdTreeFLANN<Point>::Ptr kdtree_global_map;

    // 配置参数
    int io_queue_size_ = 10;
    double map_publish_rate_hz_ = 5.0;
    double localization_rate_hz_ = 10.0;
    double tf_publish_rate_hz_ = 100.0;
    float map_voxel_size_;      // 地图点云体素滤波尺寸（米）
    float scan_voxel_size_;     // 扫描点云体素滤波尺寸（米）
    float submap_voxel_size_first_;   // 首次定位子图体素尺寸（米）
    float submap_voxel_size_track_;   // 后续跟踪子图体素尺寸（米）
    float fov_far_;             // 视场内的最远距离（米）
    float localization_th_;     // ICP 匹配成功的阈值（内点比例）
    float first_localization_th_; // 首次匹配阈值（内点比例）
    // 条件数阈值：平移信息矩阵 XY 子块的 λ_max/λ_min 超过此值视为方向退化
    // （典型是长走廊）。命中后不整拍拒绝，而是把修正投影到可观测方向。
    double degenerate_condition_threshold_{100.0};
    // 硬拒绝阈值：条件数超过此值说明连旋转都不可信，整拍放弃本次结果。
    double degenerate_hard_reject_condition_{5000.0};
    // EMA 平滑系数 α：接受新结果后 T_smooth = α*T_new + (1-α)*T_prev。
    // 越小越平滑、越滞后；默认 0.7 响应较快。
    double ema_alpha_{0.7};
    // 出洞/几何恢复拍的临时 EMA 系数：接近 1（一拍吸收绝大部分修正，避免数秒
    // 爬行），又保留少量平滑防止单拍噪声完全外露成抖动。
    double recovery_alpha_{0.95};
    bool ema_initialized_{false};
    Eigen::Matrix4f ema_transform_{Eigen::Matrix4f::Identity()};
    // 出洞检测：上一拍是否退化。退化→正常的转变（几何恢复，如出隧道）意味着
    // 本拍 ICP 修正是找回退化期间累积的漂移，应用高 α 快速收敛而非 0.7 爬行。
    bool last_degenerate_{false};
    bool degeneracy_recovery_{false};
    // 粗→细校正拍：本拍跳出了单尺度局部最优，修正应彻底吸收（否则每拍剩
    // 30% 会累积成漂移）。与出洞恢复共用 recovery_alpha_ 快速收敛。
    bool coarse_escape_{false};
    // 连续退化拍数（迟滞）：单拍条件数在阈值附近抖动会频繁误触发出洞。
    // 连续 degenerate_enter_streak_ 拍退化才认定"在洞里"，之后出洞才触发恢复。
    int degenerate_streak_{0};
    int degenerate_enter_streak_{3};
    // 粗→细逃逸的触发周期：每 coarse_every_ 拍穿插一次粗尺度 ICP（打破自锁
    // 累积漂移），其余拍用单尺度 1.0 保持逐拍稳定。粗尺度每拍都跑会把随机
    // 逃逸变成逐拍抖动（体素 0.4m、对应距离 3m 太松，重复几何里每拍收敛到
    // 不同局部最优），降频后只在需要时拉回漂移，抖动被 EMA 平滑。
    int coarse_every_{5};
    int frame_counter_{0};
    int gicp_max_iterations_first_ = 50;
    int gicp_max_iterations_track_ = 20;
    bool publish_tf_ = true; // 是否发布 map->odom TF
    bool publish_map_to_odometry_ = false; // 是否发布 map_to_odometry 里程计话题
    bool use_cuda_ = false;      // 是否启用 GPU 加速
    int gicp_num_threads_ = 2;   // 8-core budget; 0 in YAML still falls back to 2
    bool enable_global_search_ = true;
    float global_search_refine_radius_ = 12.0f;
    // 精化阶段(ICP)使用的堆积帧数;粗搜仍用主 scan。<=1 表示不额外堆积。
    int global_search_refine_accumulate_ = 3;
    // 全局搜索通过后,是否再用下一帧扫描做一次一致性校验。
    bool enable_temporal_verification_ = true;
    // 一致性校验时,新扫描在最佳位姿附近的最低得分。
    float temporal_verification_min_score_ = 0.30f;
    fast_location::GlobalSearchConfig global_search_config_;
    fast_location::TrackingRecovery tracking_recovery_{5};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_localization_status_;
    fast_location::AlignmentAssessment last_assessment_{};
    Eigen::Matrix<double, 6, 6> last_hessian_{Eigen::Matrix<double, 6, 6>::Zero()};
    bool enable_nis_{true};
    double nis_reject_threshold_{12.0};
    int nis_lost_streak_{8};
    double nis_prior_xy_std_{0.20};
    double nis_prior_yaw_std_{0.139626};
    int frontend_diverged_streak_limit_{5};
    int nis_conflict_streak_{0};
    int frontend_diverged_streak_{0};

    // 一致性校验暂存的待接受结果。仅在 enable_temporal_verification_ 为真时使用。
    bool pending_global_result_valid_{false};
    Eigen::Matrix4f pending_global_result_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pending_global_guess_ = Eigen::Matrix4f::Identity();
    float pending_global_fitness_ = 0.0f;
    // loc 回调组独占；每个 locationThread 拍开头从 reloc_generation_ 拷一份。
    uint64_t loc_tick_epoch_{0};
    // 与 pending/T_pcd_to_odom_ 同属 tf_mutex_。subInitPose / enterLost 自增后，
    // 在飞的搜索不得把旧结果写进 map→odom。
    uint64_t reloc_generation_{0};
    // 操作员给了近处 /initialpose：下一拍走该先验的多尺度 ICP，不跑全场网格。
    // 相对上一拍跳变过大则不置位，直接 LOST 召回。
    bool use_pose_prior_{false};
    // acceptLocalizationResult 成功过一次。enterLost 不清：召回仍全场搜，
    // 但用上一拍 T_pcd_to_odom_ 限制 map→odom 跳变；开机没有这个位姿，走出生点局部 ICP。
    bool had_accepted_pose_{false};
    float global_search_max_map_odom_jump_{4.0f};
    std::string scan_input_frame_mode_ = "odom"; // 源点云所在坐标系: odom/base
    int scan_accumulate_frames_ = 1;   // 源点云堆积帧数
    float scan_min_range_ = 0.0f;   // 源点云最小距离（米）
    float scan_max_range_ = 0.0f;   // 源点云最大距离（米，<=0表示不限制）
    std::string map_frame_;     // 地图坐标系
    std::string base_frame_;    // 机器人/雷达坐标系
    std::string pc_in_map_frame_;  // pc_in_map 对外统一使用 map_frame
    double map_z_offset_{0.0};  // Z 轴偏移补偿（仿真地面厚度，实车为 0）
    Eigen::Matrix4f map_from_pcd_{Eigen::Matrix4f::Identity()};
    std::atomic<bool> first_localization_{true};  // 是否为首次定位（首次用多尺度）
    bool tf_ready_ = false;
    std::atomic<bool> has_new_scan_{false};
    std::atomic<bool> odom_received_{false};
    std::deque<PointCloudXYZI::Ptr> scan_buffer_;

    // 性能优化相关
    std::unordered_map<int, PointCloudXYZI::Ptr> scan_downsample_cache_;  // 扫描点云下采样缓存
    std::unordered_map<int, PointCloudXYZI::Ptr> map_downsample_cache_;   // 地图点云下采样缓存
    sensor_msgs::msg::PointCloud2 global_map_msg_;
    bool global_map_msg_ready_ = false;
};
