#include "fast_location/robot_localization.hpp"
#include "fast_location/alignment_quality.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <filesystem>
#include <stdexcept>


std::mutex data_mutex;
std::mutex tf_mutex;

PointCloudXYZI::Ptr global_map(new PointCloudXYZI);
PointCloudXYZI::Ptr cur_scan(new PointCloudXYZI);
nav_msgs::msg::Odometry::SharedPtr cur_odom(new nav_msgs::msg::Odometry);
Eigen::Matrix4f T_pcd_to_odom = Eigen::Matrix4f::Identity();
Eigen::Matrix4f initial_pcd_to_odom = Eigen::Matrix4f::Identity();


std::atomic<bool> initial_pose_received{true};    // 默认使用代码内置零位姿作为初始位姿
bool initialized = false;              // 系统是否已初始化

RobotLocalizationNode::RobotLocalizationNode(const rclcpp::NodeOptions & options): rclcpp::Node("robot_localization_node", options)
{
    // 初始化性能优化相关成员
    accumulated_scan_.reset(new PointCloudXYZI);

    this->declare_parameter<std::string>("map_pcd_path", "package://bringup/PCD/RMUL.pcd");
    this->declare_parameter<float>("map_voxel_size", 0.1);
    this->declare_parameter<float>("scan_voxel_size", 0.1);
    this->declare_parameter<float>("submap_voxel_size_first", 0.1);
    this->declare_parameter<float>("submap_voxel_size_track", 0.2);
    this->declare_parameter<float>("fov_far", 20.0);
    this->declare_parameter<float>("refine_fov_far", 15.0);
    this->declare_parameter<float>("localization_th", 0.85);
    this->declare_parameter<float>("first_localization_th", 0.95);
    this->declare_parameter<bool>("use_fast_gicp", true);
    this->declare_parameter<bool>("use_cuda", false);
    this->declare_parameter<int>("gicp_num_threads", 2);
    this->declare_parameter<int>("gicp_max_iterations_first", 50);
    this->declare_parameter<int>("gicp_max_iterations_track", 20);
    this->declare_parameter<bool>("enable_global_search", true);
    this->declare_parameter<float>("global_search_xy_step", 2.0);
    this->declare_parameter<float>("global_search_yaw_step_deg", 30.0);
    this->declare_parameter<float>("global_search_score_distance", 0.45);
    this->declare_parameter<int>("global_search_score_stride", 4);
    this->declare_parameter<int>("global_search_top_k", 6);
    this->declare_parameter<float>("global_search_min_score", 0.20);
    this->declare_parameter<float>("global_search_min_score_margin", 0.03);
    this->declare_parameter<float>("global_search_score_tie_epsilon", 0.02);
    this->declare_parameter<float>("global_search_candidate_separation", 1.0);
    this->declare_parameter<float>("global_search_candidate_yaw_separation_deg", 20.0);
    this->declare_parameter<float>("global_search_refine_radius", 12.0);
    this->declare_parameter<float>("global_search_refine_score_distance", 0.20);
    this->declare_parameter<int>("global_search_refine_score_stride", 1);
    this->declare_parameter<int>("global_search_refine_accumulate_frames", 3);
    this->declare_parameter<bool>("global_search_enable_temporal_verification", false);
    this->declare_parameter<float>("global_search_temporal_min_score", 0.30);
    this->declare_parameter<int>("tracking_failures_before_global_search", 5);
    this->declare_parameter<std::string>("scan_input_frame_mode", "odom");
    this->declare_parameter<int>("scan_accumulate_frames", 1);
    this->declare_parameter<float>("scan_min_range", 0.0);
    this->declare_parameter<float>("scan_max_range", 0.0);

    this->declare_parameter<std::string>("sub_scan_topic", "/lio/cloud_world");
    this->declare_parameter<std::string>("sub_odom_topic", "/lio/robo/odom");
    this->declare_parameter<std::string>("sub_init_pose_topic", "initialpose_3d");
    this->declare_parameter<std::string>("sub_init_pose_covariance_topic", "/initialpose");

    this->declare_parameter<std::string>("pub_pc_in_map_topic", "pc_in_map");
    this->declare_parameter<std::string>("pub_global_map_topic", "global_map");
    this->declare_parameter<std::string>("pub_submap_topic", "submap");
    this->declare_parameter<std::string>("pub_map_to_odometry_topic", "map_to_odometry");
    this->declare_parameter<std::string>("pub_scan_downsampled_topic", "scan_downsampled");
    this->declare_parameter<std::string>("pub_map_downsampled_topic", "map_downsampled");

    this->declare_parameter<int>("io_queue_size", 10);
    this->declare_parameter<double>("map_publish_rate_hz", 5.0);
    this->declare_parameter<double>("localization_rate_hz", 10.0);
    this->declare_parameter<double>("tf_publish_rate_hz", 100.0);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("publish_map_to_odometry", false);

    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "odom");
    this->declare_parameter<std::string>("pc_in_map_frame", "map");
    // 可选：手动指定 PCD 到 map 的平面对齐 [x, y, yaw]。
    this->declare_parameter<std::vector<double>>("pcd_to_map_pose", std::vector<double>{});
    // Z 轴偏移补偿（仿真环境地面厚度补偿，实车为 0.0）
    this->declare_parameter<double>("map_z_offset", 0.0);

    std::string map_pcd_path = this->get_parameter("map_pcd_path").as_string();
    map_voxel_size_ = this->get_parameter("map_voxel_size").as_double();
    scan_voxel_size_ = this->get_parameter("scan_voxel_size").as_double();
    submap_voxel_size_first_ = this->get_parameter("submap_voxel_size_first").as_double();
    submap_voxel_size_track_ = this->get_parameter("submap_voxel_size_track").as_double();
    fov_far_ = this->get_parameter("fov_far").as_double();
    refine_fov_far_ = this->get_parameter("refine_fov_far").as_double();
    localization_th_ = this->get_parameter("localization_th").as_double();
    first_localization_th_ = this->get_parameter("first_localization_th").as_double();
    this->declare_parameter<double>("degenerate_condition_threshold", 100.0);
    degenerate_condition_threshold_ = this->get_parameter("degenerate_condition_threshold").as_double();
    this->declare_parameter<double>("degenerate_hard_reject_condition", 5000.0);
    degenerate_hard_reject_condition_ =
        this->get_parameter("degenerate_hard_reject_condition").as_double();
    // 硬拒绝阈值必须不低于软阈值，否则软路径永远不会生效。
    if (degenerate_hard_reject_condition_ < degenerate_condition_threshold_) {
        RCLCPP_WARN(
            this->get_logger(),
            "degenerate_hard_reject_condition (%.1f) < degenerate_condition_threshold (%.1f); "
            "clamping to the latter so anisotropic updates stay reachable.",
            degenerate_hard_reject_condition_, degenerate_condition_threshold_);
        degenerate_hard_reject_condition_ = degenerate_condition_threshold_;
    }
    this->declare_parameter<double>("ema_alpha", 0.7);
    ema_alpha_ = this->get_parameter("ema_alpha").as_double();
    this->declare_parameter<double>("recovery_alpha", 0.95);
    recovery_alpha_ = this->get_parameter("recovery_alpha").as_double();
    this->declare_parameter<int>("degenerate_enter_streak", 3);
    degenerate_enter_streak_ = this->get_parameter("degenerate_enter_streak").as_int();
    this->declare_parameter<int>("coarse_every", 3);
    coarse_every_ = static_cast<int>(
        std::max<int64_t>(1, this->get_parameter("coarse_every").as_int()));
    use_fast_gicp_ = this->get_parameter("use_fast_gicp").as_bool();
    use_cuda_ = this->get_parameter("use_cuda").as_bool();
    gicp_num_threads_ = this->get_parameter("gicp_num_threads").as_int();
    gicp_max_iterations_first_ = this->get_parameter("gicp_max_iterations_first").as_int();
    gicp_max_iterations_track_ = this->get_parameter("gicp_max_iterations_track").as_int();
    enable_global_search_ = this->get_parameter("enable_global_search").as_bool();
    global_search_config_.xy_step = std::max(
        0.1, this->get_parameter("global_search_xy_step").as_double());
    const double global_search_yaw_step_deg = std::max(
        1.0, this->get_parameter("global_search_yaw_step_deg").as_double());
    global_search_config_.yaw_step = static_cast<float>(global_search_yaw_step_deg * M_PI / 180.0);
    global_search_config_.score_distance = std::max(
        0.05, this->get_parameter("global_search_score_distance").as_double());
    global_search_config_.score_stride = static_cast<std::size_t>(std::max<int64_t>(
        1, this->get_parameter("global_search_score_stride").as_int()));
    global_search_config_.top_k = static_cast<std::size_t>(std::max<int64_t>(
        1, this->get_parameter("global_search_top_k").as_int()));
    global_search_config_.minimum_score = std::clamp(
        static_cast<float>(this->get_parameter("global_search_min_score").as_double()),
        0.0f, 1.0f);
    global_search_config_.minimum_score_margin = std::clamp(
        static_cast<float>(this->get_parameter("global_search_min_score_margin").as_double()),
        0.0f, 1.0f);
    global_search_config_.score_tie_epsilon = std::clamp(
        static_cast<float>(this->get_parameter("global_search_score_tie_epsilon").as_double()),
        0.0f, 1.0f);
    global_search_config_.minimum_candidate_separation = std::max(
        0.0, this->get_parameter("global_search_candidate_separation").as_double());
    const double candidate_yaw_separation_deg = std::max(
        0.0, this->get_parameter("global_search_candidate_yaw_separation_deg").as_double());
    global_search_config_.minimum_candidate_yaw_separation =
        static_cast<float>(candidate_yaw_separation_deg * M_PI / 180.0);
    global_search_refine_radius_ = std::max(
        1.0, this->get_parameter("global_search_refine_radius").as_double());
    global_search_config_.refine_score_distance = std::max(
        0.0, this->get_parameter("global_search_refine_score_distance").as_double());
    global_search_config_.refine_score_stride = static_cast<std::size_t>(std::max<int64_t>(
        1, this->get_parameter("global_search_refine_score_stride").as_int()));
    global_search_refine_accumulate_ = std::max<int>(
        1, this->get_parameter("global_search_refine_accumulate_frames").as_int());
    enable_temporal_verification_ = this->get_parameter(
        "global_search_enable_temporal_verification").as_bool();
    temporal_verification_min_score_ = std::clamp(
        static_cast<float>(this->get_parameter("global_search_temporal_min_score").as_double()),
        0.0f, 1.0f);
    tracking_recovery_ = fast_location::TrackingRecovery(static_cast<std::size_t>(std::max<int64_t>(
        1, this->get_parameter("tracking_failures_before_global_search").as_int())));
    scan_input_frame_mode_ = this->get_parameter("scan_input_frame_mode").as_string();
    scan_accumulate_frames_ = this->get_parameter("scan_accumulate_frames").as_int();
    scan_min_range_ = this->get_parameter("scan_min_range").as_double();
    scan_max_range_ = this->get_parameter("scan_max_range").as_double();
    if (scan_input_frame_mode_ != "odom" && scan_input_frame_mode_ != "base") {
        RCLCPP_WARN(this->get_logger(), "Invalid scan_input_frame_mode='%s', fallback to 'odom'", scan_input_frame_mode_.c_str());
        scan_input_frame_mode_ = "odom";
    }
    if (scan_min_range_ < 0.0f) {
        scan_min_range_ = 0.0f;
    }
    if (scan_accumulate_frames_ <= 0) {
        scan_accumulate_frames_ = 1;
    }
    if (scan_max_range_ > 0.0f && scan_max_range_ <= scan_min_range_) {
        RCLCPP_WARN(this->get_logger(), "scan_max_range must be larger than scan_min_range, disable max-range filter.");
        scan_max_range_ = 0.0f;
    }

    std::string sub_scan_topic = this->get_parameter("sub_scan_topic").as_string();
    std::string sub_odom_topic = this->get_parameter("sub_odom_topic").as_string();
    std::string sub_init_pose_topic = this->get_parameter("sub_init_pose_topic").as_string();
    std::string sub_init_pose_covariance_topic = this->get_parameter("sub_init_pose_covariance_topic").as_string();

    std::string pub_pc_in_map_topic = this->get_parameter("pub_pc_in_map_topic").as_string();
    std::string pub_global_map_topic = this->get_parameter("pub_global_map_topic").as_string();
    std::string pub_submap_topic = this->get_parameter("pub_submap_topic").as_string();
    std::string pub_map_to_odometry_topic = this->get_parameter("pub_map_to_odometry_topic").as_string();
    std::string pub_scan_downsampled_topic = this->get_parameter("pub_scan_downsampled_topic").as_string();
    std::string pub_map_downsampled_topic = this->get_parameter("pub_map_downsampled_topic").as_string();

    io_queue_size_ = this->get_parameter("io_queue_size").as_int();
    if (io_queue_size_ <= 0) {
        io_queue_size_ = 10;
    }
    map_publish_rate_hz_ = this->get_parameter("map_publish_rate_hz").as_double();
    if (map_publish_rate_hz_ <= 0.0) {
        map_publish_rate_hz_ = 1.0;
    }
    localization_rate_hz_ = this->get_parameter("localization_rate_hz").as_double();
    if (localization_rate_hz_ <= 0.0) {
        localization_rate_hz_ = 1.0;
    }
    tf_publish_rate_hz_ = this->get_parameter("tf_publish_rate_hz").as_double();
    if (tf_publish_rate_hz_ <= 0.0) {
        tf_publish_rate_hz_ = 10.0;
    }
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    publish_map_to_odometry_ = this->get_parameter("publish_map_to_odometry").as_bool();

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    pc_in_map_frame_ = this->get_parameter("pc_in_map_frame").as_string();
    map_z_offset_ = this->get_parameter("map_z_offset").as_double();
    if (!pc_in_map_frame_.empty() && pc_in_map_frame_ != map_frame_) {
        RCLCPP_WARN(
            this->get_logger(),
            "pc_in_map_frame='%s' is incompatible with map-aligned output; using '%s'.",
            pc_in_map_frame_.c_str(), map_frame_.c_str());
    }
    pc_in_map_frame_ = map_frame_;

    if (first_localization_th_ <= 0.0f || first_localization_th_ > 1.0f) {
        first_localization_th_ = localization_th_;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "Parameters loaded: map_voxel=%.2f scan_voxel=%.2f fov_far=%.2f loc_th=%.2f first_th=%.2f "
        "threads=%d scan_mode=%s accumulate=%d map_rate=%.2f loc_rate=%.2f publish_tf=%s publish_aux_odom=%s",
        map_voxel_size_, scan_voxel_size_, fov_far_, localization_th_, first_localization_th_,
        gicp_num_threads_, scan_input_frame_mode_.c_str(), scan_accumulate_frames_,
        map_publish_rate_hz_, localization_rate_hz_, publish_tf_ ? "true" : "false",
        publish_map_to_odometry_ ? "true" : "false");

    tf_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    pub_pc_in_map = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_pc_in_map_topic, io_queue_size_);
    pub_global_map = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_global_map_topic, io_queue_size_);
    pub_submap = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_submap_topic, io_queue_size_);
    if (publish_map_to_odometry_) {
        pub_map_to_odometry = this->create_publisher<nav_msgs::msg::Odometry>(pub_map_to_odometry_topic, io_queue_size_);
    }
    pub_scan_downsampled = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_scan_downsampled_topic, io_queue_size_);
    pub_map_downsampled = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_map_downsampled_topic, io_queue_size_);
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }


    sub_scam = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_scan_topic, io_queue_size_,
        std::bind(&RobotLocalizationNode::SubScan, this, std::placeholders::_1)
    );
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odom_topic, io_queue_size_,
        std::bind(&RobotLocalizationNode::SubOdom, this, std::placeholders::_1)
    );
    if (!sub_init_pose_topic.empty()) {
        sub_init_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            sub_init_pose_topic, io_queue_size_,
            std::bind(&RobotLocalizationNode::subInitPose, this, std::placeholders::_1)
        );
    }
    if (!sub_init_pose_covariance_topic.empty()) {
        sub_init_pose_covariance = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            sub_init_pose_covariance_topic, io_queue_size_,
            std::bind(&RobotLocalizationNode::subInitPoseWithCovariance, this, std::placeholders::_1)
        );
    }

    if (!loadGlobalMap(map_pcd_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load global map from: %s", map_pcd_path.c_str());
        throw std::runtime_error("Failed to load global map");
    }

    // 保留原始全局地图，不在这里额外下采样。

    RCLCPP_DEBUG(this->get_logger(), "Building KdTree for global map...");
    kdtree_global_map.reset(new pcl::KdTreeFLANN<Point>);
    kdtree_global_map->setInputCloud(global_map);
    RCLCPP_DEBUG(this->get_logger(), "KdTree built successfully!");

    // 地图加载完成后再启动全局地图发布。
    map_publish_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / map_publish_rate_hz_),
        std::bind(&RobotLocalizationNode::publishGlobalMap, this)
    );

    const auto explicit_pose = this->get_parameter("pcd_to_map_pose").as_double_array();
    if (explicit_pose.empty()) {
        map_from_pcd_ = Eigen::Matrix4f::Identity();
        RCLCPP_INFO(
            this->get_logger(),
            " >>> [ localization ] No PCD-to-map alignment configured; using identity.");
    } else {
        if (explicit_pose.size() != 3 ||
            !std::isfinite(explicit_pose[0]) ||
            !std::isfinite(explicit_pose[1]) ||
            !std::isfinite(explicit_pose[2])) {
            throw std::runtime_error("pcd_to_map_pose must contain three finite values");
        }
        map_from_pcd_ = fast_location::planarPoseMatrix(
            {explicit_pose[0], explicit_pose[1], explicit_pose[2]});
        RCLCPP_INFO(
            this->get_logger(),
            " >>> [ localization ] Using fixed PCD-to-map alignment: [%.3f, %.3f, %.3f rad].",
            explicit_pose[0], explicit_pose[1], explicit_pose[2]);
    }
    initial_pcd_to_odom = Eigen::Matrix4f::Identity();
    T_pcd_to_odom = initial_pcd_to_odom;

    // 先发一次全局地图，保证固定对齐已经生效。
    RCLCPP_DEBUG(this->get_logger(), "Publishing global map immediately...");
    publishGlobalMap();

    localization_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / localization_rate_hz_),
        std::bind(&RobotLocalizationNode::locationThread, this)
    );
    if (publish_tf_) {
        tf_publish_timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / tf_publish_rate_hz_),
            std::bind(&RobotLocalizationNode::publishMapToOdomTf, this),
            tf_callback_group_
        );
    }

    

}

void RobotLocalizationNode::locationThread()
{
    if(!initial_pose_received.load(std::memory_order_acquire) || !odom_received_.load(std::memory_order_acquire) || global_map->empty())
    {
        return;
    }

    // 还没进入跟踪前，先做一次初始定位。
    if (!initialized) {
        // 如果之前有一次全局搜索在等第二帧校验,先处理这个。
        if (pending_global_result_valid_) {
            initialized = verifyPendingGlobalResult();
            if (initialized) {
                RCLCPP_DEBUG(this->get_logger(), "Initial localization completed.");
            }
            return;
        }
        if(!cur_scan->empty())
        {
            initialized = enable_global_search_ ?
                performGlobalSearch() : globalLocalization(initial_pcd_to_odom);
            if (initialized) {
                RCLCPP_DEBUG(this->get_logger(), "Initial localization completed.");
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                " >>> [ localization ] Waiting for first scan on the configured sub_scan_topic.");
        }
        return;
    }

    // 进入跟踪后，只在新扫描到来时重新定位。
    if(!has_new_scan_.exchange(false, std::memory_order_acq_rel))
    {
        return;
    }

    Eigen::Matrix4f pose_guess;
    {
        std::lock_guard<std::mutex> lock(tf_mutex);
        pose_guess = T_pcd_to_odom;
    }
    globalLocalization(pose_guess);
}


bool RobotLocalizationNode::snapshotLocalizationInput(
    PointCloudXYZI::Ptr &scan_for_icp,
    nav_msgs::msg::Odometry &odom_snapshot)
{
    PointCloudXYZI::Ptr scan_snapshot(new PointCloudXYZI);
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if(cur_scan->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Current scan is empty, cannot perform localization!");
            return false;
        }
        if(!odom_received_.load(std::memory_order_acquire))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Odometry not received yet, skip localization.");
            return false;
        }
        *scan_snapshot = *cur_scan;
        odom_snapshot = *cur_odom;
    }

    const auto &p = odom_snapshot.pose.pose.position;
    const auto &q = odom_snapshot.pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {
        RCLCPP_ERROR(this->get_logger(), "Odometry contains non-finite values.");
        return false;
    }

    scan_for_icp = scan_snapshot;
    if (scan_input_frame_mode_ == "base") {
        Eigen::Matrix4f odom_from_base = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
        odom_from_base.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
        odom_from_base.block<3,1>(0,3) = Eigen::Vector3f(p.x, p.y, p.z);
        PointCloudXYZI::Ptr transformed_scan(new PointCloudXYZI);
        pcl::transformPointCloud(*scan_snapshot, *transformed_scan, odom_from_base);
        scan_for_icp = transformed_scan;
    }
    return scan_for_icp && !scan_for_icp->empty();
}

PointCloudXYZI::Ptr RobotLocalizationNode::snapshotAccumulatedScan(int frames)
{
    // 从 scan_buffer_ 里取最近 frames 帧合并,提供给精化阶段。
    // <=1 或 buffer 只有一帧时不需要额外堆叠,让调用方沿用 cur_scan。
    if (frames <= 1) {
        return nullptr;
    }
    std::deque<PointCloudXYZI::Ptr> buffer_copy;
    nav_msgs::msg::Odometry odom_snapshot;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (static_cast<int>(scan_buffer_.size()) < 2) {
            return nullptr;
        }
        const int keep = std::min<int>(frames, scan_buffer_.size());
        buffer_copy.insert(
            buffer_copy.end(), scan_buffer_.end() - keep, scan_buffer_.end());
        odom_snapshot = *cur_odom;
    }

    std::size_t total_pts = 0;
    for (const auto & cloud : buffer_copy) {
        total_pts += cloud->size();
    }
    if (total_pts == 0) {
        return nullptr;
    }
    PointCloudXYZI::Ptr stacked(new PointCloudXYZI);
    stacked->reserve(total_pts);
    for (const auto & cloud : buffer_copy) {
        *stacked += *cloud;
    }

    // 与 snapshotLocalizationInput 保持一致:base 帧下需要用当前 odom 姿态搬到 odom 系。
    // 注意 buffer 里旧帧的 base 位姿其实和当前不同,这里的近似和现有单帧堆叠(cur_scan)完全一致。
    if (scan_input_frame_mode_ == "base") {
        const auto & p = odom_snapshot.pose.pose.position;
        const auto & q = odom_snapshot.pose.pose.orientation;
        Eigen::Matrix4f odom_from_base = Eigen::Matrix4f::Identity();
        const Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
        odom_from_base.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
        odom_from_base.block<3,1>(0,3) = Eigen::Vector3f(p.x, p.y, p.z);
        PointCloudXYZI::Ptr transformed(new PointCloudXYZI);
        pcl::transformPointCloud(*stacked, *transformed, odom_from_base);
        return transformed;
    }
    return stacked;
}

bool RobotLocalizationNode::performGlobalSearch()
{
    // 先做全局粗搜，再交给 ICP 精配准。
    const auto start_time = std::chrono::high_resolution_clock::now();
    PointCloudXYZI::Ptr scan_for_icp;
    nav_msgs::msg::Odometry odom_snapshot;
    if (!snapshotLocalizationInput(scan_for_icp, odom_snapshot)) {
        return false;
    }

    const auto &p = odom_snapshot.pose.pose.position;
    const auto &q = odom_snapshot.pose.pose.orientation;
    Eigen::Matrix4f odom_from_base = Eigen::Matrix4f::Identity();
    const Eigen::Quaternionf odom_rotation(q.w, q.x, q.y, q.z);
    odom_from_base.block<3,3>(0,0) = odom_rotation.normalized().toRotationMatrix();
    odom_from_base.block<3,1>(0,3) = Eigen::Vector3f(p.x, p.y, p.z);

    std::vector<Eigen::Matrix4f> candidates;
    try {
        candidates = fast_location::generatePlanarCandidates(
            fast_location::computePlanarBounds(*global_map),
            odom_from_base,
            global_search_config_);
    } catch (const std::exception &error) {
        RCLCPP_ERROR(this->get_logger(), "Cannot generate global search candidates: %s", error.what());
        return false;
    }

    // 先把扫描点云稀疏一点，减少粗搜开销。
    const float coarse_voxel = std::max(0.25f, scan_voxel_size_ * 2.0f);
    const auto coarse_scan = voxelDownSample(scan_for_icp, coarse_voxel);
    const auto ranked = fast_location::scoreGlobalCandidates(
        global_map, coarse_scan, candidates, global_search_config_);
    auto selected = fast_location::selectSeparatedCandidates(ranked, global_search_config_);
    if (selected.empty() || selected.front().score < global_search_config_.minimum_score) {
        const float best_score = selected.empty() ? 0.0f : selected.front().score;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Global search rejected: best coarse score %.3f < %.3f.",
            best_score, global_search_config_.minimum_score);
        return false;
    }

    // 只在少量入选候选上再打一遍精细分数(stride=1、距离更严),打破粗搜的量化粒度。
    // 精细打分用未再降采样的 scan_for_icp,残差更能反映真实对齐情况。
    const float coarse_top_score = selected.front().score;
    selected = fast_location::refineCandidateScores(
        global_map, scan_for_icp, selected, global_search_config_);

    if (selected.empty() || selected.front().score < global_search_config_.minimum_score) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Global search rejected after refinement: best refined score %.3f < %.3f.",
            selected.empty() ? 0.0f : selected.front().score,
            global_search_config_.minimum_score);
        return false;
    }

    const float second_score = selected.size() > 1 ? selected[1].score : 0.0f;
    if (selected.size() > 1 &&
        selected.front().score - second_score < global_search_config_.minimum_score_margin)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Global search ambiguous: best %.3f second %.3f margin %.3f < %.3f.",
            selected.front().score, second_score,
            selected.front().score - second_score,
            global_search_config_.minimum_score_margin);
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Global search: %zu candidates, coarse best=%.3f, refined best=%.3f second=%.3f, refining %zu candidates.",
        candidates.size(), coarse_top_score, selected.front().score, second_score, selected.size());

    // 精化 ICP 阶段允许用更多历史帧堆积,提高稳定性。粗搜依旧用主 scan。
    PointCloudXYZI::Ptr scan_for_refine = scan_for_icp;
    if (global_search_refine_accumulate_ > 1) {
        auto stacked = snapshotAccumulatedScan(global_search_refine_accumulate_);
        if (stacked && !stacked->empty()) {
            scan_for_refine = stacked;
            RCLCPP_DEBUG(
                this->get_logger(),
                "Refine ICP uses %zu accumulated points (target %d frames).",
                scan_for_refine->size(), global_search_refine_accumulate_);
            // 精化扫描换了,清掉之前的下采样缓存。
            scan_downsample_cache_.clear();
        }
    }

    float best_fitness = 0.0f;
    Eigen::Matrix4f best_result = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f best_guess = selected.front().pcd_from_odom;
    for (const auto &candidate : selected) {
        // 只对少量高分候选做逐级精化。
        auto submap = gropGlobalMapInFOV(
            candidate.pcd_from_odom, odom_snapshot, global_search_refine_radius_, false);
        if (!submap || submap->empty()) {
            continue;
        }

        map_downsample_cache_.clear();
        float fitness = 0.0f;
        const auto first = runICP(
            scan_for_refine, submap, candidate.pcd_from_odom,
            3.0f, gicp_max_iterations_first_, fitness);
        const auto second = runICP(
            scan_for_refine, submap, first,
            2.0f, gicp_max_iterations_first_, fitness);
        const auto third = runICP(
            scan_for_refine, submap, second,
            1.5f, gicp_max_iterations_first_, fitness);
        const auto refined = runICP(
            scan_for_refine, submap, third,
            1.0f, gicp_max_iterations_first_, fitness);
        if (fitness > best_fitness) {
            best_fitness = fitness;
            best_result = refined;
            best_guess = candidate.pcd_from_odom;
        }
    }

    if (best_fitness <= first_localization_th_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Global search refinement rejected: fitness %.3f <= %.3f.",
            best_fitness, first_localization_th_);
        return false;
    }

    if (enable_temporal_verification_) {
        // 先把结果挂到 pending,等下一帧扫描再校验一次才算成功。
        pending_global_result_valid_ = true;
        pending_global_result_ = best_result;
        pending_global_guess_ = best_guess;
        pending_global_fitness_ = best_fitness;
        // 要求下一次进 verify 时确实拿到新扫描,不复用同一帧。
        has_new_scan_.store(false, std::memory_order_release);
        RCLCPP_INFO(
            this->get_logger(),
            "Global search pending temporal verification (fitness=%.3f).", best_fitness);
        return false;
    }

    return acceptLocalizationResult(
        best_result, best_fitness, best_guess, scan_for_icp, start_time, true);
}

bool RobotLocalizationNode::verifyPendingGlobalResult()
{
    // 用一帧新扫描在最佳位姿附近打一次分,过阈值才真正接受。
    if (!pending_global_result_valid_) {
        return false;
    }
    if (!has_new_scan_.exchange(false, std::memory_order_acq_rel)) {
        // 还没等到新扫描,先不校验。
        return false;
    }

    const auto start_time = std::chrono::high_resolution_clock::now();
    PointCloudXYZI::Ptr scan_for_verify;
    nav_msgs::msg::Odometry odom_snapshot;
    if (!snapshotLocalizationInput(scan_for_verify, odom_snapshot)) {
        return false;
    }

    const float coarse_voxel = std::max(0.25f, scan_voxel_size_ * 2.0f);
    const auto coarse_scan = voxelDownSample(scan_for_verify, coarse_voxel);
    std::vector<fast_location::GlobalSearchCandidate> single{
        {pending_global_result_, 0.0f}};
    const auto rechecked = fast_location::refineCandidateScores(
        global_map, coarse_scan, single, global_search_config_);
    const float verify_score = rechecked.empty() ? 0.0f : rechecked.front().score;

    if (verify_score < temporal_verification_min_score_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Temporal verification rejected: score %.3f < %.3f. Retrying global search.",
            verify_score, temporal_verification_min_score_);
        pending_global_result_valid_ = false;
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Temporal verification passed (score=%.3f). Accepting pending global result.",
        verify_score);
    const Eigen::Matrix4f result = pending_global_result_;
    const Eigen::Matrix4f guess = pending_global_guess_;
    const float fitness = pending_global_fitness_;
    pending_global_result_valid_ = false;
    return acceptLocalizationResult(
        result, fitness, guess, scan_for_verify, start_time, true);
}

bool RobotLocalizationNode::globalLocalization(const Eigen::Matrix4f &pose_guess)
{
    // 常规定位：先截局部子图，再做多尺度 ICP。
    auto start_time = std::chrono::high_resolution_clock::now();

    PointCloudXYZI::Ptr scan_for_icp;
    nav_msgs::msg::Odometry odom_snapshot;
    if (!snapshotLocalizationInput(scan_for_icp, odom_snapshot)) {
        return false;
    }

    auto submap_start = std::chrono::high_resolution_clock::now();
    auto submap = gropGlobalMapInFOV(pose_guess, odom_snapshot);
    auto submap_end = std::chrono::high_resolution_clock::now();
    auto submap_time = std::chrono::duration_cast<std::chrono::milliseconds>(submap_end - submap_start).count();
    RCLCPP_DEBUG(this->get_logger(), "Submap extraction took %ld ms", submap_time);

    if(!submap || submap->empty())
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Extracted submap is empty, cannot perform localization!");
        return false;
    }

    // 子图每次都会变，先清掉地图下采样缓存。
    map_downsample_cache_.clear();

    float fitness;
    Eigen::Matrix4f T_final;

    auto icp_start = std::chrono::high_resolution_clock::now();
    if(first_localization_)
    {
        // 首次对齐更保守，按大到小尺度逐步收敛。
        auto T_1 = this->runICP(scan_for_icp, submap, pose_guess, 3.0, gicp_max_iterations_first_, fitness);
        auto T_2 = this->runICP(scan_for_icp, submap, T_1, 2.0, gicp_max_iterations_first_, fitness);
        auto T_3 = this->runICP(scan_for_icp, submap, T_2, 1.5, gicp_max_iterations_first_, fitness);
        T_final = this->runICP(scan_for_icp, submap, T_3, 1.0, gicp_max_iterations_first_, fitness);
    }
    else
    {
        // 跟踪：默认单尺度 1.0（逐拍稳定），每 coarse_every_ 拍穿插一次粗→细
        // 逃逸。粗尺度（2.0）体素大、点数少、对应距离阈值放大到 3m，收敛域
        // 更大，能跳出单尺度 ICP 困住的局部最优（累积漂移的根源）；但每拍都跑
        // 会把这种随机逃逸变成逐拍抖动（重复几何里每拍收敛到不同局部最优），
        // 所以降频穿插，只定期拉回漂移，其余拍交给稳定的单尺度 + EMA 平滑。
        ++frame_counter_;
        coarse_escape_ = false;
        if (frame_counter_ >= coarse_every_) {
            frame_counter_ = 0;
            Eigen::Matrix4f T_coarse = this->runICP(
                scan_for_icp, submap, pose_guess, 2.0, gicp_max_iterations_track_, fitness);
            T_final = this->runICP(
                scan_for_icp, submap, T_coarse, 1.0, gicp_max_iterations_track_, fitness);
            // 校正拍彻底吸收：跳出局部最优后的大修正若仍用 0.7 的 EMA，每拍会
            // 剩 30% 累积成漂移。用 recovery_alpha_ 一次到位（漂移本就慢，校正
            // 量小，不会产生可见跳变）。
            coarse_escape_ = true;
        } else {
            T_final = this->runICP(
                scan_for_icp, submap, pose_guess, 1.0, gicp_max_iterations_track_, fitness);
        }
        // 出洞迟滞：连续 degenerate_enter_streak_ 拍退化才认定"在洞里"，
        // 退化→正常的转变（几何恢复，如出隧道）才触发快速收敛。单拍条件数在
        // 阈值附近抖动不会误触发（否则频繁跳过 EMA 平滑，逐拍噪声外露成抖动）。
        if (last_degenerate_) {
            ++degenerate_streak_;
        } else if (degenerate_streak_ >= degenerate_enter_streak_) {
            degeneracy_recovery_ = true;
            degenerate_streak_ = 0;
        } else {
            degenerate_streak_ = 0;
        }
    }
    auto icp_end = std::chrono::high_resolution_clock::now();
    auto icp_time = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start).count();
    RCLCPP_DEBUG(this->get_logger(), "Localization ICP took %ld ms", icp_time);

    // 计算最终位姿和初值的偏移。
    Eigen::Vector3f final_trans = T_final.block<3,1>(0,3);

    // 用偏移量辅助判断这次对齐是否稳定。
    Eigen::Vector3f guess_trans = pose_guess.block<3,1>(0,3);
    Eigen::Vector3f pose_delta = final_trans - guess_trans;
    float pose_shift = pose_delta.norm();

    float th = first_localization_ ? first_localization_th_ : localization_th_;

    if(fitness > th)
    {
        return acceptLocalizationResult(
            T_final, fitness, pose_guess, scan_for_icp, start_time, false);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[FAILED] Localization failed | Fitness: %.4f < %.4f | Pose: (%.2f, %.2f, %.2f) | Shift: %.2fm | Time: %ldms",
        fitness, th, final_trans.x(), final_trans.y(), final_trans.z(), pose_shift, total_time);
    if (!first_localization_) {
        // 本拍失败不会进入 acceptLocalizationResult，出洞标志若残留会在下一拍
        // 被误当成"几何恢复直接接受"。这里清零，避免跨拍泄漏。
        degeneracy_recovery_ = false;
        coarse_escape_ = false;
        degenerate_streak_ = 0;
        handleTrackingFailure();
    }
    return false;

}

bool RobotLocalizationNode::acceptLocalizationResult(
    const Eigen::Matrix4f &result,
    float fitness,
    const Eigen::Matrix4f &pose_guess,
    const PointCloudXYZI::Ptr &scan_for_icp,
    const std::chrono::high_resolution_clock::time_point &start_time,
    bool global_search_result)
{
    // 出洞/几何恢复拍用高 α 快速收敛：退化期间沿退化轴交回 LIO，LIO 累积的
    // 漂移在几何恢复后由 ICP 一次性找回；若仍走 0.7 的 EMA 会抹成数秒爬行。
    // 保留少量平滑（recovery_alpha_≈0.95）防止本拍单点噪声完全外露成抖动。
    const bool recovered = degeneracy_recovery_ || coarse_escape_;
    degeneracy_recovery_ = false;
    coarse_escape_ = false;
    {
        std::lock_guard<std::mutex> lock(tf_mutex);
        // EMA 平滑：防止定位更新在低频（0.5Hz）下产生 map→odom 的突变跳帧。
        // 借鉴 B（HWSentryNav26）的 odom_localizer 平滑策略；α=0.7 时每拍吸收
        // 70% 新结果，约 3 个更新周期（6s）完全收敛。
        // 全局重定位/首次后直接接受（ema_initialized_ 清零），避免从错误旧位置插值。
        if (!ema_initialized_ || global_search_result) {
            ema_transform_ = result;
            ema_initialized_ = true;
        } else {
            // 在 SE(2) 上做线性插值（z 轴忽略，平面机器人）。
            // 出洞恢复拍临时用 recovery_alpha_（≈0.95）快速吸收大修正，其余用 ema_alpha_。
            const float alpha = recovered
                ? static_cast<float>(std::clamp(recovery_alpha_, 0.0, 1.0))
                : static_cast<float>(std::clamp(ema_alpha_, 0.0, 1.0));
            // 平移插值
            ema_transform_.block<3,1>(0,3) =
                alpha * result.block<3,1>(0,3) +
                (1.0f - alpha) * ema_transform_.block<3,1>(0,3);
            // 旋转插值：提取各自的 yaw，最短路径插值
            const double yaw_new = std::atan2(result(1, 0), result(0, 0));
            const double yaw_old = std::atan2(ema_transform_(1, 0), ema_transform_(0, 0));
            double dyaw = yaw_new - yaw_old;
            while (dyaw > M_PI) { dyaw -= 2.0 * M_PI; }
            while (dyaw < -M_PI) { dyaw += 2.0 * M_PI; }
            const double yaw_smoothed = yaw_old + alpha * dyaw;
            // 重建旋转部分（只改 z 轴旋转，保留原来的 roll/pitch）
            const float cy = static_cast<float>(std::cos(yaw_smoothed));
            const float sy = static_cast<float>(std::sin(yaw_smoothed));
            ema_transform_(0, 0) = cy; ema_transform_(0, 1) = -sy;
            ema_transform_(1, 0) = sy; ema_transform_(1, 1) =  cy;
        }
        T_pcd_to_odom = ema_transform_;
        tf_ready_ = true;
    }
    tracking_recovery_.recordSuccess();

    const Eigen::Matrix4f map_from_odom =
        fast_location::composeMapToOdom(map_from_pcd_, result);
    const Eigen::Vector3f pcd_translation = result.block<3,1>(0,3);
    const Eigen::Vector3f map_translation = map_from_odom.block<3,1>(0,3);
    const Eigen::Quaternionf map_rotation(map_from_odom.block<3,3>(0,0));
    const double pcd_yaw = std::atan2(result(1, 0), result(0, 0));
    const double map_yaw = std::atan2(map_from_odom(1, 0), map_from_odom(0, 0));
    const float pose_shift =
        (pcd_translation - pose_guess.block<3,1>(0,3)).norm();

    if (hasPointCloudSubscribers(pub_pc_in_map)) {
        const auto scan_in_map = transformCloud(*scan_for_icp, map_from_odom);
        publishPointCloudIfSubscribed(pub_pc_in_map, *scan_in_map, map_frame_, this->now());
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.position.x = map_translation.x();
    odom_msg.pose.pose.position.y = map_translation.y();
    odom_msg.pose.pose.position.z = map_translation.z();
    odom_msg.pose.pose.orientation.x = map_rotation.x();
    odom_msg.pose.pose.orientation.y = map_rotation.y();
    odom_msg.pose.pose.orientation.z = map_rotation.z();
    odom_msg.pose.pose.orientation.w = map_rotation.w();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;
    if (publish_map_to_odometry_ && pub_map_to_odometry) {
        pub_map_to_odometry->publish(odom_msg);
    }

    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = odom_msg.header;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = map_translation.x();
        tf_msg.transform.translation.y = map_translation.y();
        tf_msg.transform.translation.z = map_translation.z();
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    const auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count();
    const bool was_first_localization = first_localization_;
    first_localization_ = false;

    RCLCPP_DEBUG(
        this->get_logger(),
        "Localization result PCD->odom: [%.3f, %.3f, %.1fdeg], map->odom: [%.3f, %.3f, %.1fdeg]",
        pcd_translation.x(), pcd_translation.y(), pcd_yaw * 180.0 / M_PI,
        map_translation.x(), map_translation.y(), map_yaw * 180.0 / M_PI);

    if (global_search_result) {
        RCLCPP_INFO(
            this->get_logger(),
            "Global localization accepted: PCD->odom=(%.2f, %.2f, %.1fdeg), map->odom=(%.2f, %.2f, %.1fdeg), fitness=%.4f, time=%ldms.",
            pcd_translation.x(), pcd_translation.y(), pcd_yaw * 180.0 / M_PI,
            map_translation.x(), map_translation.y(), map_yaw * 180.0 / M_PI,
            fitness, total_time);
    } else if (was_first_localization) {
        RCLCPP_INFO(
            this->get_logger(),
            "[SUCCESS] First localization | Fitness: %.4f | Pose: (%.2f, %.2f, %.2f) Yaw: %.1f° | Shift: %.2fm | Time: %ldms",
            fitness, pcd_translation.x(), pcd_translation.y(), pcd_translation.z(),
            pcd_yaw * 180.0 / M_PI, pose_shift, total_time);
    } else {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[SUCCESS] Tracking | Fitness: %.4f | Pose: (%.2f, %.2f, %.2f) | Shift: %.2fm | Time: %ldms",
            fitness, pcd_translation.x(), pcd_translation.y(), pcd_translation.z(),
            pose_shift, total_time);
    }
    return true;
}

void RobotLocalizationNode::handleTrackingFailure()
{
    if (!tracking_recovery_.recordFailure()) {
        return;
    }

    // 连续失败到阈值后，回退到全局搜索。
    initialized = false;
    first_localization_ = true;
    {
        std::lock_guard<std::mutex> lock(tf_mutex);
        tf_ready_ = false;
    }
    has_new_scan_.store(true, std::memory_order_release);
    RCLCPP_WARN(
        this->get_logger(),
        "Tracking lost after %zu consecutive failures; returning to full global search.",
        tracking_recovery_.failureCount());
}




Eigen::Matrix4f RobotLocalizationNode::runICP(PointCloudXYZI::Ptr src, PointCloudXYZI::Ptr tgt, const Eigen::Matrix4f &initial_guess, float voxel_scale, int max_iterations, float &fitness_score)
{
    auto icp_start = std::chrono::high_resolution_clock::now();

    if(!src || src->empty() || !tgt || tgt->empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Source or target point cloud is empty!");
        fitness_score = 0.0f;
        return Eigen::Matrix4f::Identity();
    }

    auto downsample_start = std::chrono::high_resolution_clock::now();

    // 用缓存避免重复下采样。
    int scale_key = static_cast<int>(voxel_scale * 10);  // 转为整数键

    PointCloudXYZI::Ptr src_filtered;
    if (scan_downsample_cache_.find(scale_key) != scan_downsample_cache_.end()) {
        src_filtered = scan_downsample_cache_[scale_key];
    } else {
        src_filtered = voxelDownSample(src, scan_voxel_size_ * voxel_scale);
        scan_downsample_cache_[scale_key] = src_filtered;
    }

    // 地图点云按尺度决定是否再下采样。
    PointCloudXYZI::Ptr tgt_filtered;
    if (voxel_scale > 1.0) {
        if (map_downsample_cache_.find(scale_key) != map_downsample_cache_.end()) {
            tgt_filtered = map_downsample_cache_[scale_key];
        } else {
            tgt_filtered = voxelDownSample(tgt, map_voxel_size_ * voxel_scale);
            map_downsample_cache_[scale_key] = tgt_filtered;
        }
    } else {
        tgt_filtered = tgt;
    }

    auto downsample_end = std::chrono::high_resolution_clock::now();
    auto downsample_time = std::chrono::duration_cast<std::chrono::milliseconds>(downsample_end - downsample_start).count();
    RCLCPP_DEBUG(this->get_logger(), "ICP downsampling took %ld ms", downsample_time);

    const auto debug_stamp = this->now();
    publishPointCloudIfSubscribed(pub_scan_downsampled, *src_filtered, base_frame_, debug_stamp);
    publishPcdCloudInMap(pub_map_downsampled, *tgt_filtered, debug_stamp);

    auto gicp_start = std::chrono::high_resolution_clock::now();

    PointCloudXYZI::Ptr aligned(new PointCloudXYZI);
    Eigen::Matrix4f final_transformation;
    bool converged = false;

    if (use_cuda_) {
        RCLCPP_WARN_ONCE(this->get_logger(),
            "CUDA requested but not available in this build. Using CPU FastGICP instead.");
        use_cuda_ = false;
    }

    fast_gicp::FastGICP<Point, Point> gicp;
    gicp.setNumThreads(gicp_num_threads_ > 0 ? gicp_num_threads_ : 4);
    gicp.setInputSource(src_filtered);
    gicp.setInputTarget(tgt_filtered);
    gicp.setMaximumIterations(max_iterations);
    gicp.setMaxCorrespondenceDistance(1.5f * voxel_scale);
    gicp.setRANSACOutlierRejectionThreshold(1.5f * voxel_scale);
    gicp.setCorrespondenceRandomness(20);

    gicp.align(*aligned, initial_guess);
    converged = gicp.hasConverged();
    final_transformation = gicp.getFinalTransformation();
    // 6×6 Hessian（[rot, trans] 排序）是可观测性的直接来源，比点云空间分布准。
    const Eigen::Matrix<double, 6, 6> hessian = gicp.getFinalHessian();

    auto gicp_end = std::chrono::high_resolution_clock::now();
    auto gicp_time = std::chrono::duration_cast<std::chrono::milliseconds>(gicp_end - gicp_start).count();
    RCLCPP_DEBUG(this->get_logger(), "FastGICP solve took %ld ms", gicp_time);

    if (!final_transformation.allFinite())
    {
        fitness_score = 0.0f;
        RCLCPP_WARN(this->get_logger(), "GICP returned a non-finite transformation.");
        return Eigen::Matrix4f::Identity();
    }

    auto fitness_start = std::chrono::high_resolution_clock::now();
    const auto assessment = fast_location::assessAlignment(
        converged, aligned, tgt_filtered, 1.5f * voxel_scale,
        static_cast<float>(degenerate_condition_threshold_), &hessian);
    if (!assessment.usable) {
        fitness_score = 0.0f;
        RCLCPP_WARN(this->get_logger(), "GICP returned no usable aligned cloud.");
        return Eigen::Matrix4f::Identity();
    }
    fitness_score = assessment.inlier_ratio;
    // 记录本拍退化状态，供出洞检测（退化→正常的转变）在 acceptLocalizationResult 使用。
    last_degenerate_ = assessment.is_degenerate;

    // 可观测性门控。长走廊场景下点云沿走廊方向几乎没有匹配约束，GICP 在该
    // 方向给出的平移修正是噪声。
    //
    // 这里不整拍丢弃：fast_location 是本工作空间唯一的 map→odom 来源，整拍
    // 返回单位矩阵会让 TF 冻结，定位精度完全交给 LIO 开环漂移。改为把修正
    // 投影到可观测方向——垂直于走廊的那一维仍然可信，照常修正；沿走廊方向
    // 交回初值，信任 LIO 推算。
    //
    // 只有条件数极端到连旋转都不可信时才彻底放弃（见下面的 hard reject）。
    if (assessment.is_degenerate) {
        if (assessment.condition_number > degenerate_hard_reject_condition_) {
            fitness_score = 0.0f;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Localization rejected: scan is fully unconstrained "
                "(condition_number=%.1f > hard limit=%.1f). Using previous transform.",
                assessment.condition_number,
                static_cast<float>(degenerate_hard_reject_condition_));
            return Eigen::Matrix4f::Identity();
        }

        const Eigen::Matrix4f projected = fast_location::projectToObservableSubspace(
            final_transformation, initial_guess, assessment.observable_direction);
        const Eigen::Vector2f dropped =
            final_transformation.block<2, 1>(0, 3) - projected.block<2, 1>(0, 3);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Localization degenerate (condition_number=%.1f > threshold=%.1f, likely a "
            "corridor): keeping correction along observable axis (%.2f, %.2f), dropped "
            "%.3f m along the unobservable axis.",
            assessment.condition_number, static_cast<float>(degenerate_condition_threshold_),
            assessment.observable_direction.x(), assessment.observable_direction.y(),
            dropped.norm());
        final_transformation = projected;
    }
    auto fitness_end = std::chrono::high_resolution_clock::now();
    auto fitness_time = std::chrono::duration_cast<std::chrono::milliseconds>(fitness_end - fitness_start).count();

    auto icp_end = std::chrono::high_resolution_clock::now();
    auto total_icp_time = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start).count();
    RCLCPP_DEBUG(this->get_logger(), "ICP fitness evaluation took %ld ms, total ICP took %ld ms",
                 fitness_time, total_icp_time);

    if (converged) {
        RCLCPP_DEBUG(this->get_logger(), "GICP converged, fitness: %.4f", fitness_score);
    } else {
        RCLCPP_DEBUG(
            this->get_logger(),
            "GICP reached its iteration limit; using measured fitness %.4f",
            fitness_score);
    }
    return final_transformation;
}







PointCloudXYZI::Ptr RobotLocalizationNode::gropGlobalMapInFOV(
    const Eigen::Matrix4f &pose_set,
    const nav_msgs::msg::Odometry &odom,
    float search_radius,
    bool publish_debug_cloud)
{
    if(!global_map || global_map->empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Global map is empty!");
        return nullptr;
    }

    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    if(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w))
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry contains NaN position!");
        return nullptr;
    }

    Eigen::Matrix4f T_odom_to_base = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    T_odom_to_base.block<3,3>(0,0) = quat.toRotationMatrix();
    T_odom_to_base.block<3,1>(0,3) = Eigen::Vector3f(p.x, p.y, p.z);

    Eigen::Matrix4f T_map_to_base =  pose_set* T_odom_to_base;
    Eigen::Matrix4f T_base_to_map = this->inverseSE3(T_map_to_base);

    if (T_base_to_map.hasNaN())
    {
        RCLCPP_ERROR(this->get_logger(), "Transformation matrix contains NaN values!");
        return nullptr;
    }


    Eigen::Vector3f robot_pos = T_map_to_base.block<3,1>(0,3);
    Point search_point;
    search_point.x = robot_pos(0);
    search_point.y = robot_pos(1);
    search_point.z = robot_pos(2);
    search_point.intensity = 0.0f;
    if (search_radius <= 0.0f) {
        search_radius = fov_far_;
    }
    std::vector<int> indices;
    std::vector<float> distances;

    RCLCPP_DEBUG(this->get_logger(), ">>> [Submap] Searching within %.2fm radius around (%.2f, %.2f, %.2f)",
                search_radius, robot_pos(0), robot_pos(1), robot_pos(2));

    if (kdtree_global_map->radiusSearch(search_point, search_radius, indices, distances) > 0)
    {
        // 这里直接按索引拎点，少做一次拷贝。
        PointCloudXYZI::Ptr submap(new PointCloudXYZI);
        submap->reserve(indices.size());
        for (int idx : indices) {
            submap->push_back(global_map->points[idx]);
        }

        RCLCPP_DEBUG(this->get_logger(), ">>> [Submap] Extracted %zu points from global map", submap->size());

        // 只有需要时才再降一次采样。
        float target_voxel = first_localization_ ? submap_voxel_size_first_ : submap_voxel_size_track_;
        if (target_voxel > 0.0f && target_voxel > map_voxel_size_) {
            size_t before_size = submap->size();
            submap = voxelDownSample(submap, target_voxel);
            float submap_reduction = 100.0 * (1.0 - (float)submap->size() / (float)before_size);
            RCLCPP_DEBUG(this->get_logger(), ">>> [Submap Downsample] Voxel size: %.2fm, %zu -> %zu points (reduced %.1f%%)",
                        target_voxel, before_size, submap->size(), submap_reduction);
        } else {
            RCLCPP_DEBUG(this->get_logger(), ">>> [Submap] No additional downsampling needed (target_voxel: %.2fm <= map_voxel: %.2fm)",
                        target_voxel, map_voxel_size_);
        }

        RCLCPP_DEBUG(this->get_logger(), ">>> [Submap] Final submap size: %zu points", submap->size());
        if (publish_debug_cloud) {
            publishPcdCloudInMap(pub_submap, *submap, this->now());
        }

        return submap;
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "No points found within search radius!");
        return nullptr;
    }
    
}


Eigen::Matrix4f RobotLocalizationNode::inverseSE3(const Eigen::Matrix4f &T)
{
    Eigen::Matrix4f inv = Eigen::Matrix4f::Identity();
    inv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();  // R^T
    inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);  // -R^T * t
    return inv;
}


Eigen::Matrix4f RobotLocalizationNode::poseToMat(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;

    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    mat.block<3,3>(0,0) = quat.toRotationMatrix();
    mat.block<3,1>(0,3) = Eigen::Vector3f(p.x, p.y, p.z);
    return mat;
}


namespace
{
std::string resolvePackageUrl(const std::string &path)
{
    const std::string prefix = "package://";
    if (path.rfind(prefix, 0) != 0) {
        return path;
    }

    const std::string package_path = path.substr(prefix.size());
    const auto separator = package_path.find('/');
    if (separator == std::string::npos) {
        throw std::runtime_error("package URL must include a package name and relative path: " + path);
    }

    const std::string package_name = package_path.substr(0, separator);
    const std::string relative_path = package_path.substr(separator + 1);
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) /
            relative_path)
        .string();
}
}  // namespace

bool RobotLocalizationNode::loadGlobalMap(const std::string &pcd_file_path)
{
    std::lock_guard<std::mutex> lock(data_mutex);

    std::string resolved_pcd_file_path;
    try {
        resolved_pcd_file_path = resolvePackageUrl(pcd_file_path);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to resolve PCD path '%s': %s", pcd_file_path.c_str(), e.what());
        return false;
    }

    PointCloudXYZI::Ptr raw_map(new PointCloudXYZI);
    if (pcl::io::loadPCDFile<Point>(resolved_pcd_file_path, *raw_map) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", resolved_pcd_file_path.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded global map: %zu points from %s", raw_map->size(), resolved_pcd_file_path.c_str());

    // 对全局地图进行下采样,减少点云数量
    RCLCPP_DEBUG(this->get_logger(), ">>> [Downsampling] Applying voxel filter with size: %.2fm", map_voxel_size_);
    auto downsample_start = std::chrono::high_resolution_clock::now();
    global_map = voxelDownSample(raw_map, map_voxel_size_);
    auto downsample_end = std::chrono::high_resolution_clock::now();
    auto downsample_time = std::chrono::duration_cast<std::chrono::milliseconds>(downsample_end - downsample_start).count();

    float reduction_ratio = 100.0 * (1.0 - (float)global_map->size() / (float)raw_map->size());
    RCLCPP_DEBUG(this->get_logger(), ">>> [Downsampling] Result: %zu -> %zu points (reduced %.1f%%) in %ld ms",
                raw_map->size(), global_map->size(), reduction_ratio, downsample_time);

    return true;
}


PointCloudXYZI::Ptr RobotLocalizationNode::voxelDownSample(PointCloudXYZI::Ptr cloud, float voxel_size)
{
    PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI);
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*cloud_filtered);
    return cloud_filtered;
}


void RobotLocalizationNode::extractFeatures(PointCloudXYZI::Ptr cloud,
                                            PointCloudXYZI::Ptr edge_features,
                                            PointCloudXYZI::Ptr planar_features,
                                            int num_neighbors,
                                            float edge_threshold,
                                            float planar_threshold)
{
    if (!cloud || cloud->empty())
    {
        RCLCPP_WARN(this->get_logger(), "Input cloud is empty for feature extraction!");
        return;
    }

    // 构建KD树用于近邻搜索
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<float> curvatures(cloud->size(), 0.0f);
    std::vector<bool> valid_point(cloud->size(), true);

    // 计算每个点的曲率
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const Point& point = cloud->points[i];

        // 跳过无效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            valid_point[i] = false;
            continue;
        }

        std::vector<int> indices(num_neighbors);
        std::vector<float> distances(num_neighbors);

        // 查邻域点。
        if (kdtree.nearestKSearch(point, num_neighbors, indices, distances) < num_neighbors)
        {
            valid_point[i] = false;
            continue;
        }

        // 计算邻域中心点
        Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
        for (int idx : indices)
        {
            center.x() += cloud->points[idx].x;
            center.y() += cloud->points[idx].y;
            center.z() += cloud->points[idx].z;
        }
        center /= static_cast<float>(num_neighbors);

        // 计算协方差矩阵
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (int idx : indices)
        {
            Eigen::Vector3f diff(
                cloud->points[idx].x - center.x(),
                cloud->points[idx].y - center.y(),
                cloud->points[idx].z - center.z()
            );
            covariance += diff * diff.transpose();
        }
        covariance /= static_cast<float>(num_neighbors);

        // 特征值分解
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f eigenvalues = solver.eigenvalues();

        // 特征值按升序排列: λ0 <= λ1 <= λ2
        float lambda0 = eigenvalues(0);
        float lambda1 = eigenvalues(1);
        float lambda2 = eigenvalues(2);

        float sum = lambda0 + lambda1 + lambda2;
        if (sum < 1e-6f)
        {
            valid_point[i] = false;
            continue;
        }

        // 计算曲率 (最小特征值 / 特征值之和)
        // 平面点: 曲率小 (λ0 << λ1 ≈ λ2)
        // 边缘点: 曲率大 (λ0 ≈ λ1 << λ2)
        curvatures[i] = lambda0 / sum;
    }

    // 根据曲率分类点
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!valid_point[i])
            continue;

        const Point& point = cloud->points[i];

        if (curvatures[i] > edge_threshold)
        {
            // 高曲率 -> 边缘点
            edge_features->push_back(point);
        }
        else if (curvatures[i] < planar_threshold)
        {
            // 低曲率 -> 平面点
            planar_features->push_back(point);
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Feature extraction: %zu edge points, %zu planar points from %zu input points",
                edge_features->size(), planar_features->size(), cloud->size());
}






void RobotLocalizationNode::SubScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    PointCloudXYZI::Ptr scan_in(new PointCloudXYZI);
    pcl::fromROSMsg(*msg, *scan_in);
    if (scan_in->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Incoming scan is empty, skip.");
        return;
    }

    PointCloudXYZI::Ptr scan_filtered(new PointCloudXYZI);
    scan_filtered->reserve(scan_in->size());
    const float min_range_sq = scan_min_range_ > 0.0f ? scan_min_range_ * scan_min_range_ : 0.0f;
    const float max_range_sq = scan_max_range_ > 0.0f ? scan_max_range_ * scan_max_range_ : 0.0f;
    for (const auto& pt : scan_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        const float d2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (d2 < min_range_sq) {
            continue;
        }
        if (max_range_sq > 0.0f && d2 > max_range_sq) {
            continue;
        }
        scan_filtered->push_back(pt);
    }

    if (scan_filtered->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Filtered scan is empty, skip.");
        return;
    }

    // 缩小锁范围，减少阻塞。
    // buffer 容量要能同时喂饱常规跟踪和全局搜索的精化阶段。
    const int buffer_capacity = std::max(scan_accumulate_frames_, global_search_refine_accumulate_);
    PointCloudXYZI::Ptr stacked_scan(new PointCloudXYZI);
    std::deque<PointCloudXYZI::Ptr> buffer_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        scan_buffer_.push_back(scan_filtered);
        while (static_cast<int>(scan_buffer_.size()) > buffer_capacity) {
            scan_buffer_.pop_front();
        }
        // 只拷贝跟踪需要的最新几帧,避免每帧都在锁外拷太多。
        const int keep = std::min<int>(scan_accumulate_frames_, scan_buffer_.size());
        buffer_copy.insert(
            buffer_copy.end(), scan_buffer_.end() - keep, scan_buffer_.end());
    }

    // 在锁外合并点云，避免卡住其他回调。
    std::size_t total_pts = 0;
    for (const auto& cloud : buffer_copy) {
        total_pts += cloud->size();
    }
    stacked_scan->reserve(total_pts);
    for (const auto& cloud : buffer_copy) {
        *stacked_scan += *cloud;
    }

    // 更新当前定位使用的扫描。
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        *cur_scan = *stacked_scan;
        has_new_scan_.store(true, std::memory_order_release);
        // 新扫描到了，清空旧缓存。
        scan_downsample_cache_.clear();
    }

}


void RobotLocalizationNode::SubOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    cur_odom = msg;
    odom_received_.store(true, std::memory_order_release);
}


void RobotLocalizationNode::subInitPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 外部初始位姿进来后，重置重定位状态。
    if (!odom_received_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Initial pose received before odometry; ignoring it.");
        return;
    }

    nav_msgs::msg::Odometry odom_snapshot;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        odom_snapshot = *cur_odom;
    }

    const Eigen::Matrix4f map_from_base = this->poseToMat(msg);
    Eigen::Matrix4f odom_from_base = Eigen::Matrix4f::Identity();
    const auto & position = odom_snapshot.pose.pose.position;
    const auto & orientation = odom_snapshot.pose.pose.orientation;
    Eigen::Quaternionf odom_rotation(
        orientation.w, orientation.x, orientation.y, orientation.z);
    odom_from_base.block<3, 3>(0, 0) = odom_rotation.toRotationMatrix();
    odom_from_base.block<3, 1>(0, 3) =
        Eigen::Vector3f(position.x, position.y, position.z);

    const Eigen::Matrix4f pcd_from_odom = fast_location::initialPoseToPcdOdom(
        map_from_pcd_, map_from_base, odom_from_base);
    if (!fast_location::isFiniteTransform(pcd_from_odom)) {
        RCLCPP_WARN(this->get_logger(), "Initial pose conversion produced a non-finite transform.");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex);
        scan_buffer_.clear();
        cur_scan->clear();
        has_new_scan_.store(false, std::memory_order_release);
    }
    {
        std::lock_guard<std::mutex> lock(tf_mutex);
        initial_pcd_to_odom = pcd_from_odom;
        T_pcd_to_odom = pcd_from_odom;
        tf_ready_ = false;
    }
    initial_pose_received.store(true, std::memory_order_release);
    // 唤醒定位线程。
    initial_pose_cv_.notify_one();
    first_localization_ = true;
    // 用户重新指定初始位姿,丢弃之前挂起的全局搜索结果。
    pending_global_result_valid_ = false;

    RCLCPP_INFO(this->get_logger(), "Initial pose received, reset to multi-scale mode.");
}

void RobotLocalizationNode::subInitPoseWithCovariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // 兼容带协方差的初始位姿话题，复用同一套处理。
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header = msg->header;
    pose_msg->pose = msg->pose.pose;
    subInitPose(pose_msg);
}

void RobotLocalizationNode::publishMapToOdomTf()
{
    // 把当前重定位结果转成 map->odom TF。
    if (!publish_tf_ || !tf_broadcaster_) {
        return;
    }

    Eigen::Matrix4f T;
    bool tf_ready = false;
    {
        std::lock_guard<std::mutex> lock(tf_mutex);
        tf_ready = tf_ready_;
        if (!tf_ready) {
            return;
        }
        T = fast_location::composeMapToOdom(map_from_pcd_, T_pcd_to_odom);
    }

    // map->odom TF 用当前时钟（now()）作为时间戳——标准做法（与 Nav2 amcl 一致）。
    // 若用 odom 消息时间戳（滞后），下游用 now()/scan 时间戳查询时反而
    // 报 extrapolation into the future/past，加剧抖动。
    Eigen::Vector3f trans = T.block<3,1>(0,3);
    Eigen::Quaternionf rot(T.block<3,3>(0,0));

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = trans.x();
    tf_msg.transform.translation.y = trans.y();
    tf_msg.transform.translation.z = trans.z() + map_z_offset_;  // 应用 Z 轴偏移补偿
    tf_msg.transform.rotation.x = rot.x();
    tf_msg.transform.rotation.y = rot.y();
    tf_msg.transform.rotation.z = rot.z();
    tf_msg.transform.rotation.w = rot.w();
    tf_broadcaster_->sendTransform(tf_msg);
}

bool RobotLocalizationNode::hasPointCloudSubscribers(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const
{
    return publisher &&
           (publisher->get_subscription_count() +
            publisher->get_intra_process_subscription_count()) > 0;
}

void RobotLocalizationNode::publishPointCloudIfSubscribed(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloudXYZI & cloud,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
{
    if (!hasPointCloudSubscribers(publisher)) {
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    publisher->publish(msg);
}

PointCloudXYZI::Ptr RobotLocalizationNode::transformCloud(
    const PointCloudXYZI & cloud,
    const Eigen::Matrix4f & transform) const
{
    PointCloudXYZI::Ptr transformed(new PointCloudXYZI);
    pcl::transformPointCloud(cloud, *transformed, transform);
    return transformed;
}

void RobotLocalizationNode::publishPcdCloudInMap(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloudXYZI & cloud,
    const builtin_interfaces::msg::Time & stamp)
{
    if (!hasPointCloudSubscribers(publisher)) {
        return;
    }

    const auto map_cloud = transformCloud(cloud, map_from_pcd_);
    publishPointCloudIfSubscribed(publisher, *map_cloud, map_frame_, stamp);
}

void RobotLocalizationNode::publishGlobalMap()
{
    // 把加载的全局地图发布出去，供外部可视化或调试。
    if (!global_map || global_map->empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Global map is empty, skipping publish");
        return;
    }

    if (!hasPointCloudSubscribers(pub_global_map)) {
        return;
    }

    if (!global_map_msg_ready_) {
        const auto map_cloud = transformCloud(*global_map, map_from_pcd_);
        pcl::toROSMsg(*map_cloud, global_map_msg_);
        global_map_msg_.header.frame_id = map_frame_;
        global_map_msg_ready_ = true;
    }

    global_map_msg_.header.stamp = this->now();
    pub_global_map->publish(global_map_msg_);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Publishing global map with %zu points", global_map->size());
}



RobotLocalizationNode::~RobotLocalizationNode()
{

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);
    auto node = std::make_shared<RobotLocalizationNode>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
