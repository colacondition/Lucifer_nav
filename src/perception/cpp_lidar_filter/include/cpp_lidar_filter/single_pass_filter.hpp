#ifndef CPP_LIDAR_FILTER__SINGLE_PASS_FILTER_HPP_
#define CPP_LIDAR_FILTER__SINGLE_PASS_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cpp_lidar_filter
{

struct SinglePassParams
{
  double range{10.0};       // 水平半径（navigation 系，含边界）
  double body_min_x{-0.4};  // 云台自车结构 box 边界（navigation=base_link=雷达系）
  double body_max_x{0.4};
  double body_min_y{-0.3};
  double body_max_y{0.3};
  double body_min_z{-0.1};
  double body_max_z{0.6};
  bool negative{true};  // true = 挖掉 box 内的点，保留 box 外
  double leaf{0.06};    // 近似体素边长（按输入系坐标量化，每格保留首个点）
};

// 单遍过滤：一次遍历完成「变换到 navigation 系 → 水平半径裁剪 → 云台自车结构
// box → 近似体素降采样」。替代旧的 toPCL→ExtractIndices→CropBox→VoxelGrid→fromPCL
// 五连（每帧 4~5 次全量拷贝）。
//
// 输出 PointXYZI 的点坐标仍在输入系（下游地面分割按输入 frame 解释几何，
// 坐标变换只用于判定）；intensity 字段存在则保留，否则填 0。
// 输入 x/y/z 必须是 FLOAT32/FLOAT64 的单字段布局；PointCloud2 行允许
// row_step >= width*point_step 的 padding（不再要求严格 packed）。空点云是有效
// 输入：返回 true 且输出为空。布局非法（缺 z、字段截断、数据区不足等）返回 false。
bool filterSinglePass(
  const sensor_msgs::msg::PointCloud2 & input, const Eigen::Affine3d & navigation_from_cloud,
  const SinglePassParams & params, pcl::PointCloud<pcl::PointXYZI> & output);

}  // namespace cpp_lidar_filter

#endif  // CPP_LIDAR_FILTER__SINGLE_PASS_FILTER_HPP_
