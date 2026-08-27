#ifndef RM_NAVIGATION2__GRID_UTILS_HPP_
#define RM_NAVIGATION2__GRID_UTILS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

/*

 ██╗  ██╗  ██████╗  ███╗   ██╗  ██████╗  ██╗       ██████╗  ███╗   ██╗  ██████╗ 
 ██║  ██║ ██╔═══██╗ ████╗  ██║ ██╔════╝  ██║      ██╔═══██╗ ████╗  ██║ ██╔════╝ 
 ███████║ ██║   ██║ ██╔██╗ ██║ ██║  ███╗ ██║      ██║   ██║ ██╔██╗ ██║ ██║  ███╗
 ██╔══██║ ██║   ██║ ██║╚██╗██║ ██║   ██║ ██║      ██║   ██║ ██║╚██╗██║ ██║   ██║
 ██║  ██║ ╚██████╔╝ ██║ ╚████║ ╚██████╔╝ ███████╗ ╚██████╔╝ ██║ ╚████║ ╚██████╔╝
 ╚═╝  ╚═╝  ╚═════╝  ╚═╝  ╚═══╝  ╚═════╝  ╚══════╝  ╚═════╝  ╚═╝  ╚═══╝  ╚═════╝ 

*/

namespace navigation2
{

struct GridCell
{
  int x{0};
  int y{0};
};

double normalizeAngle(double angle);

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

bool inBounds(const nav_msgs::msg::OccupancyGrid & grid, int x, int y);

std::size_t gridIndex(const nav_msgs::msg::OccupancyGrid & grid, int x, int y);

bool worldToMap(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y, int & map_x,
  int & map_y);

void mapToWorld(
  const nav_msgs::msg::OccupancyGrid & grid, int map_x, int map_y, double & world_x,
  double & world_y);

bool isOccupied(int8_t value, int occupied_threshold, bool unknown_is_occupied);

void inflateOccupancyGrid(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold = 50);

// radius_limit 给每格一个膨胀半径上限（m），长度必须等于 grid.data.size()，否则
// 整个上限被忽略。用于隧道：窄通道容不下全局那么大的膨胀半径，见
// semantic_map_consumer.hpp 的 makeInflationRadiusLimit。空 vector = 无逐格上限。
void applyInflationCostGradient(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold = 50,
  double cost_scaling_factor = 8.0, const std::vector<float> & radius_limit = {});

// —— 下列 *_EDT 变体与上方两个函数输出逐字节等价，复杂度从
// O(障碍格 × 核体积) 降到 O(格数)（Felzenszwalb 精确距离变换，无堆），
// 大图高频膨胀时不再随障碍数量线性爆炸。调用方无脑切换即可。——

void inflateOccupancyGridEDT(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold = 50);

void applyInflationCostGradientEDT(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold = 50,
  double cost_scaling_factor = 8.0, const std::vector<float> & radius_limit = {});

std::vector<GridCell> raytraceLine(int x0, int y0, int x1, int y1);

geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::TransformStamped & transform, double x, double y, double z);

// 每帧一次预计算（点云→全局平面变换的展开形式）：点级热路径不再反复
// 构造 tf2::Transform / 做四元数到矩阵。仅使用平移+yaw 分量（栅格/高度
// 判定均只关心平面投影与 z 线性分量；z 变换 = R22*z + tz 对固定横滚成立，
// 本仓 TF 树不引入横滚）。
struct PlanarFrame
{
  double cos_yaw{1.0}, sin_yaw{0.0};
  double tx{0.0}, ty{0.0}, tz{0.0};
  // R33 与 R31/R32 由 yaw+roll/pitch 决定；这里用完整矩阵的 (2,*) 行。
  double m20{1.0}, m21{0.0}, m22{1.0};
};

PlanarFrame makePlanarFrame(const geometry_msgs::msg::TransformStamped & transform);

geometry_msgs::msg::Point applyPlanarFrame(const PlanarFrame & f, double x, double y, double z);

geometry_msgs::msg::Point transformPointPlanar(
  const geometry_msgs::msg::TransformStamped & transform, double x, double y, double z);

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame_id, const rclcpp::Time & stamp, double x, double y, double yaw);

}  // namespace navigation2

#endif  // RM_NAVIGATION2__GRID_UTILS_HPP_
