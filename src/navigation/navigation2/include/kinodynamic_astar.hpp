#ifndef NAVIGATION2__KINODYNAMIC_ASTAR_HPP_
#define NAVIGATION2__KINODYNAMIC_ASTAR_HPP_

// 二维全向 Kinodynamic A*：状态 [x, y, vx, vy]，控制为加速度。
//
// 算法来源：TDT-nav-kit 的 KinodynamicAstar（MIT License，
// Copyright (c) 2026 Rongxuan Ye and BoLin Li），其本身是 Fast-Planner 的
// 「Robust and Efficient Quadrotor Trajectory Generation」一系做法在 2D 全向
// 场景的落地。按 Lucifer 的模块纪律重写，三处有意的差异：
//
// 1. 无状态。TDT 是 setMap() + search() 的成员状态模型，还要靠 mutex 保护；
//    这里把 grid 作为 search() 的入参，整条搜索没有可变成员 —— 与
//    recovery_planner / speed_profile 的纯函数风格一致，天然线程安全，
//    也不在调用之间保留任何内存。节点缓冲是搜索内的局部 vector，
//    reserve(max_nodes) 一次性预留、结束即释放，不存在跨调用增长。
//
// 2. 地图语义适配 ROS OccupancyGrid（-1 未知 / 0..100 代价），致命判定由
//    obstacle_threshold 给出，与全局规划器同一取值。
//
// 3. 启发式权重默认 8.0（TDT 同值）。这是**加权** A*：故意放弃最优性换速度，
//    所以结果不保证最短/最快，只保证可行。文档里写这个是提醒别拿它做最优性
//    对比。全局搜索整张地图约 2 s（TDT 实测），只能用于局部段。
//
// 什么时候用它、什么时候不用：
//   用 —— 需要一条**动力学可行**的局部段的场景：给 MPC 一个从当前速度出发、
//        真能开出来的参考；脱困时验证某个安全点是否「 maneuver 得过去」。
//   不用 —— 纯几何可达性。普通 A* 便宜得多；本搜索的状态空间是位置×速度×
//        控制时长，规模大两个数量级。
//
// 输出轨迹是**分段恒加速度**的采样（每段三次多项式解析给出位置/速度/加速度），
// 不是样条。要更高质量的连续轨迹应把它交给 MINCO/后端优化，而不是直接下发。

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace navigation2::mpc
{

struct KinoConfig
{
  // 各轴速度/加速度上限（米/秒、米/秒²）。逐轴判定，与 MPC 的语义一致。
  double max_vel{2.0};
  double max_acc{4.0};
  // 单段控制的最长持续时间（秒）。搜索会把每段拆成 maxTau/steps 的几个档。
  double max_tau{2.0};
  // 速度离散分辨率（米/秒）。同一个离散 (格, vx, vy) 只保留最优节点。
  // 越小状态越多、结果越好、越慢。
  double vel_resolution{0.25};
  // 时间代价权重：cost = ∫|a|² dt + time_weight * T。
  double time_weight{1.0};
  // 启发式权重（加权 A*，>1 即放弃最优性换速度）。
  double heuristic_weight{8.0};
  // 输出轨迹的最大采样间隔（秒）。
  double sample_time{0.05};
  // 节点上限。到达即终止并返回 failure —— 内存有界，不会为了找解无限扩张。
  std::size_t max_nodes{20000};
};

// 轨迹采样点。acceleration 是该段起点处的加速度（分段恒加速）。
struct KinoSample
{
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};  // [x, y, vx, vy]
  Eigen::Vector2d acceleration{Eigen::Vector2d::Zero()};
  double time{0.0};  // 相对搜索起点的秒数
};

struct KinoResult
{
  bool success{false};  // 只有真正连到终点才为 true
  std::vector<KinoSample> trajectory;
  std::size_t nodes{0};        // 已创建的节点数（内存用量的直接指标）
  std::size_t iterations{0};   // 展开的节点数
};

// 参数合法性。非法配置会在 search() 里直接返回失败，不会产生越界。
bool kinoParamsValid(const KinoConfig & config) noexcept;

// 从 start（位置 + 速度 + 首段加速度）搜索到 goal（速度归零）的可行轨迹。
// 起点/终点在致命格上、或配置非法时返回 success=false 且轨迹为空。
KinoResult kinodynamicSearch(
  const nav_msgs::msg::OccupancyGrid & grid, const KinoConfig & config,
  const Eigen::Vector2d & start, const Eigen::Vector2d & start_vel,
  const Eigen::Vector2d & start_acc, const Eigen::Vector2d & goal);

}  // namespace navigation2::mpc

#endif  // NAVIGATION2__KINODYNAMIC_ASTAR_HPP_
