#ifndef NAVIGATION2__MINCO_OPTIMIZER_HPP_
#define NAVIGATION2__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <vector>
#include "minco.hpp"
#include "trajectory.hpp"

namespace navigation2
{

// 隧道横向走廊约束的几何描述。与 semantic_map_consumer.hpp 的 TunnelRegionGrid::
// CorridorFrame 字段一致，但这里独立定义 —— minco 库不依赖语义地图消费层，
// 只认这四个数。dir 是无向单位轴（隧道正进倒进等价）。
struct TunnelCorridorFrame
{
  Eigen::Vector2d centroid{0.0, 0.0};  // 本体格质心（世界系）
  Eigen::Vector2d dir{1.0, 0.0};       // 单位轴向（无向）
  double half_len{0.0};                // 纵向半长：|along| <= half_len 算洞内
  double half_width_inner{0.0};        // 横向半宽（物理），可为 0（= 压轴线）
  // 影响区横向外沿。|lat| 超过它时走廊边界随 |lat| 一起增长，惩罚平滑封顶，
  // 避免在影响区边界上出现代价跳变（L-BFGS 最怕的就是这种悬崖）。
  double lateral_outer{0.0};
};

class MincoOptimizer
{
public:
  struct Params
  {
    double smooth_weight{1.0};
    double data_weight{10.0};        // 数据项：保持接近原路径
    // 隧道内偏离轴线的软代价权重：每段加 w * (1 - |cos θ|)，θ 是该段走向与隧道
    // 轴线的夹角。设 0 关闭。A* 出的路已经沿轴（global_planner 的同名代价），这里
    // 是防止平滑阶段在洞口切角、把本来对着洞的走向拧歪成斜切。软代价而非硬约束：
    // 车是圆柱，朝向不影响能不能过，偏轴只是该少走。
    double tunnel_axis_weight{0.0};
    // 隧道横向走廊软代价：路标点横向偏移超出 clear_width/2 - robot_radius 时加
    // w * 超出量²。把「洞里别斜着切」从 A*/MINCO 的轴向软代价升级成几何边界
    // （借鉴 TDT-nav-kit 的 Item.corridor 走廊思想；无约束 L-BFGS 上只能做成
    // 带解析梯度的惩罚项，硬保证由发布前校验兜住）。设 0 关闭。
    double tunnel_corridor_weight{0.0};
    // 障碍 soft 代价：控制点处到最近障碍距离 d < safe_dist 时加
    // obstacle_weight*(safe_dist-d)²，梯度沿距离场远离障碍方向。设 0 关闭。
    // 距离场由 setDistanceQuery 注入（进程内 DistanceFieldRegistry 的查询），
    // 为空时障碍项自动失效（等价于关闭），退回纯几何平滑。
    double obstacle_weight{0.0};
    double safe_dist{0.2};
    // 精优化阶段只保留障碍梯度的轨迹法向分量，避免沿切向推拉路标点破坏时间分配。
    bool obstacle_normal_only{false};
    // L-BFGS 资源上限。history=32 时历史矩阵为 O(64*N)，远小于旧值256；迭代上限
    // 分阶段配置，避免开启两阶段后把两个4000次预算简单相加。
    int lbfgs_memory_size{32};
    int max_iterations{800};
    bool enable{true};
  };

  MincoOptimizer();
  ~MincoOptimizer();

  void setParams(const Params & params);

  // 设置隧道轴线查询：给世界坐标，若落在隧道本体内返回 true 并写出单位轴线向量，
  // 否则返回 false。为空或恒返回 false 时对齐项自动失效（等价于关闭）。
  void setTunnelAxisQuery(
    std::function<bool(const Eigen::Vector2d &, Eigen::Vector2d &)> query_fn);

  // 设置隧道横向走廊查询：给世界坐标，若落在某条隧道的可约束范围内返回 true 并
  // 写出走廊几何，否则返回 false。为空或恒返回 false 时走廊项自动失效。
  void setTunnelCorridorQuery(
    std::function<bool(const Eigen::Vector2d &, TunnelCorridorFrame &)> query_fn);

  // 设置距离场查询：给世界坐标，写出该点到最近障碍的有符号距离（米，正=障碍外）
  // 与梯度（单位向量，指向远离障碍）。返回 false 表示查询失败（地图外/未就绪）。
  // 为空时障碍项自动失效。
  void setDistanceQuery(
    std::function<bool(const Eigen::Vector2d &, double &, Eigen::Vector2d &)> query_fn);

  // 优化轨迹
  std::vector<Piece<5, 2>> optimize(
    const std::vector<Eigen::Vector2d> & waypoints,
    const std::vector<double> & segment_times);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation2

#endif  // NAVIGATION2__MINCO_OPTIMIZER_HPP_
