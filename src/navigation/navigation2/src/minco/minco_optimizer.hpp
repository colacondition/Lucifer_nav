#ifndef NAVIGATION2__MINCO_OPTIMIZER_HPP_
#define NAVIGATION2__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>
#include "minco.hpp"
#include "trajectory.hpp"

namespace navigation2
{

class MincoOptimizer
{
public:
  struct Params
  {
    double smooth_weight{1.0};
    double obstacle_weight{10.0};
    double data_weight{10.0};        // 新增：数据项权重
    double robot_radius{1.0};
    double penalty_mu{0.4};
    // 隧道内偏离轴线的软代价权重：每段加 w * (1 - |cos θ|)，θ 是该段走向与隧道
    // 轴线的夹角。设 0 关闭。A* 出的路已经沿轴（global_planner 的同名代价），这里
    // 是防止平滑阶段在洞口切角、把本来对着洞的走向拧歪成斜切。软代价而非硬约束：
    // 车是圆柱，朝向不影响能不能过，偏轴只是该少走。
    double tunnel_axis_weight{0.0};
    bool enable{true};
  };

  MincoOptimizer();
  ~MincoOptimizer();

  void setParams(const Params & params);

  // 设置 ESDF 查询函数（适配 RcEsdfMap）
  void setEsdfQuery(
    std::function<bool(const Eigen::Vector2d &, double &, Eigen::Vector2d &)> query_fn);

  // 设置隧道轴线查询：给世界坐标，若落在隧道本体内返回 true 并写出单位轴线向量，
  // 否则返回 false。为空或恒返回 false 时对齐项自动失效（等价于关闭）。
  void setTunnelAxisQuery(
    std::function<bool(const Eigen::Vector2d &, Eigen::Vector2d &)> query_fn);

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
