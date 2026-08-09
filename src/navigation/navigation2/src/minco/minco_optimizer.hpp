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
    bool enable{true};
  };

  MincoOptimizer();
  ~MincoOptimizer();

  void setParams(const Params & params);

  // 设置 ESDF 查询函数（适配 RcEsdfMap）
  void setEsdfQuery(
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
