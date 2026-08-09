#ifndef NAVIGATION2__RC_ESDF_HPP_
#define NAVIGATION2__RC_ESDF_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Core>

class RcEsdfMap
{
public:
  RcEsdfMap() = default;

  // 先定栅格尺寸。
  void initialize(double width_m, double height_m, double resolution);

  // 用机器人外形生成距离场。
  void generateFromPolygon(const std::vector<Eigen::Vector2d> & polygon);

  // 查询车体系下的距离和梯度。
  bool query(const Eigen::Vector2d & pos_body, double & dist, Eigen::Vector2d & grad) const;

private:
  inline void posToGrid(const Eigen::Vector2d & pos, double & gx, double & gy) const
  {
    gx = (pos.x() - origin_x_) / resolution_;
    gy = (pos.y() - origin_y_) / resolution_;
  }

  inline float getRaw(int x, int y) const
  {
    if (x < 0 || x >= grid_size_x_ || y < 0 || y >= grid_size_y_) {
      return 0.0f;
    }
    return data_[y * grid_size_x_ + x];
  }

  double pointToSegmentDistSq(
    const Eigen::Vector2d & p,
    const Eigen::Vector2d & v,
    const Eigen::Vector2d & w);

  bool isPointInPolygon(
    const Eigen::Vector2d & p,
    const std::vector<Eigen::Vector2d> & poly);

  double resolution_{0.05};
  double width_m_{1.0}, height_m_{1.0};
  double origin_x_{0.0}, origin_y_{0.0};
  int grid_size_x_{0}, grid_size_y_{0};
  std::vector<float> data_;
};

#endif  // NAVIGATION2__RC_ESDF_HPP_
