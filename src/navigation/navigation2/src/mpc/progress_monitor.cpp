#include "mpc/progress_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace navigation2::mpc {

void ProgressMonitor::reset() noexcept
{
  anchor_.setZero();
  has_anchor_ = false;
  stagnant_time_ = 0.0;
  commanded_stagnant_time_ = 0.0;
  no_progress_ = false;
  stuck_ = false;
}

void ProgressMonitor::update(const Eigen::Vector2d & pos, double commanded_speed, double dt)
{
  if (!pos.allFinite()) {
    return;
  }

  const double step_dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.0;

  if (!has_anchor_) {
    anchor_ = pos;
    has_anchor_ = true;
    stagnant_time_ = 0.0;
    commanded_stagnant_time_ = 0.0;
    no_progress_ = false;
    stuck_ = false;
    return;
  }

  const double displacement = (pos - anchor_).norm();
  const double min_displacement = std::max(params_.min_displacement, 0.0);

  if (displacement >= min_displacement) {
    // 确实移动了：重设锚点，两路计时器一起清零。
    anchor_ = pos;
    stagnant_time_ = 0.0;
    commanded_stagnant_time_ = 0.0;
    no_progress_ = false;
    stuck_ = false;
    return;
  }

  // 位移不够，累计停滞时间。
  stagnant_time_ += step_dt;
  no_progress_ = stagnant_time_ >= std::max(params_.no_progress_timeout, 0.0);

  // 卡住这一路只在「正在下发指令」时累计：没指令时车不动是正常的。
  const bool commanding = commanded_speed > std::max(params_.cmd_epsilon, 0.0);
  if (commanding) {
    commanded_stagnant_time_ += step_dt;
    stuck_ = commanded_stagnant_time_ >= std::max(params_.stuck_timeout, 0.0);
  } else {
    commanded_stagnant_time_ = 0.0;
    stuck_ = false;
  }
}

}  // namespace navigation2::mpc
