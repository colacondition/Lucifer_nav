#include "ground_segmentation/segment.h"

#include <cmath>

Segment::Segment(const unsigned int& n_bins,
                 const double& min_slope,
                 const double& max_slope,
                 const double& max_error,
                 const double& long_threshold,
                 const double& max_long_height,
                 const double& max_start_height,
                 const double& sensor_height) :
                 bins_(n_bins),
                 min_slope_(min_slope),
                 max_slope_(max_slope),
                 max_error_(max_error),
                 long_threshold_(long_threshold),
                 max_long_height_(max_long_height),
                 max_start_height_(max_start_height),
                 sensor_height_(sensor_height){}

void Segment::reset() {
  lines_.clear();
  for (auto &bin : bins_) {
    bin.reset();
  }
}

void Segment::fitSegmentLines() {
  // Find first point.
  auto line_start = bins_.begin();
  while (!line_start->hasPoint()) {
    ++line_start;
    // Stop if we reached last point.
    if (line_start == bins_.end()) return;
  }
  // Fill lines.
  bool is_long_line = false;
  double cur_ground_height = -sensor_height_;
  std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());
  LocalLine cur_line = std::make_pair(0,0);
  for (auto line_iter = line_start+1; line_iter != bins_.end(); ++line_iter) {
    if (line_iter->hasPoint()) {
      Bin::MinZPoint cur_point = line_iter->getMinZPoint();
      if (cur_point.d - current_line_points.back().d > long_threshold_) is_long_line = true;
      if (current_line_points.size() >= 2) {
        // Get expected z value to possibly reject far away points.
        double expected_z = std::numeric_limits<double>::max();
        if (is_long_line && current_line_points.size() > 2) {
          expected_z = cur_line.first * cur_point.d + cur_line.second;
        }
        current_line_points.push_back(cur_point);
        cur_line = fitLocalLine(current_line_points);
        const double error = getMaxError(current_line_points, cur_line);
        // Check if not a good line.
        if (error > max_error_ ||
            std::fabs(cur_line.first) > max_slope_ ||
            (current_line_points.size() > 2 && std::fabs(cur_line.first) < min_slope_) ||
            is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_) {
          // Add line until previous point as ground.
          current_line_points.pop_back();
          // Don't let lines with 2 base points through.
          if (current_line_points.size() >= 3) {
            const LocalLine new_line = fitLocalLine(current_line_points);
            lines_.push_back(localLineToLine(new_line, current_line_points));
            cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
          }
          // Start new line.
          is_long_line = false;
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());
          --line_iter;
        }
        // Good line, continue.
        else { }
      }
      else {
        // Not enough points.
        if (cur_point.d - current_line_points.back().d < long_threshold_ &&
            std::fabs(current_line_points.back().z - cur_ground_height) < max_start_height_) {
          // Add point if valid.
          current_line_points.push_back(cur_point);
        }
        else {
          // Start new line.
          current_line_points.clear();
          current_line_points.push_back(cur_point);
        }
      }
    }
  }
  // Add last line.
  if (current_line_points.size() > 2) {
    const LocalLine new_line = fitLocalLine(current_line_points);
    lines_.push_back(localLineToLine(new_line, current_line_points));
  }
}

Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const std::list<Bin::MinZPoint>& line_points) {
  Line line;
  const double first_d = line_points.front().d;
  const double second_d = line_points.back().d;
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;
  line.first.z = first_z;
  line.first.d = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}

double Segment::verticalDistanceToLine(const double &d, const double &z) {
  static const double kMargin = 0.1;
  double distance = -1;
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
      const double delta_z = it->second.z - it->first.z;
      const double delta_d = it->second.d - it->first.d;
      const double expected_z = (d - it->first.d)/delta_d *delta_z + it->first.z;
      distance = std::fabs(z - expected_z);
    }
  }
  return distance;
}

double Segment::getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double error_sum = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    error_sum += residual * residual;
  }
  return error_sum/points.size();
}

double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double max_error = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    const double error = residual * residual;
    if (error > max_error) max_error = error;
  }
  return max_error;
}

Segment::LocalLine Segment::fitLocalLine(const std::list<Bin::MinZPoint> &points) {
  // 闭式最小二乘 z = a*d + b（2×2 正规方程手解）。每条线只有 3~6 个点，但每帧
  // 要拟合上百次：旧的 MatrixXd 动态分配 + colPivHouseholderQr 是纯浪费。
  double sum_d = 0.0;
  double sum_z = 0.0;
  double sum_d2 = 0.0;
  double sum_dz = 0.0;
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    sum_d += iter->d;
    sum_z += iter->z;
    sum_d2 += iter->d * iter->d;
    sum_dz += iter->d * iter->z;
  }
  const double n = static_cast<double>(points.size());
  const double denominator = n * sum_d2 - sum_d * sum_d;
  LocalLine line_result;
  if (std::abs(denominator) < 1e-12) {
    // 所有点的 d 几乎相同（竖直排列）：取 z 均值、斜率 0，与 QR 的病态行为一致。
    line_result.first = 0.0;
    line_result.second = sum_z / n;
  } else {
    line_result.first = (n * sum_dz - sum_d * sum_z) / denominator;
    line_result.second = (sum_z - line_result.first * sum_d) / n;
  }
  return line_result;
}

bool Segment::getLines(std::list<Line> *lines) {
  if (lines_.empty()) {
    return false;
  }
  else {
    *lines = lines_;
    return true;
  }
}
