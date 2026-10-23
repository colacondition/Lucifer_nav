#include "decision/target_input.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <vector>

namespace decision
{
namespace
{

// 发布的编号 = ArmorName + 1，ArmorName 是 ARMOR_NAMES 的下标。
// 上游：tasks/auto_aim/armor.hpp 的 ARMOR_NAMES、decider.hpp:60 的注释
// "无敌状态机器人编号,英雄为1，哨兵为6"。
const char * const kPublishedIdNames[] = {
  "unknown", "hero", "engineer", "infantry3", "infantry4",
  "infantry5", "sentry", "outpost", "base"};

bool parseDouble(const std::string & text, double & out)
{
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    if (!std::isfinite(value)) {
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception &) {
    // std::stod 对空串/非数字抛异常；这里把它当"解析失败"，不是错误路径。
    return false;
  }
}

double clampAlpha(double alpha)
{
  if (!std::isfinite(alpha)) {
    return 0.0;
  }
  return std::min(1.0, std::max(0.0, alpha));
}

}  // namespace

std::optional<TargetObservation> parseTargetPayload(const std::string & payload)
{
  std::vector<std::string> fields;
  std::stringstream stream(payload);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (fields.size() != 4) {
    return std::nullopt;
  }

  double values[4] = {0.0, 0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 4; ++i) {
    if (!parseDouble(fields[i], values[i])) {
      return std::nullopt;
    }
  }

  // 上游没有目标时发的是全 0（而且照常发、不停发），所以这一位是唯一的判据。
  if (values[2] < 0.5) {
    return std::nullopt;
  }
  const int published_id = static_cast<int>(std::lround(values[3]));
  if (published_id <= 0) {
    return std::nullopt;
  }

  TargetObservation obs;
  obs.x = values[0];
  obs.y = values[1];
  obs.published_id = published_id;
  return obs;
}

std::string publishedIdName(int published_id)
{
  if (published_id <= 0 ||
    static_cast<std::size_t>(published_id) >=
    sizeof(kPublishedIdNames) / sizeof(kPublishedIdNames[0]))
  {
    return kPublishedIdNames[0];
  }
  return kPublishedIdNames[published_id];
}

TargetInput::TargetInput(TargetInputConfig config)
: config_(std::move(config))
{
  config_.filter_alpha = clampAlpha(config_.filter_alpha);
  config_.min_valid_sec = std::max(0.0, config_.min_valid_sec);
  config_.lost_timeout_sec = std::max(0.0, config_.lost_timeout_sec);
  config_.reissue_sec = std::max(0.0, config_.reissue_sec);
  config_.reissue_move_m = std::max(0.0, config_.reissue_move_m);
  config_.min_range_m = std::max(0.0, config_.min_range_m);
}

void TargetInput::update(const std::optional<TargetObservation> & obs, double now_sec)
{
  id_changed_ = false;

  if (!obs.has_value()) {
    // 这一拍没有可用目标：不清滤波状态（被遮挡几帧很常见），按超时判定。
    if (now_sec - last_valid_sec_ > config_.lost_timeout_sec) {
      valid_since_sec_ = -1.0e9;
      filtered_ = false;
      state_.usable = false;
      return;
    }
    // 还在容忍窗口内：目标位置用最后一次滤波值，可用性保持
    refreshUsability(now_sec);
    return;
  }

  if (state_.published_id != obs->published_id) {
    // 换目标了：滤波和"连续有效"都从头算，否则新目标的第一个点会被旧目标的
    // 历史拉偏，而且会立刻满足 min_valid_sec 而跳过防抖。
    id_changed_ = true;
    filtered_ = false;
    valid_since_sec_ = now_sec;
    state_.published_id = obs->published_id;
  } else if (valid_since_sec_ < -1.0e8) {
    valid_since_sec_ = now_sec;
  }

  if (!filtered_) {
    state_.x = obs->x;
    state_.y = obs->y;
    filtered_ = true;
  } else {
    const double a = config_.filter_alpha;
    state_.x = a * obs->x + (1.0 - a) * state_.x;
    state_.y = a * obs->y + (1.0 - a) * state_.y;
  }

  last_valid_sec_ = now_sec;
  state_.range_m = std::hypot(state_.x, state_.y);
  refreshUsability(now_sec);
}

void TargetInput::refreshUsability(double now_sec)
{
  const bool have_target = last_valid_sec_ > -1.0e8;
  const bool stable = have_target && (now_sec - valid_since_sec_) >= config_.min_valid_sec;
  const bool fresh = have_target && (now_sec - last_valid_sec_) <= config_.lost_timeout_sec;
  const bool far_enough = state_.range_m >= config_.min_range_m;
  state_.usable = stable && fresh && far_enough;
}

bool TargetInput::shouldReissue(double x, double y, double now_sec) const
{
  if (now_sec - last_reissue_sec_ >= config_.reissue_sec) {
    return true;
  }
  return std::hypot(x - last_reissue_x_, y - last_reissue_y_) >= config_.reissue_move_m;
}

void TargetInput::markReissued(double x, double y, double now_sec)
{
  last_reissue_sec_ = now_sec;
  last_reissue_x_ = x;
  last_reissue_y_ = y;
}

Pose standoffGoal(const Pose & enemy, const Pose & robot, double standoff_m)
{
  Pose goal = enemy;
  goal.qx = 0.0;
  goal.qy = 0.0;
  goal.qz = 0.0;
  goal.qw = 1.0;
  goal.z = 0.0;
  if (standoff_m <= 0.0) {
    return goal;
  }
  const double dx = robot.x - enemy.x;
  const double dy = robot.y - enemy.y;
  const double d = std::hypot(dx, dy);
  if (d < 1.0e-6) {
    // 方向无定义：不猜一个方向，就用敌人位置（调用方还会夹进区域）
    return goal;
  }
  goal.x = enemy.x + standoff_m * dx / d;
  goal.y = enemy.y + standoff_m * dy / d;
  return goal;
}

Pose targetToMap(const Pose & lidar_pose_in_map, double x_gimbal, double y_gimbal)
{
  // p_gimbal 的第三维用 0：上游把真 z 顶掉了，而它只通过云台俯仰影响 XY（厘米级）。
  const double px = x_gimbal;
  const double py = y_gimbal;

  // 用四元数把 (px, py, 0) 转到 map。这里展开写，避免为一个函数引入 tf2 依赖。
  const double qx = lidar_pose_in_map.qx;
  const double qy = lidar_pose_in_map.qy;
  const double qz = lidar_pose_in_map.qz;
  const double qw = lidar_pose_in_map.qw;
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1.0e-9) {
    // 四元数无效：退化成一个纯平移，总比算出 NaN 好（调用方应从 TF 拿到合法值）。
    Pose out = lidar_pose_in_map;
    out.x += px;
    out.y += py;
    return out;
  }
  const double nx = qx / norm;
  const double ny = qy / norm;
  const double nz = qz / norm;
  const double nw = qw / norm;

  // v' = v + 2 * cross(q_vec, cross(q_vec, v) + w * v)
  const double tx = 2.0 * (ny * 0.0 - nz * py);
  const double ty = 2.0 * (nz * px - nx * 0.0);
  const double tz = 2.0 * (nx * py - ny * px);

  const double rx = px + nw * tx + (ny * tz - nz * ty);
  const double ry = py + nw * ty + (nz * tx - nx * tz);

  Pose out = lidar_pose_in_map;
  out.x = lidar_pose_in_map.x + rx;
  out.y = lidar_pose_in_map.y + ry;
  // 结果是一个位置点，不该带朝向；调用方如果拿它去下发，朝向由导航决定。
  out.z = 0.0;
  out.qx = 0.0;
  out.qy = 0.0;
  out.qz = 0.0;
  out.qw = 1.0;
  return out;
}

}  // namespace decision
