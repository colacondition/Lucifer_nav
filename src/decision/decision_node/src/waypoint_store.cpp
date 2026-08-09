#include "decision/waypoint_store.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace decision
{

namespace
{

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> columns;
  std::stringstream stream(line);
  std::string column;

  while (std::getline(stream, column, ',')) {
    columns.push_back(column);
  }

  return columns;
}

std::optional<double> parseDouble(const std::string & value)
{
  try {
    std::size_t parsed = 0;
    const double number = std::stod(value, &parsed);
    if (parsed != value.size()) {
      return std::nullopt;
    }
    return number;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<Pose> parsePoseRow(const std::string & line)
{
  const auto columns = splitCsvLine(line);
  if (columns.size() < 8U) {
    return std::nullopt;
  }

  Pose pose;
  auto x = parseDouble(columns[1]);
  auto y = parseDouble(columns[2]);
  auto z = parseDouble(columns[3]);
  auto qx = parseDouble(columns[4]);
  auto qy = parseDouble(columns[5]);
  auto qz = parseDouble(columns[6]);
  auto qw = parseDouble(columns[7]);
  if (!x || !y || !z || !qx || !qy || !qz || !qw) {
    return std::nullopt;
  }

  pose.x = *x;
  pose.y = *y;
  pose.z = *z;
  pose.qx = *qx;
  pose.qy = *qy;
  pose.qz = *qz;
  pose.qw = *qw;
  return pose;
}

}  // namespace

WaypointStore::WaypointStore(DecisionConfig config)
: config_(std::move(config))
{
  waypoint_files_[TargetName::WaitHome] = config_.targets.wait_home_waypoint_file;
  waypoint_files_[TargetName::Home] = config_.targets.home_waypoint_file;
  waypoint_files_[TargetName::Patrol] = config_.targets.patrol_waypoint_file;
  waypoint_files_[TargetName::Center] = config_.targets.center_waypoint_file;
  waypoint_files_[TargetName::WaitCenter] = config_.targets.wait_center_waypoint_file;
  waypoint_files_[TargetName::WaitHp] = config_.targets.wait_hp_waypoint_file;
}

std::string WaypointStore::waypointFile(TargetName target) const
{
  const auto found = waypoint_files_.find(target);
  if (found == waypoint_files_.end()) {
    return "";
  }
  return found->second;
}

bool WaypointStore::hasWaypointFile(TargetName target) const
{
  return !waypointFile(target).empty();
}

std::optional<std::vector<Pose>> WaypointStore::loadWaypoints(TargetName target)
{
  const auto filename = waypointFile(target);
  if (filename.empty()) {
    return std::nullopt;
  }

  const auto cached_file = cached_files_.find(target);
  const auto cached_waypoints = cached_waypoints_.find(target);
  if (
    cached_file != cached_files_.end() && cached_waypoints != cached_waypoints_.end() &&
    cached_file->second == filename)
  {
    return cached_waypoints->second;
  }

  auto waypoints = readCsv(filename);
  if (!waypoints.has_value()) {
    return std::nullopt;
  }

  cached_files_[target] = filename;
  cached_waypoints_[target] = *waypoints;
  return waypoints;
}

std::optional<Pose> WaypointStore::anchorPose(TargetName target)
{
  auto waypoints = loadWaypoints(target);
  if (!waypoints.has_value() || waypoints->empty()) {
    return std::nullopt;
  }

  return waypoints->back();
}

std::optional<std::vector<Pose>> WaypointStore::prepareFollowWaypoints(
  TargetName target,
  const std::optional<Pose> & robot_pose)
{
  auto waypoints = loadWaypoints(target);
  if (!waypoints.has_value()) {
    return std::nullopt;
  }

  if (!robot_pose.has_value() || waypoints->empty()) {
    return waypoints;
  }

  const int start_index = selectForwardWaypointIndex(*waypoints, *robot_pose);
  return std::vector<Pose>(
    waypoints->begin() + start_index,
    waypoints->end());
}

std::optional<std::vector<Pose>> WaypointStore::readCsv(const std::string & filename) const
{
  std::ifstream input(filename);
  if (!input.is_open()) {
    return std::nullopt;
  }

  std::vector<Pose> waypoints;
  std::string line;
  bool header_skipped = false;
  while (std::getline(input, line)) {
    if (!header_skipped) {
      header_skipped = true;
      continue;
    }

    auto pose = parsePoseRow(line);
    if (pose.has_value()) {
      waypoints.push_back(*pose);
    }
  }

  if (waypoints.empty()) {
    return std::nullopt;
  }

  return waypoints;
}

int WaypointStore::selectForwardWaypointIndex(
  const std::vector<Pose> & waypoints,
  const Pose & robot_pose) const
{
  if (waypoints.empty()) {
    return 0;
  }

  int closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    const double current_distance = distance(robot_pose, waypoints[index]);
    if (current_distance <= config_.waypoint.switch_distance) {
      return static_cast<int>(index);
    }

    if (current_distance < closest_distance) {
      closest_distance = current_distance;
      closest_index = static_cast<int>(index);
    }
  }

  return closest_index;
}

double WaypointStore::distance(const Pose & a, const Pose & b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace decision
