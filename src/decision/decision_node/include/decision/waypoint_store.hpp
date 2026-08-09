#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "decision/decision_config.hpp"
#include "decision/types.hpp"

namespace decision
{

class WaypointStore
{
public:
  explicit WaypointStore(DecisionConfig config);

  std::string waypointFile(TargetName target) const;
  bool hasWaypointFile(TargetName target) const;
  std::optional<std::vector<Pose>> loadWaypoints(TargetName target);
  std::optional<Pose> anchorPose(TargetName target);
  std::optional<std::vector<Pose>> prepareFollowWaypoints(
    TargetName target,
    const std::optional<Pose> & robot_pose);

private:
  std::optional<std::vector<Pose>> readCsv(const std::string & filename) const;
  int selectForwardWaypointIndex(
    const std::vector<Pose> & waypoints,
    const Pose & robot_pose) const;
  double distance(const Pose & a, const Pose & b) const;

  DecisionConfig config_;
  std::map<TargetName, std::string> waypoint_files_;
  std::map<TargetName, std::vector<Pose>> cached_waypoints_;
  std::map<TargetName, std::string> cached_files_;
};

}  // namespace decision
