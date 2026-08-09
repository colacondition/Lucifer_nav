#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "decision/waypoint_store.hpp"

namespace
{

std::filesystem::path writeCsv(const std::string & name, const std::string & body)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream out(path);
  out << "name,x,y,z,qx,qy,qz,qw\n";
  out << body;
  return path;
}

decision::DecisionConfig configWithCenter(const std::filesystem::path & center_file)
{
  decision::DecisionConfig cfg;
  cfg.targets.center_waypoint_file = center_file.string();
  cfg.waypoint.switch_distance = 0.6;
  cfg.waypoint.final_goal_tolerance = 0.35;
  return cfg;
}

}  // namespace

TEST(WaypointStore, LoadsValidCsv)
{
  const auto csv = writeCsv(
    "decision_waypoints_valid.csv",
    "p0,1.0,2.0,0.0,0.0,0.0,0.0,1.0\n"
    "p1,3.0,4.0,0.0,0.0,0.0,0.0,1.0\n");
  decision::WaypointStore store(configWithCenter(csv));

  auto waypoints = store.loadWaypoints(decision::TargetName::Center);

  ASSERT_TRUE(waypoints.has_value());
  ASSERT_EQ(waypoints->size(), 2U);
  EXPECT_DOUBLE_EQ((*waypoints)[0].x, 1.0);
  EXPECT_DOUBLE_EQ((*waypoints)[1].y, 4.0);
}

TEST(WaypointStore, LoadsWaitHpCsv)
{
  const auto csv = writeCsv(
    "decision_wait_hp.csv",
    "p0,-0.8,-0.4,0.0,0.0,0.0,0.0,1.0\n");
  decision::DecisionConfig cfg;
  cfg.targets.wait_hp_waypoint_file = csv.string();
  decision::WaypointStore store(cfg);

  const auto waypoints = store.loadWaypoints(decision::TargetName::WaitHp);

  ASSERT_TRUE(waypoints.has_value());
  ASSERT_EQ(waypoints->size(), 1U);
  EXPECT_DOUBLE_EQ(waypoints->front().x, -0.8);
}

TEST(WaypointStore, LoadsRelativeCsvFromWorkingDirectory)
{
  const auto original_path = std::filesystem::current_path();
  const auto test_root = std::filesystem::temp_directory_path() / "decision_relative_waypoints";
  const auto waypoint_dir = test_root / "src/HL_bringup/config/waypoints/RMUL";
  std::filesystem::create_directories(waypoint_dir);
  const auto csv = waypoint_dir / "center.csv";
  {
    std::ofstream out(csv);
    out << "name,x,y,z,qx,qy,qz,qw\n";
    out << "p0,5.0,6.0,0.0,0.0,0.0,0.0,1.0\n";
  }

  std::filesystem::current_path(test_root);
  decision::WaypointStore store(configWithCenter("src/HL_bringup/config/waypoints/RMUL/center.csv"));

  auto waypoints = store.loadWaypoints(decision::TargetName::Center);
  std::filesystem::current_path(original_path);

  ASSERT_TRUE(waypoints.has_value());
  ASSERT_EQ(waypoints->size(), 1U);
  EXPECT_DOUBLE_EQ((*waypoints)[0].x, 5.0);
  EXPECT_DOUBLE_EQ((*waypoints)[0].y, 6.0);
}

TEST(WaypointStore, RejectsMissingCsv)
{
  decision::WaypointStore store(configWithCenter("/tmp/file_that_does_not_exist.csv"));

  EXPECT_FALSE(store.loadWaypoints(decision::TargetName::Center).has_value());
}

TEST(WaypointStore, AnchorPoseReturnsFinalWaypoint)
{
  const auto csv = writeCsv(
    "decision_waypoints_anchor.csv",
    "p0,1.0,2.0,0.0,0.0,0.0,0.0,1.0\n"
    "p1,3.0,4.0,0.0,0.0,0.0,0.0,1.0\n");
  decision::WaypointStore store(configWithCenter(csv));

  auto anchor = store.anchorPose(decision::TargetName::Center);

  ASSERT_TRUE(anchor.has_value());
  EXPECT_DOUBLE_EQ(anchor->x, 3.0);
  EXPECT_DOUBLE_EQ(anchor->y, 4.0);
}

TEST(WaypointStore, PrepareFollowWaypointsStartsNearForwardWaypoint)
{
  const auto csv = writeCsv(
    "decision_waypoints_forward.csv",
    "p0,0.0,0.0,0.0,0.0,0.0,0.0,1.0\n"
    "p1,1.0,0.0,0.0,0.0,0.0,0.0,1.0\n"
    "p2,2.0,0.0,0.0,0.0,0.0,0.0,1.0\n"
    "p3,3.0,0.0,0.0,0.0,0.0,0.0,1.0\n");
  auto cfg = configWithCenter(csv);
  cfg.waypoint.switch_distance = 0.6;
  decision::WaypointStore store(cfg);

  decision::Pose robot;
  robot.x = 1.2;
  robot.y = 0.0;
  auto prepared = store.prepareFollowWaypoints(decision::TargetName::Center, robot);

  ASSERT_TRUE(prepared.has_value());
  ASSERT_FALSE(prepared->empty());
  EXPECT_DOUBLE_EQ(prepared->front().x, 1.0);
}
