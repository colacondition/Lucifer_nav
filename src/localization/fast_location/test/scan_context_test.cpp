#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fast_location/frame_transforms.hpp"
#include "fast_location/scan_context.hpp"

namespace
{

using fast_location::ScanContextCandidate;
using fast_location::ScanContextConfig;
using fast_location::ScanContextDatabase;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

constexpr float kRoomHalf = 5.0f;    // 10m × 10m 房间
constexpr float kFloorZ = -0.35f;    // 地面，相对传感器
constexpr float kSampleStep = 0.2f;

float wrapAngle(float angle)
{
  while (angle > static_cast<float>(M_PI)) {
    angle -= 2.0f * static_cast<float>(M_PI);
  }
  while (angle <= -static_cast<float>(M_PI)) {
    angle += 2.0f * static_cast<float>(M_PI);
  }
  return angle;
}

float sectorAngle(const ScanContextConfig & config)
{
  return 2.0f * static_cast<float>(M_PI) / static_cast<float>(config.sectors);
}

// 合成房间：地面 + 四面墙，z 相对传感器 -0.35 ~ 2.0。
// 墙高故意做成互不相同（-x 2.0 / +x 1.0 / -y 1.6 / +y 0.6）：
// 正方形房间在 180° 旋转下严格对称，而 90° / 180° 旋转在 90 扇区下都是整数
// 列移，纯对称房间会让描述子距离在多个列移上并列，yaw 唯一性/符号就测不出来。
// 墙高不对称保证只有一个最优列移，同时仍然是"四面墙 + 地面"。
PointCloud makeRoomCloud()
{
  PointCloud cloud;
  const auto add = [&cloud](float x, float y, float z) {
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 0.0f;
    cloud.push_back(point);
  };

  // 地面
  for (float x = -kRoomHalf; x <= kRoomHalf + 1e-3f; x += kSampleStep) {
    for (float y = -kRoomHalf; y <= kRoomHalf + 1e-3f; y += kSampleStep) {
      add(x, y, kFloorZ);
    }
  }

  // 四面墙：along_y 表示墙面法向为 x（墙沿 y 方向铺开）。
  struct Wall
  {
    float x;
    float y;
    float height;
    bool along_y;
  };
  const Wall walls[] = {
    {-kRoomHalf, 0.0f, 2.0f, true},
    {kRoomHalf, 0.0f, 1.0f, true},
    {0.0f, -kRoomHalf, 1.6f, false},
    {0.0f, kRoomHalf, 0.6f, false},
  };
  for (const auto & wall : walls) {
    for (float along = -kRoomHalf; along <= kRoomHalf + 1e-3f; along += kSampleStep) {
      for (float z = kFloorZ; z <= wall.height + 1e-3f; z += kSampleStep) {
        if (wall.along_y) {
          add(wall.x, along, z);
        } else {
          add(along, wall.y, z);
        }
      }
    }
  }
  return cloud;
}

ScanContextConfig makeConfig()
{
  ScanContextConfig config;
  config.rings = 30;
  config.sectors = 90;
  config.max_radius = 30.0f;
  config.min_range = 1.0f;
  config.map_sensor_z = 0.0f;
  config.grid_step = 1.0f;
  // 合成房间本身已经是 0.2m 网格，测试里关闭降采样，保证描述子与地图逐点一致。
  config.map_voxel = 0.0f;
  config.distance_threshold = 0.6f;
  return config;
}

std::vector<Eigen::Vector2f> twoPositions()
{
  return {Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(5.0f, 0.0f)};
}

ScanContextDatabase buildTwoPositionDatabase(
  const PointCloud & room, const ScanContextConfig & config)
{
  ScanContextDatabase database;
  database.buildAtPositions(room, config, twoPositions());
  return database;
}

// 场景约定：odom 系在该测试里与 map 系重合，scan_odom 就是地图点云本身，
// 传感器位姿完全由 odom_from_base 表达（等价于"传感器在 map 的 (x, y, yaw)"）。
TEST(ScanContextTest, RecallsBuiltPositionAndZeroYaw)
{
  const auto room = makeRoomCloud();
  const auto config = makeConfig();
  auto database = buildTwoPositionDatabase(room, config);
  ASSERT_EQ(database.size(), 2u);

  const auto start_pose = fast_location::planarPoseMatrix({0.0, 0.0, 0.0});
  const auto candidates = database.query(room, start_pose, 2);
  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().position.x(), 0.0f, config.grid_step);
  EXPECT_NEAR(candidates.front().position.y(), 0.0f, config.grid_step);
  EXPECT_NEAR(wrapAngle(candidates.front().yaw_rad), 0.0f, sectorAngle(config) + 1e-4f);
  EXPECT_LE(candidates.front().distance, config.distance_threshold);
}

// yaw 符号约定：yaw_rad = -shift * 2π/sectors，输出传感器在 map 系的朝向
// （map_from_base 的 yaw）。这个测试把符号钉死：正负 90° 都必须估对，
// 如果实现里写成 +shift 这里会直接变成 -90°。
TEST(ScanContextTest, EstimatesMapFromBaseYawSignConvention)
{
  const auto room = makeRoomCloud();
  const auto config = makeConfig();
  auto database = buildTwoPositionDatabase(room, config);
  ASSERT_EQ(database.size(), 2u);

  const float yaws[] = {
    static_cast<float>(M_PI_2),
    -static_cast<float>(M_PI_2),
    static_cast<float>(M_PI_4),
  };
  const float tolerance = sectorAngle(config) + 1e-3f;

  for (const float yaw : yaws) {
    const auto pose = fast_location::planarPoseMatrix({0.0, 0.0, yaw});
    const auto candidates = database.query(room, pose, 2);
    ASSERT_FALSE(candidates.empty()) << "yaw=" << yaw;
    EXPECT_NEAR(candidates.front().position.x(), 0.0f, config.grid_step) << "yaw=" << yaw;
    EXPECT_NEAR(candidates.front().position.y(), 0.0f, config.grid_step) << "yaw=" << yaw;
    EXPECT_NEAR(wrapAngle(candidates.front().yaw_rad - yaw), 0.0f, tolerance) << "yaw=" << yaw;
  }
}

// 两个建库位置必须能区分开：在 (5, 0) 的扫描不能被召回成 (0, 0)。
TEST(ScanContextTest, DistinguishesTwoBuiltPositions)
{
  const auto room = makeRoomCloud();
  const auto config = makeConfig();
  auto database = buildTwoPositionDatabase(room, config);
  ASSERT_EQ(database.size(), 2u);

  const auto near_wall_pose = fast_location::planarPoseMatrix({5.0, 0.0, 0.0});
  const auto candidates = database.query(room, near_wall_pose, 2);
  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().position.x(), 5.0f, config.grid_step);
  EXPECT_NEAR(candidates.front().position.y(), 0.0f, config.grid_step);
  EXPECT_NEAR(wrapAngle(candidates.front().yaw_rad), 0.0f, sectorAngle(config) + 1e-4f);
}

// 按包围盒网格建库（build() 路径）：10m 房间、1m 步长 → 11×11 = 121 个位置，
// 且位置网格包含原点。
TEST(ScanContextTest, BuildsGridOverPlanarBounds)
{
  const auto room = makeRoomCloud();
  const auto config = makeConfig();
  ScanContextDatabase database;
  ASSERT_TRUE(database.build(room, config));
  EXPECT_EQ(database.size(), 121u);

  const auto candidates = database.query(
    room, fast_location::planarPoseMatrix({0.0, 0.0, 0.0}), 1);
  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().position.x(), 0.0f, config.grid_step);
  EXPECT_NEAR(candidates.front().position.y(), 0.0f, config.grid_step);
  EXPECT_NEAR(wrapAngle(candidates.front().yaw_rad), 0.0f, sectorAngle(config) + 1e-4f);
}

// save → load 往返：条目数/配置不变，查询结果逐项一致（sector key 由 image 重算）。
TEST(ScanContextTest, SaveLoadRoundTripKeepsQueryResults)
{
  const auto room = makeRoomCloud();
  const auto config = makeConfig();
  auto database = buildTwoPositionDatabase(room, config);
  ASSERT_EQ(database.size(), 2u);

  const std::string path =
    std::string(::testing::TempDir()) + "fast_location_scan_context_test.scdb";
  ASSERT_TRUE(database.save(path));

  ScanContextDatabase loaded;
  ASSERT_TRUE(loaded.load(path));
  EXPECT_EQ(loaded.size(), database.size());
  EXPECT_EQ(loaded.config().rings, config.rings);
  EXPECT_EQ(loaded.config().sectors, config.sectors);
  EXPECT_NEAR(loaded.config().max_radius, config.max_radius, 1e-6f);
  EXPECT_NEAR(loaded.config().map_sensor_z, config.map_sensor_z, 1e-6f);

  const auto pose = fast_location::planarPoseMatrix({0.0, 0.0, M_PI_2});
  const auto expected = database.query(room, pose, 2);
  const auto actual = loaded.query(room, pose, 2);
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    EXPECT_NEAR(actual[index].position.x(), expected[index].position.x(), 1e-6f);
    EXPECT_NEAR(actual[index].position.y(), expected[index].position.y(), 1e-6f);
    EXPECT_NEAR(actual[index].yaw_rad, expected[index].yaw_rad, 1e-6f);
    EXPECT_NEAR(actual[index].distance, expected[index].distance, 1e-6f);
  }

  // 不存在的文件必须失败，且不破坏已有数据库内容。
  ScanContextDatabase missing;
  EXPECT_FALSE(missing.load(path + ".does_not_exist"));
  EXPECT_TRUE(missing.empty());

  std::remove(path.c_str());
}

// 边界：空地图/空扫描/非法位姿/top_k<=0/非法配置都必须安全返回空，不崩不挂。
TEST(ScanContextTest, EmptyAndInvalidInputsReturnEmpty)
{
  const auto config = makeConfig();
  const auto room = makeRoomCloud();

  PointCloud empty_cloud;
  ScanContextDatabase database;
  EXPECT_FALSE(database.build(empty_cloud, config));
  EXPECT_TRUE(database.empty());
  EXPECT_EQ(database.size(), 0u);
  EXPECT_TRUE(database.query(empty_cloud, Eigen::Matrix4f::Identity(), 5).empty());

  ASSERT_TRUE(database.buildAtPositions(room, config, twoPositions()));
  ASSERT_FALSE(database.empty());
  EXPECT_TRUE(database.query(empty_cloud, Eigen::Matrix4f::Identity(), 5).empty());
  EXPECT_TRUE(database.query(room, Eigen::Matrix4f::Identity(), 0).empty());

  Eigen::Matrix4f non_finite_pose = Eigen::Matrix4f::Identity();
  non_finite_pose(0, 0) = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(database.query(room, non_finite_pose, 5).empty());

  // 非法配置拒绝建库（rings=0 会在描述子里除零）。
  ScanContextConfig invalid = config;
  invalid.rings = 0;
  ScanContextDatabase invalid_database;
  EXPECT_FALSE(invalid_database.buildAtPositions(room, invalid, twoPositions()));
  EXPECT_TRUE(invalid_database.empty());
}

}  // namespace
