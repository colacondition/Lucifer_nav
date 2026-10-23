// 自瞄目标输入的判据：解析、防抖、迟滞、滤波、id 切换，以及换算到 map。
//
// 这些判据飘了之后的现场表现是「哨兵朝一个不存在的方向冲」或者「目标明显在却不动」，
// 都是没有一行报错的故障。尤其上游是**单帧未滤波**的 PnP，防抖不做就会被误检带跑。

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "decision/target_input.hpp"

namespace
{

constexpr double kEps = 1e-9;

decision::Pose pt(double x, double y)
{
  decision::Pose p;
  p.x = x;
  p.y = y;
  return p;
}

decision::TargetObservation obs(double x, double y, int id)
{
  decision::TargetObservation o;
  o.x = x;
  o.y = y;
  o.published_id = id;
  return o;
}

// 朝向为单位四元数、位置 (2.0, 3.0) 的"雷达/云台"位姿
decision::Pose lidarPose(double x, double y, double qz = 0.0, double qw = 1.0)
{
  decision::Pose p;
  p.x = x;
  p.y = y;
  p.qz = qz;
  p.qw = qw;
  return p;
}

// ==================== 解析 ====================

TEST(ParseTargetPayload, ReadsTheCsvTriple)
{
  // 上游用 std::to_string 拼的，所以是 6 位小数的固定格式
  const auto parsed = decision::parseTargetPayload("1.234567,-0.500000,1.000000,6.000000");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_NEAR(parsed->x, 1.234567, kEps);
  EXPECT_NEAR(parsed->y, -0.5, kEps);
  EXPECT_EQ(parsed->published_id, 6);
}

TEST(ParseTargetPayload, AllZeroMeansNoTarget)
{
  // ⚠️ 上游**没有目标时照常发、只是全 0**，所以不能靠"没收到消息"判断丢失
  EXPECT_FALSE(decision::parseTargetPayload("0.000000,0.000000,0.000000,0.000000")
    .has_value());
}

TEST(ParseTargetPayload, ZeroValidityMeansNoTarget)
{
  // valid 位是 0 但坐标非零：按"没有目标"处理，不要采信坐标
  EXPECT_FALSE(decision::parseTargetPayload("3.000000,1.000000,0.000000,6.000000")
    .has_value());
}

TEST(ParseTargetPayload, RejectsMalformedInput)
{
  EXPECT_FALSE(decision::parseTargetPayload("").has_value());
  EXPECT_FALSE(decision::parseTargetPayload("1.0,2.0,1.0").has_value());        // 少一个
  EXPECT_FALSE(decision::parseTargetPayload("1.0,2.0,1.0,6.0,7.0").has_value());  // 多一个
  EXPECT_FALSE(decision::parseTargetPayload("a,2.0,1.0,6.0").has_value());       // 不是数字
  EXPECT_FALSE(decision::parseTargetPayload("1.0,2.0,1.0,nan").has_value());     // 非有限
  EXPECT_FALSE(decision::parseTargetPayload("1.0,2.0,1.0,").has_value());        // 空字段
}

TEST(PublishedIdName, MapsArmorNamePlusOneAndRejectsTheRest)
{
  // ⚠️ 这套编号是 ArmorName+1，**不是**裁判协议的 robot_id（哨兵在那边是 7，这里是 6）
  EXPECT_EQ(decision::publishedIdName(1), "hero");
  EXPECT_EQ(decision::publishedIdName(6), "sentry");
  EXPECT_EQ(decision::publishedIdName(3), "infantry3");
  EXPECT_EQ(decision::publishedIdName(0), "unknown");
  EXPECT_EQ(decision::publishedIdName(-2), "unknown");
  EXPECT_EQ(decision::publishedIdName(99), "unknown");
}

// ==================== 防抖与迟滞 ====================

TEST(TargetInput, RequiresTheTargetToStayValidBeforeUsingIt)
{
  decision::TargetInputConfig config;
  config.min_valid_sec = 0.5;
  decision::TargetInput input(config);

  // 第一帧有效：还不能用（上游是单帧 PnP，一帧就用会被误检带跑）
  input.update(obs(2.0, 0.0, 6), 0.0);
  EXPECT_FALSE(input.state().usable);

  // 还没到 min_valid_sec
  input.update(obs(2.0, 0.0, 6), 0.3);
  EXPECT_FALSE(input.state().usable);

  // 到点了
  input.update(obs(2.0, 0.0, 6), 0.6);
  EXPECT_TRUE(input.state().usable);
  EXPECT_EQ(input.state().published_id, 6);
}

TEST(TargetInput, KeepsTheTargetThroughBriefDropouts)
{
  decision::TargetInputConfig config;
  config.min_valid_sec = 0.5;
  config.lost_timeout_sec = 1.5;
  decision::TargetInput input(config);

  input.update(obs(2.0, 0.0, 6), 0.0);
  input.update(obs(2.0, 0.0, 6), 0.6);
  ASSERT_TRUE(input.state().usable);

  // 短暂丢几帧（遮挡/换弹常见）不该放弃目标
  input.update(std::nullopt, 0.8);
  EXPECT_TRUE(input.state().usable);
  input.update(std::nullopt, 1.0);
  EXPECT_TRUE(input.state().usable);

  // 超过 lost_timeout 才放弃
  input.update(std::nullopt, 2.2);
  EXPECT_FALSE(input.state().usable);
}

TEST(TargetInput, ReacquiringAfterTimeoutNeedsTheFullDebounceAgain)
{
  decision::TargetInputConfig config;
  config.min_valid_sec = 0.5;
  config.lost_timeout_sec = 1.0;
  decision::TargetInput input(config);

  input.update(obs(2.0, 0.0, 6), 0.0);
  input.update(obs(2.0, 0.0, 6), 0.6);
  ASSERT_TRUE(input.state().usable);

  input.update(std::nullopt, 2.5);          // 丢太久
  ASSERT_FALSE(input.state().usable);

  // 重新出现：不能立刻可用，要重新攒够 min_valid_sec
  input.update(obs(2.0, 0.0, 6), 2.6);
  EXPECT_FALSE(input.state().usable);
  input.update(obs(2.0, 0.0, 6), 3.2);
  EXPECT_TRUE(input.state().usable);
}

TEST(TargetInput, IdSwitchResetsFilterAndDebounce)
{
  decision::TargetInputConfig config;
  config.min_valid_sec = 0.5;
  decision::TargetInput input(config);

  // 先让 hero(id=1) 稳定可用，滤波也收敛到 (2, 0)
  for (int i = 0; i <= 10; ++i) {
    input.update(obs(2.0, 0.0, 1), i * 0.1);
  }
  ASSERT_TRUE(input.state().usable);
  ASSERT_EQ(input.state().published_id, 1);

  // 突然换成哨兵(id=6)、位置也完全变了
  input.update(obs(-3.0, 1.0, 6), 1.1);
  EXPECT_TRUE(input.idChanged());
  // 换目标后必须重新防抖，而且滤波重置到新目标上（不能被旧目标的历史拉偏）
  EXPECT_FALSE(input.state().usable);
  EXPECT_NEAR(input.state().x, -3.0, kEps);
  EXPECT_NEAR(input.state().y, 1.0, kEps);

  input.update(obs(-3.0, 1.0, 6), 1.7);
  EXPECT_TRUE(input.state().usable);
  // idChanged 是"本拍事件"，下一拍该清零
  EXPECT_FALSE(input.idChanged());
}

TEST(TargetInput, FiltersStepChangesTowardsTheNewValue)
{
  decision::TargetInputConfig config;
  config.min_valid_sec = 0.0;
  config.filter_alpha = 0.5;
  decision::TargetInput input(config);

  input.update(obs(0.0, 0.0, 6), 0.0);
  input.update(obs(4.0, 0.0, 6), 0.1);
  // 一阶滤波：0.5*4 + 0.5*0 = 2
  EXPECT_NEAR(input.state().x, 2.0, kEps);
  input.update(obs(4.0, 0.0, 6), 0.2);
  EXPECT_NEAR(input.state().x, 3.0, kEps);
}

TEST(TargetInput, RejectsTargetsThatAreTooClose)
{
  decision::TargetInputConfig config;
  config.min_valid_sec = 0.0;
  config.min_range_m = 0.8;
  // 关掉滤波（alpha=1），让 range 断言只反映距离判据，不受滤波系数影响
  config.filter_alpha = 1.0;
  decision::TargetInput input(config);

  // 0.5m：太近，下发会让车原地抖
  input.update(obs(0.5, 0.0, 6), 0.0);
  EXPECT_FALSE(input.state().usable);
  EXPECT_NEAR(input.state().range_m, 0.5, kEps);

  input.update(obs(2.0, 0.0, 6), 0.1);
  EXPECT_TRUE(input.state().usable);
  EXPECT_NEAR(input.state().range_m, 2.0, kEps);
}

// ==================== 重发判据 ====================

TEST(TargetInput, ReissuesOnFirstUseThenOnlyOnTimeOrMovement)
{
  decision::TargetInputConfig config;
  config.reissue_sec = 0.5;
  config.reissue_move_m = 0.4;
  decision::TargetInput input(config);

  // 从没下发过 → 第一条就该发
  EXPECT_TRUE(input.shouldReissue(2.0, 3.0, 10.0));
  input.markReissued(2.0, 3.0, 10.0);

  // 时间没到、位置没动 → 不重发（否则 30Hz 会把规划器打爆）
  EXPECT_FALSE(input.shouldReissue(2.0, 3.0, 10.2));
  EXPECT_FALSE(input.shouldReissue(2.1, 3.1, 10.2));

  // 目标移动够多 → 立刻重发
  EXPECT_TRUE(input.shouldReissue(2.5, 3.0, 10.2));

  // 时间到 → 重发
  EXPECT_TRUE(input.shouldReissue(2.0, 3.0, 10.6));
}

// ==================== 换算到 map ====================

// ==================== 保持距离的圆 ====================

TEST(StandoffGoal, BacksOffAlongTheLineToTheRobot)
{
  // 机器人在敌人正东 4m、要求保持 1.5m ⇒ 目标点在敌人正东 1.5m 处
  const auto enemy = pt(0.0, 0.0);
  const auto robot = pt(4.0, 0.0);
  const auto goal = decision::standoffGoal(enemy, robot, 1.5);
  EXPECT_NEAR(goal.x, 1.5, 1e-9);
  EXPECT_NEAR(goal.y, 0.0, 1e-9);
}

TEST(StandoffGoal, AlsoRetreatsWhenTooClose)
{
  // 机器人在敌人正东 0.5m（太近）⇒ 同一个式子把它推回 1.5m 处（即往西退）
  const auto enemy = pt(0.0, 0.0);
  const auto robot = pt(0.5, 0.0);
  const auto goal = decision::standoffGoal(enemy, robot, 1.5);
  EXPECT_NEAR(goal.x, 1.5, 1e-9);
  EXPECT_NEAR(goal.y, 0.0, 1e-9);
}

TEST(StandoffGoal, PreservesDirectionAnywhereOnThePlane)
{
  // 斜方向：敌人 (1,1)，机器人在 (4,5) ⇒ 方向 (3,4)/5，目标点 = 敌人 + 1.5*(0.6,0.8)
  const auto goal = decision::standoffGoal(pt(1.0, 1.0), pt(4.0, 5.0), 1.5);
  EXPECT_NEAR(goal.x, 1.0 + 0.9, 1e-9);
  EXPECT_NEAR(goal.y, 1.0 + 1.2, 1e-9);
}

TEST(StandoffGoal, DegenerateAndDisabledCases)
{
  // 与敌人重合：方向无定义，返回敌人位置本身（不去猜一个方向）
  const auto same = decision::standoffGoal(pt(2.0, 3.0), pt(2.0, 3.0), 1.5);
  EXPECT_NEAR(same.x, 2.0, 1e-9);
  EXPECT_NEAR(same.y, 3.0, 1e-9);
  // standoff <= 0：等于"直接以敌人为目标"
  const auto zero = decision::standoffGoal(pt(2.0, 3.0), pt(9.0, 9.0), 0.0);
  EXPECT_NEAR(zero.x, 2.0, 1e-9);
  EXPECT_NEAR(zero.y, 3.0, 1e-9);
}

TEST(StandoffGoal, OutputIsAPositionWithoutOrientation)
{
  const auto goal = decision::standoffGoal(pt(0.0, 0.0), pt(4.0, 0.0), 1.5);
  EXPECT_DOUBLE_EQ(goal.qx, 0.0);
  EXPECT_DOUBLE_EQ(goal.qz, 0.0);
  EXPECT_DOUBLE_EQ(goal.qw, 1.0);
}

TEST(TargetToMap, IdentityRotationJustTranslates)
{
  // 雷达/云台在 (2.0, 3.0) 且朝向与 map 一致：目标 (1.5, -0.5) 应该落在 (3.5, 2.5)
  const auto p = decision::targetToMap(lidarPose(2.0, 3.0), 1.5, -0.5);
  EXPECT_NEAR(p.x, 3.5, kEps);
  EXPECT_NEAR(p.y, 2.5, kEps);
}

TEST(TargetToMap, RotatesWithTheGimbalYaw)
{
  // 云台相对 map 转 +90°：云台系的前方(+x)对应 map 的 +y
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto p = decision::targetToMap(lidarPose(0.0, 0.0, s, c), 1.0, 0.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 1.0, 1e-9);

  // 转 -90°：云台前方对应 map 的 -y
  const auto q = decision::targetToMap(lidarPose(0.0, 0.0, -s, c), 1.0, 0.0);
  EXPECT_NEAR(q.x, 0.0, 1e-9);
  EXPECT_NEAR(q.y, -1.0, 1e-9);
}

TEST(TargetToMap, OutputIsAPositionWithoutOrientation)
{
  // 结果是"一个点"，朝向由导航决定 —— 不能把云台的朝向带出去
  const auto p = decision::targetToMap(lidarPose(2.0, 3.0, 0.5, 0.5), 1.0, 0.0);
  EXPECT_DOUBLE_EQ(p.qx, 0.0);
  EXPECT_DOUBLE_EQ(p.qy, 0.0);
  EXPECT_DOUBLE_EQ(p.qz, 0.0);
  EXPECT_DOUBLE_EQ(p.qw, 1.0);
  EXPECT_DOUBLE_EQ(p.z, 0.0);
}

TEST(TargetToMap, SurvivesAnUnnormalisedQuaternion)
{
  // TF 一般给单位四元数，但别假设它是 —— 不归一化会让旋转量偏大
  const auto unit = decision::targetToMap(lidarPose(0.0, 0.0, 0.7071067811865476,
      0.7071067811865476), 1.0, 0.0);
  const auto scaled = decision::targetToMap(lidarPose(0.0, 0.0, 7.071067811865476,
      7.071067811865476), 1.0, 0.0);
  EXPECT_NEAR(unit.x, scaled.x, 1e-9);
  EXPECT_NEAR(unit.y, scaled.y, 1e-9);
}

TEST(TargetToMap, DegenerateQuaternionFallsBackToTranslation)
{
  // 四元数全 0（TF 没填）时不能算出 NaN —— 退化成纯平移，让调用方能看出"方向没生效"
  const auto p = decision::targetToMap(lidarPose(1.0, 2.0, 0.0, 0.0), 0.5, 0.5);
  EXPECT_TRUE(std::isfinite(p.x));
  EXPECT_TRUE(std::isfinite(p.y));
  EXPECT_NEAR(p.x, 1.5, kEps);
  EXPECT_NEAR(p.y, 2.5, kEps);
}

}  // namespace
