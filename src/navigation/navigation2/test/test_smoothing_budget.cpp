#include "smoothing_budget.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

namespace navigation2
{
namespace
{

using namespace std::chrono_literals;

// 默认构造 = 关闭。关闭时 expired() 恒 false、elapsedMs() 恒 0：所有预算检查
// 退化为一次 double 比较，与不接预算的旧行为完全一致。
TEST(SmoothingBudget, DefaultConstructedIsDisabled)
{
  const SmoothingBudget budget;
  EXPECT_FALSE(budget.enabled());
  EXPECT_FALSE(budget.expired());
  EXPECT_DOUBLE_EQ(budget.elapsedMs(), 0.0);
}

// <=0 的预算视为关闭（0 是参数默认值；负数属于参数写错，按关闭处理而不是
// 「立刻超时」——后者会把平滑静默停掉）。
TEST(SmoothingBudget, NonPositiveBudgetIsDisabled)
{
  for (const double ms : {0.0, -1.0, -80.0}) {
    SmoothingBudget budget(ms);
    EXPECT_FALSE(budget.enabled()) << "budget_ms=" << ms;
    budget.start();
    std::this_thread::sleep_for(2ms);
    EXPECT_FALSE(budget.expired()) << "budget_ms=" << ms;
    EXPECT_GE(budget.elapsedMs(), 0.0) << "budget_ms=" << ms;
  }
}

// NaN / Inf 也当关闭，避免参数写错时每拍立刻超时。
TEST(SmoothingBudget, NonFiniteBudgetIsDisabled)
{
  for (const double ms : {std::nan(""), std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()}) {
    const SmoothingBudget budget(ms);
    EXPECT_FALSE(budget.enabled()) << "budget_ms=" << ms;
    EXPECT_FALSE(budget.expired()) << "budget_ms=" << ms;
  }
}

// 未 start() 前 elapsedMs() 为 0，且不会因为构造后过了很久就超时。
TEST(SmoothingBudget, ElapsedIsZeroBeforeStart)
{
  SmoothingBudget budget(1.0);
  EXPECT_TRUE(budget.enabled());
  std::this_thread::sleep_for(3ms);
  EXPECT_DOUBLE_EQ(budget.elapsedMs(), 0.0);
  EXPECT_FALSE(budget.expired()) << "budget must not run before start()";
}

// 预算内不超时，elapsed 单调不减且非负（steady_clock 单调 + 负值钳零）。
TEST(SmoothingBudget, ElapsedIsMonotonicAndNonNegative)
{
  SmoothingBudget budget(1000.0);
  budget.start();

  std::vector<double> samples;
  for (int i = 0; i < 5; ++i) {
    samples.push_back(budget.elapsedMs());
    EXPECT_GE(budget.elapsedMs(), 0.0);
    EXPECT_FALSE(budget.expired());
    std::this_thread::sleep_for(1ms);
  }
  for (size_t i = 1; i < samples.size(); ++i) {
    EXPECT_GE(samples[i], samples[i - 1]) << "elapsed regressed at sample " << i;
  }
}

// 小预算过一段时间必然过期，且此时 elapsed 不小于预算。
TEST(SmoothingBudget, ExpiresAfterBudgetElapses)
{
  SmoothingBudget budget(1.0);
  ASSERT_TRUE(budget.enabled());
  budget.start();
  std::this_thread::sleep_for(10ms);
  EXPECT_TRUE(budget.expired());
  EXPECT_GE(budget.elapsedMs(), 1.0);
}

// 运行期把预算设为 0 恢复关闭；关闭后即使已经过去很久也不再报超时。
TEST(SmoothingBudget, SetBudgetZeroRestoresDisabled)
{
  SmoothingBudget budget(1.0);
  budget.start();
  std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(budget.expired());

  budget.setBudgetMs(0.0);
  EXPECT_FALSE(budget.enabled());
  EXPECT_FALSE(budget.expired());
  std::this_thread::sleep_for(2ms);
  EXPECT_FALSE(budget.expired());
}

// 重新 start() 重置计时：上一拍超时不影响下一拍。
TEST(SmoothingBudget, StartResetsElapsed)
{
  SmoothingBudget budget(1.0);
  budget.start();
  std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(budget.expired());

  budget.start();
  EXPECT_FALSE(budget.expired());
  EXPECT_LT(budget.elapsedMs(), 1000.0);
}

}  // namespace
}  // namespace navigation2
