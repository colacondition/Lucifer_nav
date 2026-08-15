#include "distance_transform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2
{

std::vector<double> exactSquaredDistanceTransform(
  const std::vector<std::uint8_t> & seeds, const int width, const int height)
{
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  // 用有限大数而不是 inf 表示「无种子」：抛物线求交点要做减法和除法，inf 会出
  // NaN。取 1e12 使得开方后（1e6 格）远超任何真实地图尺寸，等价于无穷远。
  constexpr double kNoSeed = 1e12;
  std::vector<double> squared(cell_count, kNoSeed);

  // 一维下包络：对 f 求 d(q) = min_p (f(p) + (q - p)^2)。
  const auto transform1d = [](const std::vector<double> & f, std::vector<int> & v,
      std::vector<double> & z, std::vector<double> & out, const int n) {
      constexpr double kInf1d = std::numeric_limits<double>::infinity();
      int k = 0;
      v[0] = 0;
      z[0] = -kInf1d;
      z[1] = kInf1d;
      for (int q = 1; q < n; ++q) {
        const double fq = f[static_cast<std::size_t>(q)] + static_cast<double>(q) *
          static_cast<double>(q);
        while (true) {
          const double vk = static_cast<double>(v[static_cast<std::size_t>(k)]);
          const double fv = f[static_cast<std::size_t>(v[static_cast<std::size_t>(k)])] + vk * vk;
          const double s = (fq - fv) / (2.0 * static_cast<double>(q) - 2.0 * vk);
          if (s <= z[static_cast<std::size_t>(k)] && k > 0) {
            --k;
            continue;
          }
          ++k;
          v[static_cast<std::size_t>(k)] = q;
          z[static_cast<std::size_t>(k)] = s;
          z[static_cast<std::size_t>(k) + 1] = kInf1d;
          break;
        }
      }

      k = 0;
      for (int q = 0; q < n; ++q) {
        while (z[static_cast<std::size_t>(k) + 1] < static_cast<double>(q)) {
          ++k;
        }
        const int p = v[static_cast<std::size_t>(k)];
        const double delta = static_cast<double>(q - p);
        out[static_cast<std::size_t>(q)] = delta * delta + f[static_cast<std::size_t>(p)];
      }
    };

  const int max_dim = std::max(width, height);
  std::vector<double> f(static_cast<std::size_t>(max_dim), 0.0);
  std::vector<double> out(static_cast<std::size_t>(max_dim), 0.0);
  std::vector<int> v(static_cast<std::size_t>(max_dim), 0);
  std::vector<double> z(static_cast<std::size_t>(max_dim) + 1, 0.0);

  // 先按列做，再按行做。
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
      f[static_cast<std::size_t>(y)] = seeds[index] != 0 ? 0.0 : kNoSeed;
    }
    transform1d(f, v, z, out, height);
    for (int y = 0; y < height; ++y) {
      const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
      squared[index] = out[static_cast<std::size_t>(y)];
    }
  }
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    for (int x = 0; x < width; ++x) {
      f[static_cast<std::size_t>(x)] = squared[row + static_cast<std::size_t>(x)];
    }
    transform1d(f, v, z, out, width);
    for (int x = 0; x < width; ++x) {
      squared[row + static_cast<std::size_t>(x)] = out[static_cast<std::size_t>(x)];
    }
  }

  for (double & value : squared) {
    value = std::sqrt(std::max(0.0, value));
  }
  return squared;
}

}  // namespace navigation2
