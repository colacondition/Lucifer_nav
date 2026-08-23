#include "distance_transform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2
{

void exactDistanceTransform(
  const std::vector<std::uint8_t> & seeds, const int width, const int height,
  DistanceTransformWorkspace & workspace, std::vector<double> & output)
{
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  constexpr double kNoSeed = 1e12;
  output.assign(cell_count, kNoSeed);
  if (width <= 0 || height <= 0 || seeds.size() != cell_count) {
    return;
  }

  workspace.squared.assign(cell_count, kNoSeed);
  const int max_dim = std::max(width, height);
  workspace.f.resize(static_cast<std::size_t>(max_dim));
  workspace.line_out.resize(static_cast<std::size_t>(max_dim));
  workspace.envelope_indices.resize(static_cast<std::size_t>(max_dim));
  workspace.envelope_breaks.resize(static_cast<std::size_t>(max_dim) + 1);

  const auto transform1d = [](const std::vector<double> & f, std::vector<int> & v,
      std::vector<double> & z, std::vector<double> & out, const int n) {
      constexpr double kInf1d = std::numeric_limits<double>::infinity();
      int k = 0;
      v[0] = 0;
      z[0] = -kInf1d;
      z[1] = kInf1d;
      for (int q = 1; q < n; ++q) {
        const double fq = f[static_cast<std::size_t>(q)] + static_cast<double>(q) * q;
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
        while (z[static_cast<std::size_t>(k) + 1] < static_cast<double>(q)) ++k;
        const int p = v[static_cast<std::size_t>(k)];
        const double delta = static_cast<double>(q - p);
        out[static_cast<std::size_t>(q)] = delta * delta + f[static_cast<std::size_t>(p)];
      }
    };

  auto & squared = workspace.squared;
  auto & f = workspace.f;
  auto & line_out = workspace.line_out;
  auto & v = workspace.envelope_indices;
  auto & z = workspace.envelope_breaks;

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const std::size_t index = static_cast<std::size_t>(y) * width + x;
      f[static_cast<std::size_t>(y)] = seeds[index] ? 0.0 : kNoSeed;
    }
    transform1d(f, v, z, line_out, height);
    for (int y = 0; y < height; ++y) {
      squared[static_cast<std::size_t>(y) * width + x] = line_out[static_cast<std::size_t>(y)];
    }
  }
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width; ++x) f[static_cast<std::size_t>(x)] = squared[row + x];
    transform1d(f, v, z, line_out, width);
    for (int x = 0; x < width; ++x) {
      output[row + x] = std::sqrt(std::max(0.0, line_out[static_cast<std::size_t>(x)]));
    }
  }
}

std::vector<double> exactSquaredDistanceTransform(
  const std::vector<std::uint8_t> & seeds, const int width, const int height)
{
  DistanceTransformWorkspace workspace;
  std::vector<double> output;
  exactDistanceTransform(seeds, width, height, workspace, output);
  return output;
}

}  // namespace navigation2
