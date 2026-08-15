#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_

#include <atomic>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Bin {
public:
  struct MinZPoint {
    MinZPoint() : z(0), d(0) {}
    MinZPoint(const double& d, const double& z) : z(z), d(d) {}
    bool operator==(const MinZPoint& comp) {return z == comp.z && d == comp.d;}

    double z;
    double d;
  };

private:

  std::atomic<bool> has_point_;
  std::atomic<double> min_z;
  std::atomic<double> min_z_range;
  // 保护「比较 z 并同时更新 (z, d)」的临界区：两个 double 分两次原子写会让
  // 并发 addPoint 交错，使最小高度点与其距离来自不同点（地面线拟合用错点）。
  mutable std::mutex update_mutex_;

public:

  Bin();

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin& segment);

  void addPoint(const pcl::PointXYZ& point);

  void addPoint(const double& d, const double& z);

  MinZPoint getMinZPoint();

  void reset();

  inline bool hasPoint() {return has_point_;}

};

#endif /* GROUND_SEGMENTATION_BIN_H_ */
