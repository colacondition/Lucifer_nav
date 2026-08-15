#include "ground_segmentation/viewer.h"

#include <chrono>

using namespace std::chrono_literals;



Viewer::Viewer() {
  std::lock_guard<std::mutex> lock(viewer_mutex_);

  viewer_.setBackgroundColor (0, 0, 0);
  viewer_.addCoordinateSystem (1.0);
  viewer_.initCameraParameters ();
  viewer_.setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);

  addEmptyPointCloud("min_cloud");
  addEmptyPointCloud("ground_cloud");
  addEmptyPointCloud("obstacle_cloud");

  viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             0.0f, 1.0f, 0.0f,
                                             "min_cloud");
  viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2.0f,
                                             "min_cloud");
  viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             1.0f, 0.0f, 0.0f,
                                             "ground_cloud");

  view_thread_ = std::thread(&Viewer::drawThread, this);
}



Viewer::~Viewer() {
  // 先置 stop 再 close：drawThread 每轮只持锁 spinOnce 一次就放锁，close 总能
  // 拿到锁；stop_ 保证循环退出、join 必返回（旧实现 spin 循环持锁不撒手，
  // 析构与 visualize 都可能永久卡在抢锁上，进程关停时挂起）。
  stop_ = true;
  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    viewer_.close();
  }

  view_thread_.join();
}



void Viewer::visualize(const std::list<PointLine>& lines,
                       const PointCloud::ConstPtr& min_cloud,
                       const PointCloud::ConstPtr& ground_cloud,
                       const PointCloud::ConstPtr& obstacle_cloud) {
  redraw_ = false;
  std::lock_guard<std::mutex> lock(viewer_mutex_);
  // TODO: Do not delete and add every time.
  viewer_.removeAllShapes();
  visualizePointCloud(min_cloud, "min_cloud");
  visualizePointCloud(ground_cloud, "ground_cloud");
  visualizePointCloud(obstacle_cloud, "obstacle_cloud");
  visualizeLines(lines);
  // 数据换完，恢复绘制。
  redraw_ = true;
}



void Viewer::visualizeLines(const std::list<PointLine>& lines) {
  size_t counter = 0;
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    viewer_.addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
}



void Viewer::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                             const std::string& id) {
  viewer_.updatePointCloud(cloud, id);
}



void Viewer::addEmptyPointCloud(const std::string& id) {
  viewer_.addPointCloud(PointCloud().makeShared(), id);
}



void Viewer::drawThread() {
  while (!stop_) {
    if (redraw_) {
      // 每轮只持锁 spin 一次：visualize()/析构随时能插进来，不会死锁。
      bool stopped = false;
      {
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        viewer_.spinOnce(1);
        stopped = viewer_.wasStopped();
      }
      if (stopped) {
        break;
      }
    } else {
      std::this_thread::sleep_for(1ms);
    }
  }
}
