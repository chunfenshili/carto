#include "pointcloud_process.h"

#include "Eigen/Eigen"

PointCloudProcess::PointCloudProcess(float dis) : filter_dist_(dis) {}

cartographer::sensor::PointCloud PointCloudProcess::OutlierFilter(
    const cartographer::sensor::PointCloud& point_clouds) {
  std::vector<bool> mark(point_clouds.size(), false);
  for (int j = 1; j < point_clouds.size() - 1; j++) {
    auto diff = point_clouds[j].position - point_clouds[j - 1].position;
    if (diff.norm() > filter_dist_) {
      mark[j] = false;
      mark[j - 1] = false;
    }
  }
  cartographer::sensor::PointCloud result;
  for (int i = 0; i < point_clouds.size(); i++) {
    if (mark[i]) {
      result.push_back(point_clouds[i]);
    }
  }
  return result;
}
cartographer::sensor::PointCloudWithIntensities
PointCloudProcess::OutlierFilter(
    const cartographer::sensor::PointCloudWithIntensities& point_clouds) {
  std::vector<bool> mark(point_clouds.points.size(), true);
  for (int j = 1; j < point_clouds.points.size() - 1; j++) {
    auto diff =
        point_clouds.points[j].position - point_clouds.points[j - 1].position;
    if (diff.norm() > filter_dist_) {
      mark[j] = false;
      mark[j - 1] = false;
    }
  }
  cartographer::sensor::PointCloudWithIntensities result;
  for (int i = 0; i < point_clouds.points.size(); i++) {
    if (mark[i]) {
      result.points.push_back(point_clouds.points[i]);
      result.intensities.push_back(point_clouds.intensities[i]);
    }
  }
  std::cout << "point size : " << mark.size() << " filterd point size : "
            << std::count(mark.begin(), mark.end(), false) << std::endl;
  return result;
}
