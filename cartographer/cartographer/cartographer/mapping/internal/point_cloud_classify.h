#ifndef _POINT_CLOUD_CLASSIFY_H
#define _POINT_CLOUD_CLASSIFY_H
#include <Eigen/Core>
#include "absl/memory/memory.h"
#include "cartographer/sensor/point_cloud.h"
class ClassifyBase {
 public:
  virtual std::vector<
      std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>>
  Sengment(const cartographer::sensor::PointCloud& point_cloud) = 0;
};

class SimpleClassfiy : public ClassifyBase {
 public:
  SimpleClassfiy(double delta_theta) : delta_theta_(delta_theta) {}
  std::vector<std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>>
  Sengment(const cartographer::sensor::PointCloud& point_cloud);

 private:
  double delta_theta_;
};

class SvmClassfiy : public ClassifyBase {
 public:
  std::vector<std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>>
  Sengment(const cartographer::sensor::PointCloud& point_cloud);

 private:
};

#endif