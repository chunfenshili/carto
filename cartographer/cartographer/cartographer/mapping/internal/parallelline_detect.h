#ifndef PARALLELLINE_DETECT_H
#define  PARALLELLINE_DETECT_H
#include "point_cloud_classify.h"
#include "cartographer/sensor/point_cloud.h"
#include <memory>
#include <vector>
struct ParallelParam {
  double a;
  double b;
  double c1;
  double c2;
};
std::ostream& operator<<(std::ostream& out, const ParallelParam& param);
class ParallelLIneFillting {
 public:
  ParallelLIneFillting(
      const std::vector<cartographer::sensor::PointCloud>& sengment)
      : sengment_(sengment) {}
  ParallelParam Fit();
  double Score(const ParallelParam& paraller);

 private:
  double thresh_hold_;
  const std::vector<cartographer::sensor::PointCloud>& sengment_;
};

struct ParallelLineResult {
  double score;
  std::shared_ptr<std::vector<cartographer::sensor::PointCloud>> point_cloud;
  ParallelParam param;
};
class ParallelLineDetect {
 public:
  ParallelLineDetect(const double& min_param_score)
      : paraller_score(min_param_score) {
    classify_ = std::make_unique<SimpleClassfiy>(1);
  }
 std::vector<ParallelLineResult> Detect(
      const cartographer::sensor::PointCloud& point_cloud);

 private:
  const double paraller_score = 0.90;
  double Score(
      const std::vector<cartographer::sensor::PointCloud>& point_clouds);
  std::unique_ptr<ClassifyBase> classify_;
  std::unique_ptr<ParallelLIneFillting> paraller_fit_;
  
  
};

#endif