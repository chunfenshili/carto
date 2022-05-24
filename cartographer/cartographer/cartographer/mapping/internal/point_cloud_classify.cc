#include "point_cloud_classify.h"

#include "cartographer/sensor/point_cloud.h"

namespace {
using namespace cartographer;

constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }
//
Eigen::Vector3f ComputeCentroid(const sensor::PointCloud& point_cloud) {
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const sensor::RangefinderPoint& point : point_cloud) {
    sum += point.position;
  }
  return sum / static_cast<float>(point_cloud.size());
}
void MinusCetroid(const sensor::PointCloud& point_cloud,
                  const Eigen::Vector3f& centroid, sensor::PointCloud& out) {
  for (const auto& point : point_cloud) {
    out.push_back({point.position - centroid});
  }
}
}  // namespace

std::vector<std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>>
SimpleClassfiy::Sengment(const cartographer::sensor::PointCloud& point_cloud) {
  cartographer::sensor::PointCloud over_point_cloud;
  Eigen::Vector3f centroid = ComputeCentroid(point_cloud);
  MinusCetroid(point_cloud, centroid, over_point_cloud);
  centroid = ComputeCentroid(over_point_cloud);
  //
  int num_line = static_cast<int>(std::round(360 / delta_theta_))/2;
  std::vector<std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>>
      result(num_line,
             std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>(
                 nullptr));
  //
  for (int i = 0; i < num_line; i++) {
    double a = tan(DegToRad(i * delta_theta_));
    double b = centroid.y() - a * centroid.x();

    std::shared_ptr<std::vector<cartographer::sensor::PointCloud>> two_segment =
        std::make_shared<std::vector<cartographer::sensor::PointCloud>>(2);
    for (const auto& point : over_point_cloud) {
      double z = point.position.x() * a + b - point.position.y();
      if (z > 0) {
        two_segment->front().push_back(point);
        // LOG(INFO)<<point.position;
      } else {
        // LOG(INFO)<<point.position;
        two_segment->back().push_back(point);
      }
    }
    result[i] = std::move(two_segment);
  }
  return result;
}

//
std::vector<std::shared_ptr<std::vector<cartographer::sensor::PointCloud>>>
SvmClassfiy::Sengment(const cartographer::sensor::PointCloud& point_cloud) {
  return {};
}
