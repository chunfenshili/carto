

#ifndef _ICP_IMPLEMENTATION_H
#define _ICP_IMPLEMENTATION_H
#ifdef USE_PCL
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#else
#include "cartographer/mapping/internal/2d/nanoflann.h"
#endif
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
namespace cartographer {
namespace mapping {

#ifndef USE_PCL

struct PointCloudFlann {
  PointCloudFlann(const std::vector<Eigen::Vector3f>& point_cloud)
      : points_(point_cloud) {}
  inline size_t kdtree_get_point_count() const { return points_.size(); }
  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    return points_[idx][dim];
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
  const std::vector<Eigen::Vector3f>& points_;
};

using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudFlann, double>,
    PointCloudFlann, 3, uint32_t>;

#endif

class IcpInterface {
 public:
  struct Option {
    int inter_num = 10;
    double err_tolerance = 0.1;
  };
  virtual bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
                     const std::vector<Eigen::Vector3f>& source_points,
                     const std::vector<Eigen::Vector3f>& target_points,
                     Eigen::Matrix4f& pose_estimate) = 0;
};

class OriginalIcp : public IcpInterface {
 public:
  bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
             const std::vector<Eigen::Vector3f>& source_points,
             const std::vector<Eigen::Vector3f>& target_points,
             Eigen::Matrix4f& pose_estimate);

 private:
  std::unique_ptr<KdTree> nano_kd_tree_;

  Eigen::Vector3f GetMeanVector(std::vector<Eigen::Vector3f>& points);
  void MinusMatrixMean(std::vector<Eigen::Vector3f>& points,
                       Eigen::Vector3f mean);
  std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>
  GetMinDistancMatchPonit(
      const std::tuple<const std::vector<Eigen::Vector3f>&,
                       const std::vector<Eigen::Vector3f>&>& points);
  std::unique_ptr<Eigen::Matrix4f>  GetPairPointsTransform(
      const std::tuple<const std::vector<Eigen::Vector3f>&,
                       const std::vector<Eigen::Vector3f>&>& points);
};
}  // namespace mapping
}  // namespace cartographer
#endif