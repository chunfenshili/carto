#ifndef _PLICP_IMPLEMENTATION_
#define  _PLICP_IMPLEMENTATION_
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "cartographer/mapping/internal/2d/icp_implementation.h"

#include "ceres/autodiff_cost_function.h"

namespace cartographer {
namespace mapping {
class PlIcp : public IcpInterface {
  public:
  bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
             const std::vector<Eigen::Vector3f>& source_points,
             const std::vector<Eigen::Vector3f>& target_points,
             Eigen::Matrix4f& pose_estimate);
  private:
  std::unique_ptr<KdTree> nano_kd_tree_;
};
}  // namespace mapping
}  // namespace cartographer

#endif