#include "cartographer/mapping/internal/2d/plicp_implementation.h"

#ifdef USE_PCL
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#else
#include "cartographer/mapping/internal/2d/nanoflann.h"
#endif
namespace cartographer {
namespace mapping {
namespace {
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization {
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};
class PointLineCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointLineCostFunction(const Eigen::Vector3f p, const Eigen::Vector3f near_p1,
                        const Eigen::Vector3f near_p2)
      : p_(p), near_p1_(near_p1), near_p2_(near_p2) {}
  template <typename T>
  bool operator()(const T* const pose_t, const T* const pose_q,
                  T* residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose_t);
    Eigen::Map<const Eigen::Quaternion<T>> q(pose_q);

    Eigen::Matrix<T, 3, 1> normalized_p1p2 =
        (near_p1_ - near_p2_).normalized().template cast<T>();
    Eigen::Matrix<T, 3, 1> normal_p1p2{-normalized_p1p2.y(),
                                       normalized_p1p2.x(), T(0)};
    Eigen::Matrix<T, 3, 1> p_a =
        q.toRotationMatrix() * p_.template cast<T>() + t;
    Eigen::Matrix<T, 3, 1> diff_pp1 = (p_a - near_p1_.template cast<T>());
    T err = normal_p1p2.dot(diff_pp1);
    //    std::cout<<err<<std::endl;
    residuals[0] = err * T(1.0);
    return true;
  }
  static ceres::CostFunction* Creat(const Eigen::Vector3f p,
                                    const Eigen::Vector3f near_p1,
                                    const Eigen::Vector3f near_p2) {
    return new ceres::AutoDiffCostFunction<PointLineCostFunction, 1, 3, 4>(
        new PointLineCostFunction(p, near_p1, near_p2));
  }

 private:
  Eigen::Vector3f p_;
  Eigen::Vector3f near_p1_;
  Eigen::Vector3f near_p2_;
};
struct Pose2d {
  double x;
  double y;
  double yaw_radians;
};
struct Measurement {
  Eigen::Vector3f p;
  Eigen::Vector3f p1;
  Eigen::Vector3f p2;
};
#ifdef USE_PCL
using PointType = pcl::PointXYZ;
Eigen::Vector3f PclPointToEigenVector(const PointType& point) {
  Eigen::Vector3f vector3f;
  vector3f.x() = point.x;
  vector3f.y() = point.y;
  vector3f.z() = point.z;
  return vector3f;
}
void BuildOptimazationProblem(){};
pcl::PointCloud<PointType>::Ptr VectorPointToPclPointCloud(
    const std::vector<Eigen::Vector3f> points) {
  pcl::PointCloud<PointType>::Ptr point_cloude(
      new pcl::PointCloud<PointType>());
  for (auto point : points) {
    PointType pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    point_cloude->push_back(pcl_point);
  }
  return point_cloude;
}
#else
#endif

}  // namespace

bool PlIcp::Match(const Option& option, const Eigen::Matrix4f& init_pose,
                  const std::vector<Eigen::Vector3f>& source_points,
                  const std::vector<Eigen::Vector3f>& target_points,
                  Eigen::Matrix4f& pose_estimate) {
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  ceres::LossFunction* loss_function = NULL;
  Eigen::Vector3d t = init_pose.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond q =
      Eigen::Quaterniond(init_pose.block<3, 3>(0, 0).cast<double>());
  ceres::Problem problem;
  // ceres::LocalParameterization* quaternion_local =
  //        AngleLocalParameterization::Create();
#ifdef USE_PCL
  pcl::PointCloud<PointType>::Ptr pcl_input_points =
      VectorPointToPclPointCloud(source_points);
  pcl::PointCloud<PointType>::Ptr pcl_targe_points =
      VectorPointToPclPointCloud(target_points);
  pcl::KdTreeFLANN<PointType> kd_tree;
  kd_tree.setInputCloud(pcl_targe_points);
  for (int i = 0; i < pcl_input_points->size(); i++) {
    Eigen::Vector3f init_point =
        init_pose.block<3, 3>(0, 0) * source_points[i] +
        init_pose.block<3, 1>(0, 3);
    PointType search_point;
    search_point.x = init_point.x();
    search_point.y = init_point.y();
    search_point.z = init_point.z();
    int k = 2;
    std::vector<int> index(k, -1);
    std::vector<float> distance(k);
    if (kd_tree.nearestKSearch(search_point, 2, index, distance) == k) {
      Eigen::Vector3f p =
          source_points[i];  // PclPointToEigenVector(init_point);
      Eigen::Vector3f near_p1 =
          PclPointToEigenVector(pcl_targe_points->points[index[0]]);
      Eigen::Vector3f near_p2 =
          PclPointToEigenVector(pcl_targe_points->points[index[1]]);
      problem.AddResidualBlock(
          PointLineCostFunction::Creat(p, near_p1, near_p2),
          new ceres::CauchyLoss(0.5), t.data(), q.coeffs().data());
    }
  }
#else
  int vali_num  = 0;
  PointCloudFlann nano_point_clouds(target_points);
  nano_kd_tree_ = absl::make_unique<KdTree>(
      3, nano_point_clouds, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  nano_kd_tree_->buildIndex();
  for (int i = 0; i < source_points.size(); i++) {
    Eigen::Vector3f init_point =
        init_pose.block<3, 3>(0, 0) * source_points[i] +
        init_pose.block<3, 1>(0, 3);
    const float query_pt[3] = {init_point[0], init_point[1], init_point[2]};
    size_t num_results = 2;
    std::vector<uint32_t> index(num_results, 0);
    std::vector<double> squared_distance(num_results, 100.0);
    if (nano_kd_tree_->knnSearch(query_pt, num_results, &index[0],
                                 &squared_distance[0]) != num_results) {
      continue;
    }
    const double max_dis = 1;
    double squared_distance_th = max_dis * max_dis;
    if (squared_distance[0] > squared_distance_th ||
        squared_distance[1] > squared_distance_th) {
      continue;
    }
    vali_num++;
    Eigen::Vector3f p = source_points[i];  // PclPointToEigenVector(init_point);
    Eigen::Vector3f near_p1 = target_points[index[0]];
    Eigen::Vector3f near_p2 = target_points[index[1]];
    problem.AddResidualBlock(PointLineCostFunction::Creat(p, near_p1, near_p2),
                             nullptr, t.data(),
                             q.coeffs().data());
  }
  if (vali_num < source_points.size()) {
    // pose_estimate = Eigen::Matrix4f::Identity();
    return false;
  }
#endif

  problem.SetParameterization(q.coeffs().data(), quaternion_local);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 4;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  Eigen::Matrix3f Mat = q.toRotationMatrix().cast<float>();
  pose_estimate.block<3, 3>(0, 0) =
      Mat;  // Eigen::AngleAxisf(yaw,Eigen::Vector3f(0,0,1)).matrix();
  pose_estimate.block<3, 1>(0, 3) = t.cast<float>();
  return true;
}

}  // namespace mapping
}  // namespace cartographer
