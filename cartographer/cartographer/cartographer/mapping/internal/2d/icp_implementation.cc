#include "cartographer/mapping/internal/2d/icp_implementation.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

#include "Eigen/Core"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

bool OriginalIcp::Match(const Option& option, const Eigen::Matrix4f& init_pose,
                        const std::vector<Eigen::Vector3f>& source_points,
                        const std::vector<Eigen::Vector3f>& target_points,
                        Eigen::Matrix4f& pose_estimate) {
  bool reverse = false;
  std::vector<Eigen::Vector3f> source_temp(source_points.size());
  Eigen::Matrix4f T = init_pose;
  for (int inter = 0; inter < 2; inter++) {
    for (int i = 0; i < source_points.size(); i++) {
      source_temp[i] =
          T.block<3, 3>(0, 0) * source_points[i] + T.block<3, 1>(0, 3);
    }
    auto Tst = GetPairPointsTransform({source_temp, target_points});
    if (Tst == nullptr) return false;
    T = *Tst * T;
  }
  pose_estimate = T;
  return true;
}

Eigen::Vector3f OriginalIcp::GetMeanVector(
    std::vector<Eigen::Vector3f>& points) {
  return std::accumulate(points.begin(), points.end(), Eigen::Vector3f{0, 0, 0},
                         [](const Eigen::Vector3f& init,
                            const Eigen::Vector3f& second) -> Eigen::Vector3f {
                           return second + init;
                         }) /
         points.size();
}
void OriginalIcp::MinusMatrixMean(std::vector<Eigen::Vector3f>& points,
                                  Eigen::Vector3f mean) {
  for (int i = 0; i < points.size(); i++) {
    points[i] = points[i] - mean;
  }
}
std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>
OriginalIcp::GetMinDistancMatchPonit(
    const std::tuple<const std::vector<Eigen::Vector3f>&,
                     const std::vector<Eigen::Vector3f>&>& points) {
  const std::vector<Eigen::Vector3f>& source_points = std::get<0>(points);
  const std::vector<Eigen::Vector3f>& target_points = std::get<1>(points);
  std::vector<Eigen::Vector3f> match_source_points;
  std::vector<Eigen::Vector3f> match_target_points;
  PointCloudFlann nano_point_clouds(target_points);
  nano_kd_tree_ = absl::make_unique<KdTree>(
      3, nano_point_clouds, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  nano_kd_tree_->buildIndex();
  for (int i = 0; i < source_points.size(); i++) {
    const auto& init_point = source_points[i];
    const float query_pt[3] = {init_point[0], init_point[1], 0};
    size_t num_results = 1;
    std::vector<uint32_t> index(num_results, 0);
    std::vector<double> squared_distance(num_results, 100.0);
    if (nano_kd_tree_->knnSearch(&query_pt[0], num_results, &index[0],
                                 &squared_distance[0]) != num_results) {
      continue;
    }
    const double max_dis = 1;
    const double squared_distance_th = max_dis * max_dis;
    if (squared_distance[0] > squared_distance_th) {
      continue;
    }
    // LOG(INFO) << init_point;
    // LOG(INFO)<<target_points[index[0]];
    match_source_points.push_back({init_point[0], init_point[1],0});
    match_target_points.push_back(
        {target_points[index[0]][0], target_points[index[0]][1],0});
  }
  LOG(INFO)<<match_source_points.size()<<" "<<match_target_points.size();
  return {match_source_points, match_target_points};
}

std::unique_ptr<Eigen::Matrix4f>  OriginalIcp::GetPairPointsTransform(
    const std::tuple<const std::vector<Eigen::Vector3f>&,
                     const std::vector<Eigen::Vector3f>&>& points) {
  std::vector<Eigen::Vector3f> source_points;
  std::vector<Eigen::Vector3f> target_points;
  std::tie(source_points, target_points) =
      GetMinDistancMatchPonit({std::get<0>(points), std::get<1>(points)});
  if(source_points.size()<std::get<0>(points).size()*0.2)return nullptr;
  auto sor_mean = GetMeanVector(source_points);
  auto tar_mean = GetMeanVector(target_points);
  MinusMatrixMean(source_points, sor_mean);
  MinusMatrixMean(target_points, tar_mean);

  // Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 3>> sour_matrix(
  //     source_points.data()->data());
  // sour_matrix.resize(source_points.size(), 3);
  // Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 3>> target_matrix(
  //     target_points.data()->data());
  Eigen::MatrixXf sour_matrix;
  Eigen::MatrixXf  target_matrix;
  sour_matrix.resize(source_points.size(), 3);
  target_matrix.resize(target_points.size(), 3);
  for (int i = 0; i < source_points.size(); i++) {
    sour_matrix.row(i) = source_points[i];
  }
  for (int i = 0; i < target_points.size(); i++) {
    target_matrix.row(i) = target_points[i];
  }
  Eigen::Matrix3f M = sour_matrix.transpose() * target_matrix;
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3f U, V;
  U = svd.matrixU();
  V = svd.matrixV();
  auto R = V * U.transpose();
  // std::cout<<R<<std::endl;
  auto t = tar_mean - R * sor_mean;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return absl::make_unique<Eigen::Matrix4f>(T);
}

}  // namespace mapping
}  // namespace cartographer