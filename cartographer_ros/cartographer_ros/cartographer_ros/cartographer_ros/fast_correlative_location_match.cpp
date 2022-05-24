#include "fast_correlative_location_match.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Modules/carto/msg_conversion.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/math.h"
#include "cartographer/common/task.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "node.h"
namespace modules {

namespace {

using cartographer::mapping::Grid2D;
using cartographer::mapping::MapById;
using cartographer::mapping::NodeId;
using cartographer::mapping::PoseGraphInterface;
using cartographer::mapping::Submap2D;
using cartographer::mapping::SubmapId;
using cartographer::sensor::PointCloud;
using cartographer::transform::Rigid2d;
using namespace cartographer::mapping::scan_matching;

cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherTestOptions2D(
    const int branch_and_bound_depth) {
  auto parameter_dictionary = cartographer::common::MakeDictionary(
      R"text(
      return {
         linear_search_window = 3.,
         angular_search_window = math.rad(360.),
         branch_and_bound_depth = )text" +
      std::to_string(branch_and_bound_depth) + "}");
  return CreateFastCorrelativeScanMatcherOptions2D(parameter_dictionary.get());
}

cartographer::sensor::proto::AdaptiveVoxelFilterOptions
CreateVoxelFilterOptions() {
  auto voxel_filter_parameter_dictionary =
      cartographer::common::MakeDictionary(R"text(
          return {
            max_length = 0.5,
            min_num_points = 100,
            max_range = 6.5,
          })text");
  return cartographer::sensor::CreateAdaptiveVoxelFilterOptions(
      voxel_filter_parameter_dictionary.get());
}

cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
CreatCeresScanMatchingOption() {
  auto ceres_parameter_dictionary = cartographer::common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 1.,
          translation_weight = 10.,
          rotation_weight = 30.,
          ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 20,
            num_threads = 2,
          },
        })text");
  return cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(
      ceres_parameter_dictionary.get());
}
}  // namespace
std::unique_ptr<PreLocationMatchResult> FastCorrelativeLocationMatch::Process(
    const RangeData& range_data, const Rigid3d& pose) {
  constexpr float kMinScore = 0.1f;


  MapById<SubmapId, PoseGraphInterface::SubmapData> submap_datas =
      parent_->GetAllSubmapData();


  std::vector<std::pair<float, Rigid2d>> pose_estimates;
  EUFY_ILOG(" FastCorrelativeLocationMatch start");
  cartographer::sensor::PointCloudWithIntensities points;
  cartographer::common::Time time;
  const auto options = CreateFastCorrelativeScanMatcherTestOptions2D(7);
  std::tie(points, time) = ToPointCloudWithIntensities(range_data);
  PointCloud point_cloud;
  for (auto point : points.points) {
    point_cloud.push_back({point.position});
  }

  auto voxel_options = CreateVoxelFilterOptions();
  point_cloud = cartographer::sensor::AdaptiveVoxelFilter(voxel_options)
                    .Filter(point_cloud);

  CeresScanMatcher2D ceres_scan_matcher(CreatCeresScanMatchingOption());

  EUFY_ILOG("submap_datas size %d:", submap_datas.size());
 
  for (auto submap : submap_datas) {
    const Grid2D& grid =
        *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();
    std::shared_ptr<FastCorrelativeScanMatcher2D> fast_correlative_scan_matcher;
    
    if (parent_->all_submap_comutaion_ == nullptr) {
      fast_correlative_scan_matcher =
          std::make_shared<FastCorrelativeScanMatcher2D>(grid, options);
    } else {
      EUFY_ILOG("start fast_coorelation from save file");
      auto pre_computationgrid_stack2d =
          parent_->all_submap_comutaion_->GetPrecomputationGridStack2D(
              submap.id);
      if (pre_computationgrid_stack2d != nullptr) {
        fast_correlative_scan_matcher =
            std::make_shared<FastCorrelativeScanMatcher2D>(
                parent_->all_submap_comutaion_->GetSubMapLimit(submap.id),
                std::move(pre_computationgrid_stack2d), options);
      } else {
        break;
      }

    EUFY_ILOG("start fast_coorelation done");
    }

    Rigid2d pose_estimate;
    float score;

    EUFY_ILOG("start MatchFullSubmap");
    fast_correlative_scan_matcher->MatchFullSubmap(point_cloud, kMinScore,
                                                  &score, &pose_estimate);
    pose_estimate = Rigid2d{
        pose_estimate.translation(),
        Rigid2d::Rotation2D(pose_estimate.rotation().angle() - 1.570796327)};
    EUFY_ILOG(
        "trajectory id: %d  submap id :%d  found  pose {x %f,y %f,t %f} with "
        "score %f ",
        submap.id.trajectory_id, submap.id.submap_index,
        pose_estimate.translation().x(), pose_estimate.translation().y(),
        pose_estimate.rotation().angle(), score);

    ceres::Solver::Summary unused_summary;
    ceres_scan_matcher.Match(pose_estimate.translation(), pose_estimate,
                             point_cloud, grid, &pose_estimate,
                             &unused_summary);
    pose_estimates.push_back({score, pose_estimate});
  }

  std::sort(pose_estimates.begin(), pose_estimates.end(),
            [](const std::pair<float, Rigid2d> lhs,
               const std::pair<float, Rigid2d> rhs) {
              return lhs.first < rhs.first;
            });

  return std::make_unique<PreLocationMatchResult>(PreLocationMatchResult{
      pose_estimates.begin()->first,
      cartographer::transform::Embed3D(pose_estimates.begin()->second)});
}
}  // namespace modules