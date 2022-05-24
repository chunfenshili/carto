#include "cartographer/mapping/internal/2d/local_globel_pose_fusion.h"
#include "cartographer/mapping/internal/2d/local_slide_window.h"
#include <algorithm>
namespace cartographer {
namespace mapping {

LocalGloblePoseFusion::LocalGloblePoseFusion(
    const LocalGloblePoseFusionOption& option, PoseGraph2D* pose_graph,
    CallBack call_back)
    : option_(option), pose_graph_(pose_graph), call_back_(call_back) {
  auto node_data = pose_graph->GetTrajectoryNodes();
  auto trajectories_state = pose_graph->GetTrajectoryStates();
  // pose_graph->constraint_builder_.SetConstraintMinScore({0.9, 0.85});
  std::set<int> frozen_trajectories;
  if (trajectories_state.size() >= 1) {
    for (const auto& it : trajectories_state) {
      if (it.second == PoseGraphInterface::TrajectoryState::FROZEN ||
          it.second == PoseGraphInterface::TrajectoryState::FINISHED) {
        frozen_trajectories.insert(it.first);
      }
    }
  }
  MapById<NodeId, TrajectoryNode> node_frozen;
  for (auto node : node_data) {
    if (frozen_trajectories.count(node.id.trajectory_id)) {
      node_frozen.Insert(node.id, node.data);
    }
  }
  scan_match_globle_ = absl::make_unique<ScanMatchGloble>(node_frozen);
  local_slide_widonw_ =
      absl::make_unique<LocalSlideWindow>(LocalSlideWindowOption{});
}
//
//

constexpr int k = 4;
constexpr double k_22() { return std::pow(2, k / 2); }
//
template <int N>
struct Factorial {
  enum { value = N * Factorial<N - 1>::value};
};
//
template <>
struct Factorial<0> {
  enum { value = 1 };
};

constexpr double F() { return Factorial<k / 2>::value; }
double Ration(int num) {
  return 1. / (k_22() * F()*2) *
         std::pow((static_cast<double>(num + 4) / 4.), k / 2 - 1) *
         std::exp((-num + 1) / 40.);
}

double LocalGloblePoseFusion::SampRation() {
  compute_sample_ration_num_++;
  if (compute_sample_ration_num_ >= 200) {
    compute_sample_ration_num_ = 200;
  }
  return Ration(compute_sample_ration_num_);
  
}

void LocalGloblePoseFusion::SetPoseGrapherSamplerAddScore(bool low) {
  if (low) {
    LOG(INFO) << "globle score is continue low ";
    // double ration = SampRation();
    // pose_graph_->constraint_builder_.SetConstraintMinScore({0.60, 0.65});
    // if (!pose_graph_->global_localization_samplers_only_frozen_submap_) {
    //   pose_graph_->global_localization_samplers_only_frozen_submap_ =
    //       absl::make_unique<double>(ration);
    // }
    // *pose_graph_->global_localization_samplers_only_frozen_submap_ = ration;
    return;
  }
  compute_sample_ration_num_ = 0;
  // pose_graph_->constraint_builder_.SetConstraintMinScore({0.90, 0.92});
  // pose_graph_->global_localization_samplers_only_frozen_submap_ = nullptr;
}
void LocalGloblePoseFusion::SampleRationAndRelocationScoreCompute(
    const double& ave_score) {
  if (ave_score <= kLow_score_threash_hold) {
    if (globle_score_num < kGloble_score_num) {
      globle_score_num++;
    }
  } else if (globle_score_num > -kGloble_score_num) {
    globle_score_num--;
  }
  if (globle_score_num >= kGloble_score_num) {
    globle_score_num = 0;
    SetPoseGrapherSamplerAddScore(true);
  } else if (globle_score_num <= -kGloble_score_num) {
    LOG(INFO) << "globle score is continue ok ";
    globle_score_num = 0;
    SetPoseGrapherSamplerAddScore(false);
  }
}
double LocalGloblePoseFusion::SmoothScore(const double& score) {
  if (score < kLow_score_threash_hold) {
    slide_scores_ = std::vector<double>(slide_scores_.size(), 0);
  }
  slide_scores_.push_back(score);
  if (slide_scores_.size() > kScore_smooth_num) {
    slide_scores_.erase(slide_scores_.begin());
  }
  double ave_score =
      std::accumulate(slide_scores_.begin(), slide_scores_.end(), 0.0) /
      slide_scores_.size();
  SampleRationAndRelocationScoreCompute(ave_score);
  return ave_score > kLow_score_threash_hold ? ave_score : 0.0;
}
transform::Rigid2d LocalGloblePoseFusion::Comple(
    double score, const transform::Rigid2d& pose1,
    const transform::Rigid2d& pose2) {
  transform::Rigid2d delta_pose = (pose2).inverse() * (pose1);
  // double factor = option_.globle_map_wight /
  //                 (option_.globle_map_wight + option_.local_wight);
  const double factor = SmoothScore(score);

  const Eigen::Vector2d start =
      pose2.translation() + delta_pose.translation() * factor;
  const double yaw =
      pose2.rotation().angle() + factor * delta_pose.rotation().angle();
  return transform::Rigid2d(start, yaw);
}
//
//
std::unique_ptr<transform::Rigid2d> LocalGloblePoseFusion::Fusion(
    const LocalFusionData& data) {
  transform::Rigid3d local_to_globe ;
  //
  if (pose_graph_ != nullptr) {
    auto trajector_state = pose_graph_->GetTrajectoryStates();
    int new_trajector = (--trajector_state.end())->first;
    local_to_globe = pose_graph_->GetLocalToGlobalTransform(new_trajector);
  }
  //
  transform::Rigid2d globle_init_pose = transform::Project2D(
      local_to_globe * transform::Embed3D(*data.init_pose));

  auto globle_map_estimate_result = scan_match_globle_->ScanMatch(
      cartographer::common::FromUniversal(0), globle_init_pose,
      data.filtered_gravity_aligned_point_cloud);
  //
  // if (globle_map_estimate_result.grid == nullptr) {
  //   SampleRationAndRelocationScoreCompute(0.0);
  //   return nullptr;
  // }
  //
  if (globle_map_estimate_result.pose == nullptr) {
    call_back_(nullptr, nullptr, 0.0);
    return nullptr;
  }
  transform::Rigid2d local_match_globle_pose = transform::Project2D(
      local_to_globe.inverse() *
      transform::Embed3D(*globle_map_estimate_result.pose));

  auto fusion_pose = Comple(globle_map_estimate_result.score,
                            local_match_globle_pose, *data.local_pose);
  //
  if (call_back_) {
    call_back_(globle_map_estimate_result.point_cloud,
               globle_map_estimate_result.grid,
               globle_map_estimate_result.score);
  }
  return absl::make_unique<transform::Rigid2d>(fusion_pose);
}
//
//
std::tuple<
    transform::Rigid3d,
    std::unique_ptr<
        cartographer::mapping::LocalTrajectoryBuilder2D::InsertionResult>>
LocalGloblePoseFusion::InsertSlideWidonws(const LocalFusionData& local_data) {
  LOG(INFO)<<*local_data.init_pose;
  auto pose = Fusion(local_data);
  LocalFusionData temp = local_data;
  temp.local_pose = std::make_shared<transform::Rigid2d>(transform::Project2D(
      transform::Embed3D(*temp.local_pose) *
      transform::Rigid3d::Rotation(local_data.gravity_alignment)));
  // LOG(INFO) << *pose;
  temp.init_pose= std::make_shared<transform::Rigid2d>(transform::Project2D(
      transform::Embed3D(*pose) *
      transform::Rigid3d::Rotation(local_data.gravity_alignment)));

  return local_slide_widonw_->Insert(temp);
}
//
//
void LocalGloblePoseFusion::AddImuData(const sensor::ImuData& imu_data) {
  // local_slide_widonw_->AddImuData(imu_data);
}
//
//

void LocalGloblePoseFusion::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  // local_slide_widonw_->AddOdometryData(odometry_data);
}
LocalGloblePoseFusion::~LocalGloblePoseFusion(){};
}  // namespace mapping

}  // namespace cartographer