#include "scan_match_globle.h"
#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "Eigen/Core"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/mapping/2d/submap_2d.h"
namespace cartographer {
namespace mapping {



namespace {
using namespace cartographer::mapping::scan_matching;
cartographer::mapping::ValueConversionTables conversion_tables;
class DummyFileResolverTest : public cartographer::common::FileResolver {
 public:
  DummyFileResolverTest() {}

  DummyFileResolverTest(const DummyFileResolverTest&) = delete;
  DummyFileResolverTest& operator=(const DummyFileResolverTest&) = delete;

  ~DummyFileResolverTest() {}

  std::string GetFileContentOrDie(const std::string& unused_basename) {
    LOG(FATAL) << "Not implemented";
    return {};
  }

  std::string GetFullPathOrDie(const std::string& unused_basename) {
    LOG(FATAL) << "Not implemented";
    return {};
  }
};

cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterTestOptions2D() {
  std::string code =
      "return { "
      "insert_free_space =true, "
      "hit_probability = 0.95, "
      "miss_probability = 0.2, "
      "}";
  auto parameter_dictionary = absl::make_unique<common::LuaParameterDictionary>(
      code, absl::make_unique<DummyFileResolverTest>());

  return CreateProbabilityGridRangeDataInserterOptions2D(
      parameter_dictionary.get());
}
mapping::ProbabilityGrid CreateProbabilityGrid(
    const double resolution,
    mapping::ValueConversionTables* conversion_tables) {
  constexpr int kInitialProbabilityGridSize = 100;
  Eigen::Vector2d max =
      0.5 * kInitialProbabilityGridSize * resolution * Eigen::Vector2d::Ones();
  return mapping::ProbabilityGrid(
      mapping::MapLimits(resolution, max,
                         mapping::CellLimits(kInitialProbabilityGridSize,
                                             kInitialProbabilityGridSize)),
      conversion_tables);
}

cartographer::mapping::scan_matching::proto::
    RealTimeCorrelativeScanMatcherOptions
    CreateRealTimeCorrelativeScanMatcherTestOptions2D() {
  auto code = R"text(
      return {
      linear_search_window = 0.2,
      angular_search_window =0.3, 
      translation_delta_cost_weight = 0.1, 
      rotation_delta_cost_weight = 0.1, 
      })text";
  auto parameter_dictionary = absl::make_unique<common::LuaParameterDictionary>(
      code, absl::make_unique<DummyFileResolverTest>());
  return cartographer::mapping::scan_matching::
      CreateRealTimeCorrelativeScanMatcherOptions(parameter_dictionary.get());
}

cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
CreatCeresScanMatchingOption() {
  auto code = R"text(
        return {
          occupied_space_weight = 5.,
          translation_weight = 10.,
          rotation_weight = 15.,
          ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 5,
            num_threads = 10,
          },
        })text";
  auto parameter_dictionary = absl::make_unique<common::LuaParameterDictionary>(
      code, absl::make_unique<DummyFileResolverTest>());
  return cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(
      parameter_dictionary.get());
}
cartographer::sensor::proto::AdaptiveVoxelFilterOptions
CreateVoxelFilterOptions() {
  auto code = R"text(
          return {
            max_length = 0.1,
            min_num_points = 200,
            max_range = 60.5,
          })text";
  auto parameter_dictionary = absl::make_unique<common::LuaParameterDictionary>(
      code, absl::make_unique<DummyFileResolverTest>());
  return cartographer::sensor::CreateAdaptiveVoxelFilterOptions(
      parameter_dictionary.get());
}

double ScorePose(const Grid2D* grid,
                 const sensor::PointCloud& point_cloud,
                 const transform::Rigid2d& pose) {
  sensor::PointCloud trans_point_cloud = sensor::TransformPointCloud(
      point_cloud, transform::Embed3D(pose.cast<float>()));
  double score_sum = 0.0;
  for (const auto& point : trans_point_cloud) {
    auto index = grid->limits().GetCellIndex(
        Eigen::Vector2f{point.position.x(), point.position.y()});
    double local_score = 0.0;
    for (auto i : {-1, 0, 1}) {
      for (auto j : {-1, 0, 1}) {
        local_score = fmax(
            (1.0f - grid->GetCorrespondenceCost(index + Eigen::Array2i{i, j})),
            local_score);
      }
    }
    score_sum += local_score ;
  }
  return score_sum / (point_cloud.size()+1);
}

}  // namespace

LocalMapMatch::LocalMapMatch(const LocalMapMatchOption& option)
    : option_(option) {}

std::unique_ptr<transform::Rigid2d> LocalMapMatch::Match(
    const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& local_map) {
  Eigen::Matrix4f pose_estimate=  Eigen::Matrix4f::Identity();
  Eigen::Matrix4f init_estimate = Eigen::Matrix4f::Identity();
  init_estimate.block<3, 3>(0, 0) = transform::Embed3D(pose_prediction)
                                        .rotation()
                                        .toRotationMatrix()
                                        .cast<float>();
  init_estimate.block<2, 1>(0, 3) = pose_prediction.translation().cast<float>();

  std::vector<Eigen::Vector3f> source_points(
      filtered_gravity_aligned_point_cloud.size());
  std::vector<Eigen::Vector3f> target_points(local_map.size());

  for (int i = 0; i < filtered_gravity_aligned_point_cloud.size(); i++) {
    source_points[i] = filtered_gravity_aligned_point_cloud[i].position;
  }
  for (int i = 0; i < local_map.size(); i++) {
    target_points[i] = local_map[i].position;
  }

  if (!icp_matcher_.Match(IcpInterface::Option(), init_estimate, source_points,
                          target_points, pose_estimate)) {
    return nullptr;
  }
  //
  Eigen::Quaterniond q(pose_estimate.block<3, 3>(0, 0).cast<double>());
  Eigen::Vector3d t(pose_estimate.block<3, 1>(0, 3).cast<double>());
  transform::Rigid3d result(t, q);
  // LOG(INFO)<<transform::Project2D(result);
  return absl::make_unique<transform::Rigid2d>(transform::Project2D(result));
}
//
ScanMatchGloble::ScanMatchGloble(
    const MapById<NodeId, TrajectoryNode>& node_data)
    : real_time_correlative_scan_matcher_(
          CreateRealTimeCorrelativeScanMatcherTestOptions2D()) {
  local_map_match_ = absl::make_unique<LocalMapMatch>(LocalMapMatchOption{});
  local_map_.UpdataKdtree(node_data);
  ceres_scan_matcher_ =
      absl::make_unique<CeresScanMatcher2D>(CreatCeresScanMatchingOption());
}
ScanMatchGlobleRusult ScanMatchGloble::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  // LOG(INFO) << "init: " << pose_prediction;
  if (local_map_.UpdateNearNodeId(pose_prediction,filtered_gravity_aligned_point_cloud).empty()) {
    //return {nullptr, nullptr, nullptr, 0};
  }
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  // if (local_submap_data_ == nullptr) {
  //   auto local_grid = local_map_.Grid(pose_prediction);
  // }
  const Grid2D& grid =
      *std::static_pointer_cast<const typename cartographer::mapping::Submap2D>(
           local_submap_data_->submap)
           ->grid();

  auto local_point = local_map_.PointCloud(pose_prediction);
  
  if (local_point != nullptr) {
    auto initial_ceres_pose1 = local_map_match_->Match(
        pose_prediction, filtered_gravity_aligned_point_cloud, *local_point);
    if (initial_ceres_pose1 != nullptr) {
      initial_ceres_pose = *initial_ceres_pose1;
      LOG(INFO) << "icp :" << initial_ceres_pose;
    }
  }
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  double score  =0.0;
  auto initial_ceres_pose_back = initial_ceres_pose;
  
  score = real_time_correlative_scan_matcher_.Match(
      initial_ceres_pose, filtered_gravity_aligned_point_cloud, grid,
      &initial_ceres_pose);
  if(score<0.6){
    initial_ceres_pose  =initial_ceres_pose_back;
  }
  LOG(INFO) << "correcative : " << initial_ceres_pose << " score: " << score;
  ceres_scan_matcher_->Match(pose_prediction.translation(), initial_ceres_pose,
                             filtered_gravity_aligned_point_cloud, grid,
                             pose_observation.get(), &summary);

   score = ScorePose(&grid, filtered_gravity_aligned_point_cloud,
                     *pose_observation);

  LOG(INFO) << "cost :" << summary.final_cost << " score :" << score;
  LOG(INFO) << "result: " << *pose_observation;
  // if (score < 0.5) {
  //   pose_observation = nullptr;==
  // }
  
  return {nullptr, local_point, std::move(pose_observation), score};
}
//
std::shared_ptr<Grid2D> LocalMap::Grid(const transform::Rigid2d& init_pose,
                                       float resulution) {

  if(local_map_!=nullptr){
  //   const Grid2D& grid =
  //       *std::static_pointer_cast<const cartographer::mapping::Submap2D>(
  //            local_map_->submap)
  //            ->grid();
    return nullptr;
  }
  mapping::ProbabilityGridRangeDataInserter2D range_data_inserter(
      CreateProbabilityGridRangeDataInserterTestOptions2D());
  std::shared_ptr<mapping::ProbabilityGrid> probability_grid =
      std::make_shared<mapping::ProbabilityGrid>(
          CreateProbabilityGrid(resulution, &conversion_tables));
  std::ostringstream info;
  for (const auto node_id : near_node_id_) {
    info << node_id.trajectory_id << " " << node_id.node_index << std::endl;
    transform::Rigid3d node_pose = trajectory_nodes_.at(node_id).global_pose;
    auto& filtered_gravity_aligned_point_cloud =
        trajectory_nodes_.at(node_id)
            .constant_data->filtered_gravity_aligned_point_cloud;
    transform::Rigid3d pose_imu =
        node_pose *
        transform::Rigid3d{
            transform::Rigid3d::Rotation(
                trajectory_nodes_.at(node_id).constant_data->gravity_alignment)
                .inverse()};
    if (filtered_gravity_aligned_point_cloud.size() == 0) continue;
    auto point_cloud = TransformPointCloud(filtered_gravity_aligned_point_cloud,
                                           pose_imu.cast<float>());

    Eigen::Vector3f original{
        static_cast<float>(
            trajectory_nodes_.at(node_id).global_pose.translation().x()),
        static_cast<float>(
            trajectory_nodes_.at(node_id).global_pose.translation().y()),
        0.0};

    range_data_inserter.Insert({{original}, point_cloud, {}},
                               probability_grid.get());
  }
    //  LOG(INFO)<<info.str();
  return std::move(probability_grid);
}
sensor::PointCloud LocalMap::FilterPointCloud(
    const sensor::PointCloud& point_clouds) {
#ifdef USE_PCL
  // pcl::filters::GaussianKernel< PointInT, PointOutT >
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_filter(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (auto point : point_clouds) {
    pcl_point_cloud->push_back(
        {point.position.x(), point.position.y(), point.position.z()});
  }
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(pcl_point_cloud);
  outrem.setRadiusSearch(0.2);
  outrem.setMinNeighborsInRadius(15);
  outrem.filter(*pcl_point_cloud_filter);
  //
  sensor::PointCloud output;
  for (const auto& point : pcl_point_cloud_filter->points) {
    output.push_back({{point.x, point.y, point.z}});
  }
#else
  PointCloudFlann radis_point_cloud;
  std::vector<Eigen::Vector3f> points;
  for (const auto& point : point_clouds.points()) {
    points.push_back(point.position);
  }
  radis_point_cloud.SetPointCloud(points);
  KdTree kd_tree(3, radis_point_cloud,
                 nanoflann::KDTreeSingleIndexAdaptorParams(5));

  kd_tree.buildIndex();
  double radius = 0.2;
  sensor::PointCloud result;
  for (auto point : point_clouds) {
    std::vector<std::pair<size_t, double> > index;
    const double query_pt[3] = {point.position.x(), point.position.y(), point.position.z()};
    if (kd_tree.radiusSearch(query_pt, radius, index,
                             nanoflann::SearchParams()) >= 10) {
      result.push_back(point);
    }
  }
  LOG(INFO)<<"nano : "<<result.size();
  return result;
#endif
}
//
std::shared_ptr<sensor::PointCloud> LocalMap::PointCloud(
    const transform::Rigid2d& init_pose, float resolution) {
  std::vector<sensor::PointCloud::PointType> near_point_cloud;
  for (const auto node_id : near_node_id_) {
    // LOG(INFO)<<node_id.trajectory_id<<" "<<node_id.node_index;
    transform::Rigid3d node_pose = trajectory_nodes_.at(node_id).global_pose;
    auto& filtered_gravity_aligned_point_cloud =
        trajectory_nodes_.at(node_id)
            .constant_data->filtered_gravity_aligned_point_cloud;
    transform::Rigid3d pose_imu =
        node_pose *
        transform::Rigid3d{
            transform::Rigid3d::Rotation(
                trajectory_nodes_.at(node_id).constant_data->gravity_alignment)
                .inverse()};
    if (filtered_gravity_aligned_point_cloud.size() == 0) continue;
    auto point_cloud = TransformPointCloud(filtered_gravity_aligned_point_cloud,
                                           pose_imu.cast<float>());

    near_point_cloud.insert(near_point_cloud.end(), point_cloud.begin(),
                            point_cloud.end());
  }

  // auto output = FilterPointCloud(sensor::PointCloud(near_point_cloud));
  if(near_point_cloud.empty())return nullptr;
  auto output =  sensor::VoxelFilter(sensor::PointCloud(near_point_cloud),0.15);
  LOG(INFO)<<"out size :"<<output.size();
  return std::make_shared<sensor::PointCloud>(output);
}
//

std::vector<NodeId> LocalMap::UpdateNearNodeId(
    const transform::Rigid2d& init_pose,
    const sensor::PointCloud& point_cloud) {
  auto init_pose_point_cloud = sensor::TransformPointCloud(
      point_cloud, transform::Embed3D(init_pose).cast<float>());
  Eigen::AlignedBox2f point_bb;
  for (const auto& point : init_pose_point_cloud) {
    point_bb.extend(Eigen::Vector2f{point.position.x(), point.position.y()});
  }
  double radius = 160.0;
  std::vector<std::pair<size_t, double> > index;
  const double query_pt[3] = {init_pose.translation().x(),
                              init_pose.translation().y(), 0.0};
  if (nano_node_pose_kd_tree_ == nullptr) return {};
  auto size = nano_node_pose_kd_tree_->radiusSearch(query_pt, radius, index,
                                                    nanoflann::SearchParams());
  if (size < 2) {
    near_node_id_.clear();
    return {};
  }
  std::vector<sensor::PointWintIndex> submap_pose;
  for (auto ind : index) {
    auto pose_point = nano_node_pose_clouds_.GetPoint(ind.first);
    if (point_bb.contains(Eigen::Vector2f{pose_point.x(), pose_point.y()})) {
      submap_pose.push_back({nano_node_pose_clouds_.GetPoint(ind.first),
                             nano_node_pose_clouds_.Index(ind.first)});
    }
  }
  submap_pose = sensor::VoxelFilter(submap_pose, 1);
  if (submap_pose.size() > kNearNodeNum) {
    submap_pose.erase(submap_pose.end() - kNearNodeNum, submap_pose.end());
  }
  LOG(INFO) << submap_pose.size();
  std::vector<NodeId> result;
  for (auto ind : submap_pose) {
    result.push_back({ind.index[0], ind.index[1]});
  }
  near_node_id_ = result;
  return result;
}

std::vector<NodeId> LocalMap::UpdateNearNodeId(
    const transform::Rigid2d& init_pose) {

 #ifdef USE_PCL
  NodePoint node_point;
  node_point.x = init_pose.translation().x();
  node_point.y = init_pose.translation().y();
  node_point.z = 0;
  std::vector<int> index(kNearNodeNum, -1);
  std::vector<float> distance(kNearNodeNum);
  //   LOG(INFO)<<"UpdataNearNodeId";
  std::vector<float> distances;
  //
  double radius = 3.0;
  auto size =
      node_pose_kd_tree_.radiusSearch(node_point, radius, index, distances);

  if (size < 5) {
    near_node_id_.clear();
    return {};
  }
  if (index.size() > kNearNodeNum) {
    index.erase(index.begin() + kNearNodeNum, index.end());
  }
  // if (node_pose_kd_tree_.nearestKSearch(node_point, kNearNodeNum, index,
  //                                       distance) != kNearNodeNum) {
  //   near_node_id_.clear();
  //   return {};
  // }
  std::vector<NodeId> result;
  for (auto ind : index) {
    result.push_back({node_point_clouds_->points[ind].trajectory_id,
                      node_point_clouds_->points[ind].node_index});
  }
#else
  double radius = 80.0;
  std::vector<std::pair<size_t, double> > index;
  const double query_pt[3] = {init_pose.translation().x(),
                              init_pose.translation().y(), 0.0};
  if (nano_node_pose_kd_tree_ == nullptr) return{};
  auto size = nano_node_pose_kd_tree_->radiusSearch(query_pt, radius, index,
                                                    nanoflann::SearchParams());
 
  if (size < 10) {
    near_node_id_.clear();
    return {};
  }
  std::vector<sensor::PointWintIndex> submap_pose;
  for (auto ind : index) {
    submap_pose.push_back({nano_node_pose_clouds_.GetPoint(ind.first),
                           nano_node_pose_clouds_.Index(ind.first)});
  }
  submap_pose = sensor::VoxelFilter(submap_pose, 1);

  if (submap_pose.size() > kNearNodeNum) {
    submap_pose.erase(submap_pose.end() - kNearNodeNum,submap_pose.end());
  }
  LOG(INFO)<<submap_pose.size();
  std::vector<NodeId> result;
  for (auto ind : submap_pose) {
    result.push_back({ind.index[0], ind.index[1]});
  }
#endif
  near_node_id_ = result;
  return result;
}

void LocalMap::UpdataKdtree(const MapById<NodeId, TrajectoryNode>& node_data) {
  if (node_data.empty()) return;
  trajectory_nodes_ = node_data;


#ifdef USE_PCL
  node_point_clouds_.reset(new pcl::PointCloud<NodePoint>());
  for (auto node : trajectory_nodes_) {
    NodePoint node_point;
    node_point.x = node.data.global_pose.translation().x();
    node_point.y = node.data.global_pose.translation().y();
    node_point.z = 0;
    node_point.trajectory_id = node.id.trajectory_id;
    node_point.node_index = node.id.node_index;
    node_point_clouds_->push_back(node_point);
  }
  node_pose_kd_tree_.setInputCloud(node_point_clouds_);
#else
  for (auto node : trajectory_nodes_) {
    Eigen::Vector3f pose_point{node.data.global_pose.translation().x(),
                               node.data.global_pose.translation().y(), 0};
    Eigen::Vector2i pose_index{node.id.trajectory_id, node.id.node_index};
    nano_node_pose_clouds_.push_back(pose_point, pose_index);
  }

  nano_node_pose_kd_tree_ = absl::make_unique<KdTree>(
      3, nano_node_pose_clouds_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  nano_node_pose_kd_tree_->buildIndex();
#endif


  

  
}
}  // namespace mapping
}  // namespace cartographer
