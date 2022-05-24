/*
 * @Author: your name
 * @Date: 2022-02-19 13:29:20
 * @LastEditTime: 2022-02-19 16:45:53
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /carto_0218/src/cartographer/cartographer/cartographer/mapping/internal/2d/local_globel_pose_fusion.h
 */
#ifndef _LOCAL_GLBOE_POSE_FUSION_H
#define _LOCAL_GLBOE_POSE_FUSION_H
#include "cartographer/mapping/internal/2d/scan_match_globle.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "functional"
#include "cartographer/mapping/pose_graph_interface.h"
namespace cartographer {
namespace mapping {
class LocalSlideWindow;

struct LocalFusionData {

  cartographer::common::Time time;
  std::shared_ptr<transform::Rigid2d> init_pose;
  std::shared_ptr<transform::Rigid2d>  local_pose;
  sensor::PointCloud filtered_gravity_aligned_point_cloud;
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  Eigen::Quaterniond gravity_alignment;
  bool front = false;
};

struct LocalGloblePoseFusionOption {
  double globle_map_wight;
  double local_wight;
};

class LocalGloblePoseFusion {
 public:
  using CallBack = std::function<void(std::shared_ptr<sensor::PointCloud>,
                                      std::shared_ptr<Grid2D>,double)>;
  LocalGloblePoseFusion(const LocalGloblePoseFusionOption& option,
                        PoseGraph2D* pose_graph,CallBack call_back);
  void SetLocalMap(std::shared_ptr<PoseGraphInterface::SubmapData> one_submap) {
    scan_match_globle_->SetLocalMap(one_submap);
  }
  std::unique_ptr<transform::Rigid2d> Fusion(const LocalFusionData& data);

  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

 std::tuple<transform::Rigid3d, std::unique_ptr<
      cartographer::mapping::LocalTrajectoryBuilder2D::InsertionResult>>
  InsertSlideWidonws(const LocalFusionData& local_data);
  ~LocalGloblePoseFusion();
 private:
  const int kGloble_score_num = 10;
  int globle_score_num = 0;
  constexpr static double kLow_score_threash_hold = 0.7;
  constexpr static int kScore_smooth_num = 10;
  int compute_sample_ration_num_=0;
  double SampRation();
  double SmoothScore(const double& score);
  transform::Rigid2d Comple(double score, const transform::Rigid2d&,
                            const transform::Rigid2d&);
  void SampleRationAndRelocationScoreCompute(const double&);
  void SetPoseGrapherSamplerAddScore(bool);
  std::vector<double> slide_scores_;
  LocalGloblePoseFusionOption option_;
  PoseGraph2D* pose_graph_;
  std::unique_ptr<ScanMatchGloble> scan_match_globle_;
  std::unique_ptr<LocalSlideWindow> local_slide_widonw_;
  CallBack call_back_;
};
}  // namespace mapping
}  // namespace cartographer
#endif