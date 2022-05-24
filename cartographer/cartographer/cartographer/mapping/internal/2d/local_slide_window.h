#ifndef _LOCAL_SLIDE_WIDOW_H
#define  _LOCAL_SLIDE_WIDOW_H
#include "cartographer/mapping/trajectory_node.h"
#include <queue>
#include "cartographer/mapping/internal/2d/local_globel_pose_fusion.h"
#include "cartographer/mapping/internal/2d/ceres_slider_window_optimazation.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/imu_data.h"

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
namespace cartographer {
namespace mapping {
struct LocalSlideWindowOption {};

class LocalSlideWindow {
 public:
  LocalSlideWindow(const LocalSlideWindowOption& option);

 std::tuple< transform::Rigid3d, std::unique_ptr<
      cartographer::mapping::LocalTrajectoryBuilder2D::InsertionResult>>
  Insert(const LocalFusionData& local_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

 private:
  std::unique_ptr<double> CalulateYawRataionBetweeNOdes(
      const LocalFusionData& first_local_data,
      const LocalFusionData& second_local_data);
  std::unique_ptr<transform::Rigid2d> CalculateOdometryBetweenNodes(
      const LocalFusionData& first_local_data,
      const LocalFusionData& second_local_data) const;
  std::unique_ptr<transform::Rigid3d> InterpolateOdometry(
      const common::Time time) const;
  void TrimImuData();
  void TrimOdometryData();
  void Optimazation();
  static constexpr int slide_window_size = 10;
  LocalSlideWindowOption option_;
  std::deque<sensor::ImuData> imu_data_;
  std::deque<sensor::OdometryData> odometry_data_;
  std::deque<LocalFusionData> local_data_;
  // std::unique_ptr<CeresSliderWindowOptimazation> ceres_optimazation_;
};
}  // namespace mapping
}  // namespace cartographer

#endif