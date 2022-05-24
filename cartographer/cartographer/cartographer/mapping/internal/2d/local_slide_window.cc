#include "cartographer/mapping/internal/2d/local_slide_window.h"

#include "cartographer/transform/transform.h"
#include "ceres/autodiff_cost_function.h"
#include "ceres/ceres.h"
namespace cartographer {
namespace mapping {
namespace {
template <typename T>
struct IntegrateImuResult {
  Eigen::Quaternion<T> delta_rotation;
};

template <typename T>
IntegrateImuResult<T> IntegrateImu(const std::deque<sensor::ImuData>& imu_data,
                                   const common::Time start_time,
                                   const common::Time end_time,
                                   std::deque<sensor::ImuData>::iterator* it) {
  common::Time current_time = start_time;
  IntegrateImuResult<T> result = {Eigen::Quaterniond::Identity().cast<T>()};

  while (current_time < end_time) {
    common::Time next_imu_data = common::Time::max();
    if (std::next(*it) != imu_data.end()) {
      next_imu_data = std::next(*it)->time;
    }
    // if(next_imu_data>end_time)break;
    common::Time next_time = std::min(next_imu_data, end_time);
    const T delta_t(common::ToSeconds(next_time - current_time));
    const Eigen::Matrix<T, 3, 1> delta_angle =
        (*it)->angular_velocity.template cast<T>() * delta_t;
    //  LOG(INFO) << delta_t;
    result.delta_rotation *=
        transform::AngleAxisVectorToRotationQuaternion(delta_angle);
    result.delta_rotation.normalized();
    current_time = next_time;
    if (current_time == next_imu_data) {
      ++(*it);
    }
  }
  return result;
}

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
class LocalSildeWindowCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalSildeWindowCostFunction(const std::array<double, 3> delta,
                               const std::array<double, 2>& weight)
      : delta_pose_(delta), weight_(weight) {}

  template <typename T>
  bool operator()(const T* const start, const T* const end,
                  T* residuals) const {
    const T cos_theta_i = cos(start[2]);
    const T sin_theta_i = sin(start[2]);
    const T delta_x = end[0] - start[0];
    const T delta_y = end[1] - start[1];
    const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                    -sin_theta_i * delta_x + cos_theta_i * delta_y,
                    end[2] - start[2]};

    residuals[0] = T(delta_pose_[0] - h[0]) * T(weight_[0]);
    residuals[1] = T(delta_pose_[1] - h[1]) * T(weight_[0]);
    residuals[2] = NormalizeAngle(T(delta_pose_[2] - h[2])) * T(weight_[1]);
    return true;
  }
  static ceres::CostFunction* Creat(std::array<double, 3> delta,
                                    const std::array<double, 2>& wight) {
    return new ceres::AutoDiffCostFunction<LocalSildeWindowCostFunction, 3, 3,
                                           3>(
        new LocalSildeWindowCostFunction(delta, wight));
  }
 private:
  std::array<double, 3> delta_pose_;
  std::array<double, 2> weight_;
};
class LocalSildeWindowImuCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalSildeWindowImuCostFunction(const double& delta_theta,
                                  const double& weight)
      : delta_theta_(delta_theta), weight_(weight) {}

  template <typename T>
  bool operator()(const T* const start, const T* const end,
                  T* residuals) const {
    const T delta_theta = end[2] - start[2];
    residuals[0] = NormalizeAngle(T(delta_theta_) - delta_theta) * T(weight_);
    return true;
  }
  static ceres::CostFunction* Creat(const double& delta, const double& wight) {
    return new ceres::AutoDiffCostFunction<LocalSildeWindowImuCostFunction, 1,
                                           3, 3>(
        new LocalSildeWindowImuCostFunction(delta, wight));
  }
 private:
  double delta_theta_;
  double weight_;
};
}  // namespace

LocalSlideWindow::LocalSlideWindow(const LocalSlideWindowOption& option)
    : option_(option) {}
 std::tuple< transform::Rigid3d, std::unique_ptr<
      cartographer::mapping::LocalTrajectoryBuilder2D::InsertionResult>>
LocalSlideWindow::Insert(const LocalFusionData& local_data_t) {
  if(local_data_t.front  ||  local_data_.size() >= slide_window_size){
      local_data_.push_back(local_data_t);
    // LOG(INFO)<<local_data_.size();
  }
  
  if (local_data_.size() > slide_window_size) {
    TrimImuData();
    TrimOdometryData();
    Optimazation();
    transform::Rigid3d pose_result =
        transform::Embed3D(*local_data_.back().local_pose) *
       transform::Rigid3d::Rotation( local_data_.back().gravity_alignment.inverse());
    if (local_data_t.front ) {
      local_data_.pop_front();
      auto local_data = local_data_.begin();
      if (local_data->insertion_submaps.size() == 0) {
        return {pose_result, nullptr};
      }
      LOG(INFO)<<transform::GetYaw(pose_result.rotation());
      auto result = std::make_unique<LocalTrajectoryBuilder2D::InsertionResult>(
          LocalTrajectoryBuilder2D::InsertionResult{
              std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
                  local_data->time,
                  local_data->gravity_alignment,
                  local_data->filtered_gravity_aligned_point_cloud,
                  {},
                  {},
                  {},
                  transform::Embed3D(*local_data->local_pose)}),
              std::move(local_data->insertion_submaps)});
      return {pose_result, std::move(result)};
    } else {
      local_data_.erase(std::prev(local_data_.end(),1));
      return {pose_result, nullptr};
    }
  }
  return {transform::Embed3D(*local_data_t.local_pose) *
              transform::Rigid3d::Rotation(
                  local_data_t.gravity_alignment.inverse()),
          nullptr};
}

void LocalSlideWindow::Optimazation() {
  ceres::Problem problem;
  // LOG(INFO)<<"start optimazation";
  ceres::LossFunction* loss_function = nullptr;
  std::vector<std::array<double, 3>> nodes_pose;
  ceres::LocalParameterization* quaternion_local =
      AngleLocalParameterization::Create();

  for (const auto& local_pose : local_data_) {
    std::array<double, 3> pose{{local_pose.local_pose->translation().x(),
                                local_pose.local_pose->translation().y(),
                                local_pose.local_pose->rotation().angle()}};
    nodes_pose.push_back(pose);
    problem.AddParameterBlock(nodes_pose.back().data(), 3);
  }
  //
//local pose
  for(int i  = 1;i<nodes_pose.size();i++){
    auto first = local_data_[i-1];
    auto end =  local_data_[i];
    transform::Rigid2d fisrt_init_pose =
        transform::Rigid2d(Eigen::Vector2d{first.init_pose->translation().x(),
                                           first.init_pose->translation().y()},
                           first.init_pose->rotation().angle());
    transform::Rigid2d end_init_pose =
        transform::Rigid2d(Eigen::Vector2d{end.init_pose->translation().x(),
                                           end.init_pose->translation().y()},
                           end.init_pose->rotation().angle());

    auto delta = fisrt_init_pose.inverse() * end_init_pose;
    // LOG(INFO)<<delta.translation();
    std::array<double, 3> relative_pose = {{delta.translation().x(),
                                            delta.translation().y(),
                                            delta.rotation().angle()}};

    std::array<double, 2> wight{{10.5, 10.5}};
    problem.AddResidualBlock(
        LocalSildeWindowCostFunction::Creat(relative_pose, wight),
        loss_function,nodes_pose[i-1].data(), nodes_pose[i].data());
    //  problem.SetParameterization(nodes_pose.front().data()+ 2, quaternion_local);
  }
  // odom
  for (int i = 1; i < local_data_.size(); i++) {
    auto first = local_data_[i - 1];
    auto end = local_data_[i];
    // LOG(INFO)<<"add odom data";
    std::unique_ptr<transform::Rigid2d> relative_odometry =
        CalculateOdometryBetweenNodes(first, end);
    if (relative_odometry != nullptr) {
      // LOG(INFO)<<*relative_odometry;
      std::array<double, 2> wight  = {{.5, 0.1}};
      std::array<double, 3> relative_pose = {{
          relative_odometry->translation().x(),
          relative_odometry->translation().y(),
          relative_odometry->rotation().angle()}};
      problem.AddResidualBlock(
          LocalSildeWindowCostFunction::Creat(relative_pose, wight),
          loss_function, nodes_pose[i - 1].data(), nodes_pose[i].data());
    }else {
      // 
LOG(INFO)<<"here is ";
    }
  }
// //imu
  auto imu_it = imu_data_.begin();
  for (int i = 1; i < local_data_.size(); i++) {
    auto first = local_data_[i - 1];
    auto end = local_data_[i];
    while (std::next(imu_it) != imu_data_.end() &&
           std::next(imu_it)->time <= first.time) {
      ++imu_it;
    }
    IntegrateImuResult<double> delta_rotation =
        IntegrateImu<double>(imu_data_, first.time, end.time, &imu_it);
    // LOG(INFO) << transform::GetYaw(delta_rotation.delta_rotation);
    // LOG(INFO)<<delta_rotation.delta_rotation.toRotationMatrix().eulerAngles(1,2,3);
    problem.AddResidualBlock(
        LocalSildeWindowImuCostFunction::Creat(
            transform::GetYaw(delta_rotation.delta_rotation), 0.5),
        loss_function, nodes_pose[i - 1].data(), nodes_pose[i].data());
  }

//

  problem.SetParameterBlockConstant(nodes_pose[0].data());
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 8;
  options.max_num_iterations = 10;
  // options.trust_region_strategy_type = ceres::DOGLEG;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (int i = 0; i < local_data_.size(); i++) {
    *local_data_[i].local_pose = transform::Rigid2d{
        Eigen::Vector2d{nodes_pose[i][0], nodes_pose[i][1]}, nodes_pose[i][2]};
  }
}

std::unique_ptr<transform::Rigid3d> LocalSlideWindow::InterpolateOdometry(
    const common::Time time) const {
  auto it = std::find_if(odometry_data_.begin(), odometry_data_.end(),
                         [&](const sensor::OdometryData& odom) {
                           if (odom.time >= time) return true;
                           return false;
                         });
  if (it == odometry_data_.end()) {
    return nullptr;
  }
  if (it == odometry_data_.begin()) {
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

std::unique_ptr<transform::Rigid2d>
LocalSlideWindow::CalculateOdometryBetweenNodes(
    const LocalFusionData& first_local_data,
    const LocalFusionData& second_local_data) const {
  const std::unique_ptr<transform::Rigid3d> first_node_odometry =
      InterpolateOdometry(first_local_data.time);
  const std::unique_ptr<transform::Rigid3d> second_node_odometry =
      InterpolateOdometry(second_local_data.time);

  if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
    transform::Rigid3d relative_odometry =
        first_node_odometry->inverse() * (*second_node_odometry);
    return absl::make_unique<transform::Rigid2d>(
        transform::Project2D(relative_odometry));
  }
  return nullptr;
}

void LocalSlideWindow::TrimImuData() {
  while (!imu_data_.empty() && (imu_data_[0].time < local_data_.front().time)) {
    imu_data_.pop_front();
  }
  // LOG(INFO)<<imu_data_.size();
}
void LocalSlideWindow::TrimOdometryData() {
  while (!odometry_data_.empty() &&
         odometry_data_[0].time < local_data_.front().time) {
    odometry_data_.pop_front();
  }
  // LOG(INFO)<<odometry_data_.size();
}

void LocalSlideWindow::AddImuData(const sensor::ImuData& imu_data) {
  
  imu_data_.push_back(imu_data);
  // LOG(INFO)<<imu_data_.size();
}
void LocalSlideWindow::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  odometry_data_.push_back(odometry_data);
}

}  // namespace mapping
}  // namespace cartographer
