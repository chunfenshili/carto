#ifndef _SCAN_MATCH_GLOBLE_H
#define _SCAN_MATCH_GLOBLE_H

#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/trajectory_node.h"

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/icp_implementation.h"
#include "cartographer/mapping/internal/2d/plicp_implementation.h"
#include "cartographer/mapping/internal/2d/nanoflann.h"

#include "cartographer/mapping/pose_graph_interface.h"
// #define USE_PCL
#ifdef USE_PCL
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#endif

#ifdef USE_PCL
struct NodePoint {
  PCL_ADD_POINT4D
  int trajectory_id;
  int node_index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    NodePoint,
    (float, x, x)(float, y, y)(float, z, z)(int, trajectory_id,
                                            trajectory_id)(int, node_index,
                                                           node_index))
#endif
namespace cartographer {
namespace mapping {

struct LocalMapMatchOption {};
class LocalMapMatch {
 public:
  LocalMapMatch(const LocalMapMatchOption& option);
  std::unique_ptr<transform::Rigid2d> Match(
      const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const sensor::PointCloud& local_map);

 private:
    OriginalIcp icp_matcher_;
  // PlIcp   icp_matcher_;
  LocalMapMatchOption option_;
};
struct LocalMapOption {
  int kNearNodeNum = 100;
};
class LocalMap {
 public:
  std::shared_ptr<Grid2D> Grid(const transform::Rigid2d& init_pose,
                               float resulution = 0.03);
  std::shared_ptr<sensor::PointCloud> PointCloud(
      const transform::Rigid2d& init_pose, float resolution = 0.1);
  void UpdataKdtree(const MapById<NodeId, TrajectoryNode>& node_data);
  std::vector<NodeId> UpdateNearNodeId(const transform::Rigid2d& init_pose);
   std::vector<NodeId> UpdateNearNodeId(const transform::Rigid2d& init_pose,
   const sensor::PointCloud& point_cloud);
  ~LocalMap(){};
  void SetLocalMap(
      std::shared_ptr<cartographer::mapping::PoseGraphInterface::SubmapData>
          one_submap) {

  }

 private:
  sensor::PointCloud FilterPointCloud(const sensor::PointCloud& point_clouds);
  //

  std::vector<NodeId> near_node_id_;
  int kNearNodeNum = 40;
#ifdef USE_PCL
  pcl::KdTreeFLANN<NodePoint> node_pose_kd_tree_;
  pcl::PointCloud<NodePoint>::Ptr node_point_clouds_;
#else
  // flann
  struct PointCloudFlann {
    void push_back(const Eigen::Vector3f& point, const Eigen::Vector2i& index) {
      points_.push_back(point);
      index_.push_back(index);
    }
    void SetPointCloud(const std::vector<Eigen::Vector3f>& point_cloud) {
      points_ = point_cloud;
    }
    Eigen::Vector3f GetPoint(size_t idx) { return points_[idx]; }
    Eigen::Vector2i Index(size_t idx) { return index_[idx]; }
    inline size_t kdtree_get_point_count() const { return points_.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
      return static_cast<double>(points_[idx][dim]);
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
    }
    std::vector<Eigen::Vector2i> index_;
    std::vector<Eigen::Vector3f> points_;
  };
  PointCloudFlann nano_node_pose_clouds_;
  using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, PointCloudFlann>, PointCloudFlann,
      3>;
  std::unique_ptr<KdTree> nano_node_pose_kd_tree_;
//
#endif
  MapById<NodeId, TrajectoryNode> trajectory_nodes_;
   std::shared_ptr<cartographer::mapping::PoseGraphInterface::SubmapData>
      local_map_; 
};

struct ScanMatchGlobleRusult {
  std::shared_ptr<Grid2D> grid;
  std::shared_ptr<sensor::PointCloud> point_cloud;
  std::unique_ptr<transform::Rigid2d> pose;
  double score;

};

class ScanMatchGloble {
 public:
  ScanMatchGloble(const MapById<NodeId, TrajectoryNode>& node_data);
  ScanMatchGlobleRusult ScanMatch(
      const common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud);
  void SetLocalMap(
      std::shared_ptr<cartographer::mapping::PoseGraphInterface::SubmapData>
          one_submap) {
    local_submap_data_ = one_submap;
  }

 private:
  std::unique_ptr<LocalMapMatch> local_map_match_;
  LocalMap local_map_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
   std::shared_ptr<cartographer::mapping::PoseGraphInterface::SubmapData>
      local_submap_data_; 
  std::unique_ptr<scan_matching::CeresScanMatcher2D> ceres_scan_matcher_;
};

}  // namespace mapping
}  // namespace cartographer
#endif