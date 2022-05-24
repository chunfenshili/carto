#ifndef _SUBMAP_SLICES_PROCESS_H
#define _SUBMAP_SLICES_PROCESS_H
#include "Eigen/Geometry"
// #include "cairo/cairo.h"
#include <map>
#include <vector>
#include <mutex>
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"
class SubmapSlicesProcess;
namespace  cartographer_ros{
class Node;
}

class SubmapFetch {
 public:
  SubmapFetch(cartographer_ros::Node* node);
  std::tuple<cartographer::mapping::MapById<
                 cartographer::mapping::SubmapId,
                 cartographer::mapping::PoseGraphInterface::SubmapData>,
             cartographer::mapping::MapById<
                 cartographer::mapping::SubmapId,
                 cartographer::mapping::PoseGraphInterface::SubmapData>,
             bool>
  Updata();
 void SetUpdate(){
     force_update_ = true; 
  }
 private:
  cartographer::mapping::MapById<
      cartographer::mapping::SubmapId,
      cartographer::mapping::PoseGraphInterface::SubmapData>
  UpdataActiveSubmapData();
  void UpdateLastSubmapData();
  bool UpdataAllSubmapData();
  bool IsOptimazation();
  bool IsSubmapFinish(const cartographer::mapping::SubmapId& submap_id);
  cartographer::mapping::MapById<
      cartographer::mapping::SubmapId,
      cartographer::mapping::PoseGraphInterface::SubmapData>
  UpdataFinishSubmapData();
  //
  cartographer_ros::Node* node_;
  cartographer::mapping::MapById<
      cartographer::mapping::SubmapId,
      cartographer::mapping::PoseGraphInterface::SubmapData>
      all_submapdata_;
  std::set<int> frozen_trajectories_;   
  std::vector<cartographer::mapping::SubmapId> finishd_submap_ids_;
  std::vector<cartographer::mapping::SubmapId> all_finish_submap_ids_;
  std::vector<cartographer::mapping::SubmapId> active_submap_ids_;
  std::map<cartographer::mapping::SubmapId, cartographer::transform::Rigid3d>
      last_submap_ids_with_pose_;
        bool force_update_=false;
};

class SubmapSlicesProcess {
 public:
  friend class SubmapFetch;
  SubmapSlicesProcess(cartographer_ros::Node* node);
  ~SubmapSlicesProcess();
  void SetMapExtric(const cartographer::transform::Rigid3d extri) {
    extric_ = extri;
    submap_fetch_->SetUpdate();
     
  }
  cartographer::io::PaintSubmapSlicesResult Process();
  cartographer::io::PaintSubmapSlicesResult GetLocalSubmapSlice();
 private:
  std::map<::cartographer::mapping::SubmapId, cartographer::io::SubmapSlice>
  ConvertSubmapDataToSubmapslice(
      const cartographer::mapping::MapById<
          cartographer::mapping::SubmapId,
          cartographer::mapping::PoseGraphInterface::SubmapData>& submapdatas);

  cartographer::io::PaintSubmapSlicesResult PaintSubmapslices(
      const std::map<cartographer::mapping::SubmapId,
                     cartographer::io::SubmapSlice>& submaps,
      const double resolution, bool updata_finish);

  void UpdataLocalSumapSlices(cairo_surface_t* slices,
                              const Eigen::Array2f& origin);
  Eigen::Array2f local_submap_slice_origin_;
  std::unique_ptr<SubmapFetch> submap_fetch_;
  cartographer::io::UniqueCairoSurfacePtr last_surface_;
  cartographer::io::UniqueCairoSurfacePtr local_submap_slice_;
  Eigen::AlignedBox2f last_bounding_box_;
  std::mutex mutex_;
  cartographer::transform::Rigid3d extric_;
  // ToCreateOccupancyGridMsg
};

// }  // namespace modules
#endif