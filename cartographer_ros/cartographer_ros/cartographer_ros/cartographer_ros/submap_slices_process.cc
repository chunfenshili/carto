#include "submap_slices_process.h"

// #include "cartographer/io/submap_painter.h"
#include <algorithm>
#include <chrono>

#include "cartographer_ros/node.h"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/io/image.h"
#define EUFY_ELOG //printf
namespace {

constexpr int max_updata_submap = 5;
namespace carto = ::cartographer;
using namespace cartographer;
using carto::mapping::MapById;
using carto::mapping::SubmapId;
using namespace cartographer::io;
// using carto::mapping::PoseGraphInterface;
using namespace cartographer::mapping;

cartographer::mapping::ValueConversionTables conversion_tables;

Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

void CairoPaintSubmapSlices(
    const double scale,
    const std::map<::cartographer::mapping::SubmapId,
                   cartographer::io::SubmapSlice>& submaps,
    cairo_t* cr,
    std::function<void(const cartographer::io::SubmapSlice&)> draw_callback) {
  cairo_scale(cr, scale, scale);

  for (auto& pair : submaps) {
    const auto& submap_slice = pair.second;
    if (submap_slice.surface == nullptr) {
      return;
    }
    const Eigen::Matrix4d homo =
        ToEigen(submap_slice.pose * submap_slice.slice_pose).matrix();

    cairo_save(cr);
    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_slice.resolution;
    cairo_scale(cr, submap_resolution, submap_resolution);

    // Invokes caller's callback to utilize slice data in global cooridnate
    // frame. e.g. finds bounding box, paints slices.
    draw_callback(submap_slice);
    cairo_restore(cr);
  }
}
};  // namespace

SubmapFetch::SubmapFetch(cartographer_ros::Node* node) : node_(node){};
bool SubmapFetch::UpdataAllSubmapData() {
  auto pose_graph = node_->GetMapBuilderBridge()->MapBuilder()->pose_graph();
  all_submapdata_ = pose_graph->GetAllSubmapData();
  if (all_submapdata_.empty()) return false; 


   // 
  auto trajectories_state = pose_graph->GetTrajectoryStates();
  if (trajectories_state.size() >= 1) {
    for (const auto& it : trajectories_state) {
      if (it.second == PoseGraphInterface::TrajectoryState::FINISHED ||
          it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
        frozen_trajectories_.insert(it.first);
      }
    }
  }
  //
  all_finish_submap_ids_.clear();
  active_submap_ids_.clear();
  for (const auto& submap : all_submapdata_) {
    if (IsSubmapFinish(submap.id)) {
      all_finish_submap_ids_.push_back(submap.id);
    } else {
      active_submap_ids_.push_back(submap.id);
    }
  }
  return true;
}
bool SubmapFetch::IsSubmapFinish(
                                 const SubmapId& submap_id) {
  if (frozen_trajectories_.count(submap_id.trajectory_id) != 0) return true;
  if (all_submapdata_.at(submap_id).submap->insertion_finished()) {
    return true;
  }
  return false;
}
//

cartographer::mapping::MapById<
    cartographer::mapping::SubmapId,
    cartographer::mapping::PoseGraphInterface::SubmapData>
SubmapFetch::UpdataFinishSubmapData() {
  std::vector<cartographer::mapping::SubmapId> finsh_submap_id =
      all_finish_submap_ids_;
  //
  for (auto it = finishd_submap_ids_.begin();
       it != finishd_submap_ids_.end();) {
    auto itend = std::find(finsh_submap_id.begin(), finsh_submap_id.end(), *it);
    if (itend == finishd_submap_ids_.end()) {
      it = finishd_submap_ids_.erase(it);
    } else {
      ++it;
    }
  }
  std::vector<SubmapId> diff_submap;
  for (auto submap : finsh_submap_id) {
    auto it = std::find(finishd_submap_ids_.begin(), finishd_submap_ids_.end(),
                        submap);
    if (it == finishd_submap_ids_.end()) {
      diff_submap.push_back(submap);
    }
  }

  std::vector<SubmapId>::iterator it = diff_submap.end();
  for (int i = 0; i < max_updata_submap; i++) {
    it = std::next(diff_submap.begin(), i);
    if (it == diff_submap.end()) {
      break;
    }
  }
  std::vector<SubmapId> finshi_result_finish_submap_id{diff_submap.begin(), it};
  finishd_submap_ids_.insert(finishd_submap_ids_.end(), diff_submap.begin(),
                             it);
  cartographer::mapping::MapById<
      cartographer::mapping::SubmapId,
      cartographer::mapping::PoseGraphInterface::SubmapData>
      result;

  for (auto submap : finshi_result_finish_submap_id) {
    result.Insert(submap, all_submapdata_.at(submap));
  }
  return result;
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
SubmapFetch::UpdataActiveSubmapData() {
  MapById<SubmapId, PoseGraphInterface::SubmapData> output_data;
  for (auto submap : active_submap_ids_) {
    output_data.Insert(submap, all_submapdata_.at(submap));
  }
  return output_data;
}

bool SubmapFetch::IsOptimazation() {
  for (auto pair : last_submap_ids_with_pose_) {
    if ((pair.second.translation() -
         all_submapdata_.at(pair.first).pose.translation())
                .norm() > 0.05 ||
        (cartographer::transform::GetAngle(
             pair.second.inverse() * all_submapdata_.at(pair.first).pose) >
         0.005)) {
      EUFY_ELOG("Submap pose Optimized...!!!!!!!!!!!!!!!!");
      return true;
    }
  }
  return false;
}
void SubmapFetch::UpdateLastSubmapData() {
  //
  for (auto it = last_submap_ids_with_pose_.begin();
       it != last_submap_ids_with_pose_.end();) {
    auto itend = std::find(all_finish_submap_ids_.begin(),
                           all_finish_submap_ids_.end(), it->first);
    if (itend == all_finish_submap_ids_.end()) {
      it = last_submap_ids_with_pose_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto submap_id : all_finish_submap_ids_) {
    if (last_submap_ids_with_pose_.count(submap_id) == 0) {
      last_submap_ids_with_pose_.insert(
          {submap_id, all_submapdata_.at(submap_id).pose});
    }
  }
}
std::tuple<MapById<SubmapId, PoseGraphInterface::SubmapData>,
           MapById<SubmapId, PoseGraphInterface::SubmapData>, bool>
SubmapFetch::Updata() {
  //
  if (node_->GetMapBuilderBridge() == nullptr) return {{}, {}, false};
  if (node_->GetMapBuilderBridge()->MapBuilder() == nullptr)
    return {{}, {}, false};
  if (node_->GetMapBuilderBridge()->MapBuilder()->pose_graph() == nullptr)
    return {{}, {}, false};

  //
  if (!UpdataAllSubmapData()) return {{}, {}, false};
  UpdateLastSubmapData();  
  bool optimazation = false;
  if (IsOptimazation() || force_update_) {
    force_update_ = false;
    finishd_submap_ids_.clear();
    last_submap_ids_with_pose_.clear();
    optimazation = true;
  }
  auto finish_submap_data = UpdataFinishSubmapData();
  // std::this_thread::sleep_for(std::chrono::microseconds(100));
  auto active_submap_data = UpdataActiveSubmapData();
  return {finish_submap_data, active_submap_data, optimazation};
}

SubmapSlicesProcess::SubmapSlicesProcess(cartographer_ros::Node* node)
    : last_surface_(MakeUniqueCairoSurfacePtr(nullptr)),
                    local_submap_slice_(MakeUniqueCairoSurfacePtr(nullptr)) {
  EUFY_ELOG("Initialize SubmapSlicesProcess()....");
  submap_fetch_ = std::make_unique<SubmapFetch>(node);
}

std::map<SubmapId, SubmapSlice>
SubmapSlicesProcess::ConvertSubmapDataToSubmapslice(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submapdatas) {
  // EUFY_ELOG("ConvertSubmapDataToSubmapslice...");
  std::map<SubmapId, SubmapSlice> submap_slices;
  for (const auto& submap : submapdatas) {
    if (!submap.data.submap) {
      continue;
    }
    auto& slice = submap_slices[submap.id];
    auto submap_proto = submap.data.submap->ToProto(true);
    if (!submap_proto.has_submap_2d()) {
    }
    cartographer::io::FillSubmapSlice(extric_ * submap.data.pose, submap_proto,
                                      &slice, &conversion_tables);
  }
  return  std::move(submap_slices);
}
int test  = 0;
cartographer::io::PaintSubmapSlicesResult SubmapSlicesProcess::Process() {
  // while (!GetFinishFlag()) {
  // }
  // if(test++>=2){
  //   test = 0;
  //   SetFinishFlag(false);
  // }
  MapById<SubmapId, PoseGraphInterface::SubmapData> finish_submap_data;
  MapById<SubmapId, PoseGraphInterface::SubmapData> active_submap_data;
  bool optimazation =false;
  std::tie(finish_submap_data, active_submap_data, optimazation) =
      submap_fetch_->Updata();
  if (optimazation) {
    last_surface_.reset(nullptr);
    last_bounding_box_ = Eigen::AlignedBox2f();
  }

  if (!finish_submap_data.empty()) {
    EUFY_ELOG("finish_submap_data not empty ");
    auto slices = ConvertSubmapDataToSubmapslice(finish_submap_data);
    if (!slices.empty()) {
      PaintSubmapslices(slices, 0.05, true);
    } else {
      EUFY_ELOG("finish submap data maby nullptr");
    }
    return PaintSubmapSlicesResult(MakeUniqueCairoSurfacePtr(nullptr),
                                   Eigen::Array2f{0, 0});
  }

  if (active_submap_data.empty()) {
    return PaintSubmapSlicesResult(MakeUniqueCairoSurfacePtr(nullptr),
                                   Eigen::Array2f{0, 0});
  }
  auto slices = ConvertSubmapDataToSubmapslice(active_submap_data);
  if (!slices.empty()) {
    return PaintSubmapslices(slices, 0.05, false);
  }
  return PaintSubmapSlicesResult(MakeUniqueCairoSurfacePtr(nullptr),
                                 Eigen::Array2f{0, 0});
}

cartographer::io::PaintSubmapSlicesResult
SubmapSlicesProcess::PaintSubmapslices(
    const std::map<::cartographer::mapping::SubmapId,
                   cartographer::io::SubmapSlice>& submaps,
    const double resolution, bool updata_finish) {
  // EUFY_ELOG("paint submap updates........%d ", updata_finish);
  // auto start = std::chrono::steady_clock::now();
  Eigen::AlignedBox2f bounding_box = last_bounding_box_;
  {
    auto surface = MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(kCairoFormat, 1, 1));
    auto cr = io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    CairoPaintSubmapSlices(
        1. / resolution, submaps, cr.get(),
        [&update_bounding_box](const SubmapSlice& submap_slice) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_slice.width, 0);
          update_bounding_box(0, submap_slice.height);
          update_bounding_box(submap_slice.width, submap_slice.height);
        });
  }
  // 2. extend boudning box with 5 pixel boundary
  // TODO: !Dont update boundary when active submap is not extending..

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  auto surface = MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(kCairoFormat, size.x(), size.y()));
  {
    auto cr = MakeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    cairo_paint(cr.get());

    if (last_surface_ != nullptr) {
      double offset_x = last_bounding_box_.min().x() - bounding_box.min().x();
      double offset_y = last_bounding_box_.min().y() - bounding_box.min().y();
      cairo_save(cr.get());
      cairo_set_source_surface(cr.get(), last_surface_.get(), offset_x,
                               offset_y);
      cairo_paint(cr.get());
      cairo_restore(cr.get());
    }

    cairo_translate(cr.get(), origin.x(), origin.y());
    CairoPaintSubmapSlices(1. / resolution, submaps, cr.get(),
                           [&cr](const SubmapSlice& submap_slice) {
                             cairo_set_source_surface(
                                 cr.get(), submap_slice.surface.get(), 0., 0.);
                             cairo_paint(cr.get());
                           });
    cairo_surface_flush(surface.get());
  }
  if (updata_finish) {
    last_bounding_box_ = bounding_box;
    last_surface_ = std::move(surface);
    for (auto& submap_id : submaps) {
      EUFY_ELOG("uddata finshed submap id : %d : %d ",
                submap_id.first.trajectory_id, submap_id.first.submap_index);
    }
    cairo_surface_t* result_surface = cairo_image_surface_create_for_data(
        cairo_image_surface_get_data(last_surface_.get()), kCairoFormat,
        cairo_image_surface_get_width(last_surface_.get()),
        cairo_image_surface_get_height(last_surface_.get()),
        cairo_image_surface_get_stride(last_surface_.get()));
    surface = MakeUniqueCairoSurfacePtr(result_surface);
  }
  UpdataLocalSumapSlices(surface.get(), origin);
  return PaintSubmapSlicesResult(std::move(surface), origin);
}
SubmapSlicesProcess::~SubmapSlicesProcess() {}

cartographer::io::PaintSubmapSlicesResult
SubmapSlicesProcess::GetLocalSubmapSlice() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (local_submap_slice_ == nullptr) {
    return PaintSubmapSlicesResult(MakeUniqueCairoSurfacePtr(nullptr),
                                   Eigen::Array2f{0, 0});
  }
  cairo_surface_t* result_surface = cairo_image_surface_create_for_data(
      cairo_image_surface_get_data(local_submap_slice_.get()), kCairoFormat,
      cairo_image_surface_get_width(local_submap_slice_.get()),
      cairo_image_surface_get_height(local_submap_slice_.get()),
      cairo_image_surface_get_stride(local_submap_slice_.get()));
  return PaintSubmapSlicesResult(MakeUniqueCairoSurfacePtr(result_surface),
                                 local_submap_slice_origin_);
}
void SubmapSlicesProcess::UpdataLocalSumapSlices(cairo_surface_t* slices,
                                                 const Eigen::Array2f& origin) {
  std::lock_guard<std::mutex> lock(mutex_);
  local_submap_slice_ =
      MakeUniqueCairoSurfacePtr(cairo_image_surface_create_for_data(
          cairo_image_surface_get_data(slices), kCairoFormat,
          cairo_image_surface_get_width(slices),
          cairo_image_surface_get_height(slices),
          cairo_image_surface_get_stride(slices)));
  local_submap_slice_origin_ = origin;
}

// }  // namespace modules