/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"
#include <thread>
#include <algorithm>
#include <cmath>
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
namespace cartographer {
namespace mapping {
namespace {

class SubmapCoverageGrid2D {
 public:
  // Aliases for documentation only (no type-safety).
  using CellId = std::pair<int64 /* x cells */, int64 /* y cells */>;
  using StoredType = std::vector<std::pair<SubmapId, common::Time>>;

  SubmapCoverageGrid2D(const MapLimits& map_limits)
      : offset_(map_limits.max()), resolution_(map_limits.resolution()) {}

  void AddPoint(const Eigen::Vector2d& point, const SubmapId& submap_id,
                const common::Time& time) {
    CellId cell_id{common::RoundToInt64((offset_(0) - point(0)) / resolution_),
                   common::RoundToInt64((offset_(1) - point(1)) / resolution_)};
    cells_[cell_id].emplace_back(submap_id, time);
  }

  const std::map<CellId, StoredType>& cells() const { return cells_; }
  bool IsInCell(const Eigen::Vector2d& point){
    CellId cell_id{common::RoundToInt64((offset_(0) - point(0)) / resolution_),
                   common::RoundToInt64((offset_(1) - point(1)) / resolution_)};
    if(cells_.count(cell_id)!=0)return true;
    return false; 
  }
  double resolution() const { return resolution_; }

 private:
  Eigen::Vector2d offset_;
  double resolution_;
  std::map<CellId, StoredType> cells_;
};

// Iterates over every cell in a submap, transforms the center of the cell to
// the global frame and then adds the submap id and the timestamp of the most
// recent range data insertion into the global grid.
std::set<SubmapId> AddSubmapsToSubmapCoverageGrid2D(
    const std::map<SubmapId, common::Time>& submap_freshness,
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    SubmapCoverageGrid2D* coverage_grid) {
  std::set<SubmapId> all_submap_ids;

  for (const auto& submap : submap_data) {
    auto freshness = submap_freshness.find(submap.id);
    if (freshness == submap_freshness.end()) continue;
    if (!submap.data.submap->insertion_finished()) continue;
    all_submap_ids.insert(submap.id);
    const Grid2D& grid =
        *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();
    // Iterate over every cell in a submap.
    Eigen::Array2i offset;
    CellLimits cell_limits;
    grid.ComputeCroppedLimits(&offset, &cell_limits);
    if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
      LOG(WARNING) << "Empty grid found in submap ID = " << submap.id;
      continue;
    }

    const transform::Rigid3d& global_frame_from_submap_frame = submap.data.pose;
    const transform::Rigid3d submap_frame_from_local_frame =
        submap.data.submap->local_pose().inverse();
    for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
      const Eigen::Array2i index = xy_index + offset;
      if (!grid.IsKnown(index)) continue;

      const transform::Rigid3d center_of_cell_in_local_frame =
          transform::Rigid3d::Translation(Eigen::Vector3d(
              grid.limits().max().x() -
                  grid.limits().resolution() * (index.y() + 0.5),
              grid.limits().max().y() -
                  grid.limits().resolution() * (index.x() + 0.5),
              0));

      const transform::Rigid2d center_of_cell_in_global_frame =
          transform::Project2D(global_frame_from_submap_frame *
                               submap_frame_from_local_frame *
                               center_of_cell_in_local_frame);
      // if ((1.0f - abs(grid.GetCorrespondenceCost(index))) < 0.2) {
      //   if (coverage_grid->IsInCell(
      //           center_of_cell_in_global_frame.translation())) {
      //     // LOG(INFO)<<"probility low in ";
      //     continue;
      //   }
      // }
      coverage_grid->AddPoint(center_of_cell_in_global_frame.translation(),
                              submap.id, freshness->second);
    }
  }
  return all_submap_ids;
}

std::map<SubmapId, common::Time> ComputeFullSubmapFreshness(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data
    ) {
  std::map<SubmapId, common::Time> submap_freshness;
  int  fack_inc_time  = 0;

  std::map<int,SubmapId,std::greater<int>> cell_coverage_num_submap;
  for (const auto& submap : submap_data) {
    // const Grid2D& grid =
    //     *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();
    // // Iterate over every cell in a submap.
    // Eigen::Array2i offset;
    // CellLimits cell_limits;
    // grid.ComputeCroppedLimits(&offset, &cell_limits);
    // if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    //   LOG(WARNING) << "Empty grid found in submap ID = " << submap.id;
    //   continue;
    // }
    // int cell_num = 0;
    // for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    //   const Eigen::Array2i index = xy_index + offset;
    //   if (!grid.IsKnown(index)) cell_num++;
    // }
    // cell_coverage_num_submap.insert({cell_num , submap.id}); 
    submap_freshness[submap.id] =
        common::Time(common::Time::duration(fack_inc_time++));
  }

  // for (const auto& submap : cell_coverage_num_submap) {
  //   submap_freshness[submap.second] =
  //       common::Time(common::Time::duration(fack_inc_time++));
  // }
  return submap_freshness;
}


// Uses intra-submap constraints and trajectory node timestamps to identify time
// of the last range data insertion to the submap.
std::map<SubmapId, common::Time> ComputeSubmapFreshness(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    const std::vector<PoseGraphInterface::Constraint>& constraints) {
  std::map<SubmapId, common::Time> submap_freshness;

  // Find the node with the largest NodeId per SubmapId.
  std::map<SubmapId, NodeId> submap_to_latest_node;
  for (const PoseGraphInterface::Constraint& constraint : constraints) {
    if (constraint.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      continue;
    }

    auto submap_to_node = submap_to_latest_node.find(constraint.submap_id);
    if (submap_to_node == submap_to_latest_node.end()) {
      submap_to_latest_node.insert(
          std::make_pair(constraint.submap_id, constraint.node_id));
      continue;
    }
    submap_to_node->second =
        std::max(submap_to_node->second, constraint.node_id);
  }

  // Find timestamp of every latest node.
  for (const auto& submap_id_to_node_id : submap_to_latest_node) {
    auto submap_data_item = submap_data.find(submap_id_to_node_id.first);
    if (submap_data_item == submap_data.end()) {
      LOG(WARNING) << "Intra-submap constraint between SubmapID = "
                   << submap_id_to_node_id.first << " and NodeID "
                   << submap_id_to_node_id.second << " is missing submap data";
      continue;
    }
    auto latest_node_id = trajectory_nodes.find(submap_id_to_node_id.second);
    if (latest_node_id == trajectory_nodes.end()) continue;
    submap_freshness[submap_data_item->id] = latest_node_id->data.time();
  }
  
  return submap_freshness;
}

// Returns IDs of submaps that have less than 'min_covered_cells_count' cells
// not overlapped by at least 'fresh_submaps_count' submaps.
std::vector<SubmapId> FindSubmapIdsToTrim(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    const SubmapCoverageGrid2D& coverage_grid,
    const std::set<SubmapId>& all_submap_ids, uint16 fresh_submaps_count,
    uint16 min_covered_cells_count,
    float long_way_area) {
      std::vector<SubmapId> long_way_submap;
 for (const auto& submap : submap_data) {
    const Grid2D& grid =
        *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();
    // Iterate over every cell in a submap.
    Eigen::Array2i offset;
    CellLimits cell_limits;
    grid.ComputeCroppedLimits(&offset, &cell_limits);
    if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
      LOG(WARNING) << "Empty grid found in submap ID = " << submap.id;
      continue;
    }
    int max_num_cell = cell_limits.num_x_cells * cell_limits.num_y_cells;
    if (max_num_cell * std::pow(grid.limits().resolution(),2) >long_way_area ) {
      long_way_submap.push_back(submap.id);
    }
  }


  std::map<SubmapId, uint16> submap_to_covered_cells_count;
  for (const auto& cell : coverage_grid.cells()) {
    std::vector<std::pair<SubmapId, common::Time>> submaps_per_cell(
        cell.second);
    for (auto it = submaps_per_cell.begin(); it != submaps_per_cell.end();) {
      if (std::find(long_way_submap.begin(), long_way_submap.end(),
                    it->first) != long_way_submap.end()) {
        it = submaps_per_cell.erase(it);
        continue;
      }
      ++it;
    }
    // In case there are several submaps covering the cell, only the freshest
    // submaps are kept.
    if (submaps_per_cell.size() > fresh_submaps_count) {
      // Sort by time in descending order.

      std::sort(submaps_per_cell.begin(), submaps_per_cell.end(),
                [](const std::pair<SubmapId, common::Time>& left,
                   const std::pair<SubmapId, common::Time>& right) {
                  return left.second > right.second;
                });
      submaps_per_cell.erase(submaps_per_cell.begin() + fresh_submaps_count,
                             submaps_per_cell.end());
    }
    for (const std::pair<SubmapId, common::Time>& submap : submaps_per_cell) {
      ++submap_to_covered_cells_count[submap.first];
    }
  }
  std::vector<SubmapId> submap_ids_to_keep;
  for (const auto& id_to_cells_count : submap_to_covered_cells_count) {
    if (id_to_cells_count.second < min_covered_cells_count) continue;
    submap_ids_to_keep.push_back(id_to_cells_count.first);
  }

  DCHECK(std::is_sorted(submap_ids_to_keep.begin(), submap_ids_to_keep.end()));
  std::vector<SubmapId> result;
  std::set_difference(all_submap_ids.begin(), all_submap_ids.end(),
                      submap_ids_to_keep.begin(), submap_ids_to_keep.end(),
                      std::back_inserter(result));
  return result;
}



}  // namespace

void OverlappingSubmapsTrimmer2D::Trim(Trimmable* pose_graph) {

  const auto submap_data = pose_graph->GetOptimizedSubmapData();
  if (submap_data.size() - current_submap_count_ <= min_added_submaps_count_) {
    return;
  }
  auto trajectories_state = pose_graph->GetTrajectoryStates();
  std::set<int> frozen_trajectories;
  LOG(INFO) << "trajectories_state.size  = " << trajectories_state.size();
  if (trajectories_state.size() > 1) {
    for (const auto& it : trajectories_state) {
      if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
        frozen_trajectories.insert(it.first);
        LOG(INFO) << "frozen_trajectory is " << it.first;
      }
    }
  }
  std::map<SubmapId, common::Time> frozen_submap;
  for (auto submap : submap_data) {
    if (frozen_trajectories.count(submap.id.trajectory_id) != 0) {
      frozen_submap.insert({submap.id, common::Time::max()});
    }
  }
  if (submap_data.size() - frozen_submap.size() <= min_added_submaps_count_) {
    return;
  }
  LOG(INFO) << "forzen_submap is " << frozen_submap.size();
  const MapLimits first_submap_map_limits =
      std::static_pointer_cast<const Submap2D>(submap_data.begin()->data.submap)
          ->grid()
          ->limits();
  SubmapCoverageGrid2D coverage_grid(first_submap_map_limits);
  std::map<SubmapId, common::Time> submap_freshness =
      ComputeSubmapFreshness(submap_data, pose_graph->GetTrajectoryNodes(),
                             pose_graph->GetConstraints());
///
  LOG(INFO)<<"submap_frashnees size :"<<submap_freshness.size();
  std::map<SubmapId, common::Time> submap_freshness_and_frozen{
      frozen_submap.begin(), frozen_submap.end()};
  submap_freshness_and_frozen.insert(submap_freshness.begin(),
                                     submap_freshness.end());
  submap_freshness = std::move(submap_freshness_and_frozen);
  LOG(INFO)<<"submap_frashnees_frozen size :"<<submap_freshness.size();

  if (full_ovelapping_) {
    submap_freshness = ComputeFullSubmapFreshness(submap_data);
  }
  //
  const std::set<SubmapId> all_submap_ids = AddSubmapsToSubmapCoverageGrid2D(
      submap_freshness, submap_data, &coverage_grid);

  const std::vector<SubmapId> submap_ids_to_remove = FindSubmapIdsToTrim(
      submap_data, coverage_grid, all_submap_ids, fresh_submaps_count_,
      min_covered_area_ / common::Pow2(coverage_grid.resolution()),
      long_way_area_);
  current_submap_count_ = submap_data.size() - submap_ids_to_remove.size();

  auto Trim = [&]() {
    for (const SubmapId& id : submap_ids_to_remove) {
      std::ostringstream info;
      info << " trim submap trajector:" << id.trajectory_id
           << " with submap index  " << id.submap_index;
      if (frozen_submap.count(id) != 0) {
        info << " submap fronzened ,wont trimed";
      } else {
        pose_graph->TrimSubmap(id);
      }
      LOG(INFO) << info.str();
    }
  };
  auto trimeble = dynamic_cast<PoseGraph2D::TrimmingHandle*>(pose_graph);
  if (trimeble != nullptr) {
    absl::MutexLock locker(&trimeble->parent_->mutex_);
    LOG(INFO) << "pose graph trimmer..";
    Trim();
    //
  } else {
    LOG(INFO) << "merg trimmer..";
    Trim();
  }
  // try {
  //   for (const SubmapId& id : submap_ids_to_remove) {
  //     //
  //     std::ostringstream info;
  //     info << " trim submap trajector:" << id.trajectory_id
  //          << " with submap index  " << id.submap_index;

  //     LOG(INFO) << "trajectory trimmer..";
  //     if (frozen_submap.count(id) != 0) {
  //       info << " submap fronzened ,wont trimed";
  //     } else {
  //       LOG(INFO) << "trajectory trimmer..";
  //       pose_graph->TrimSubmap(id);
  //     }
  //     LOG(INFO) << info.str();
  //     //
  //   }
  // } catch (std::out_of_range) {
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  //   LOG(INFO) << "catch exception............................... ";
  // }
  }

}  // namespace mapping
}  // namespace cartographer
