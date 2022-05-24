-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "back_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry =false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
MAP_BUILDER.use_trajectory_builder_2d = true
-- POSE_GRAPH.overlapping_submaps_trimmer_2d = {
--     fresh_submaps_count = 3,
--     min_covered_area =3 ,
--     min_added_submaps_count = 3,
--  }

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 4,
-- }
TRAJECTORY_BUILDER_2D.use_imu_data=true
TRAJECTORY_BUILDER_2D.min_range =  0.1
TRAJECTORY_BUILDER_2D.min_z =-5
TRAJECTORY_BUILDER_2D.max_range =30

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10
TRAJECTORY_BUILDER_2D.num_accumulated_range_data =1
TRAJECTORY_BUILDER_2D.voxel_filter_size =0.1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching =true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =math.rad(10.)

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight =4
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps=true
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based=true
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight=1
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight=100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight=10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight=301.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 16
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds=5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters =0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians=math.rad(2.)
TRAJECTORY_BUILDER_2D.submaps.num_range_data=120
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type="TSDF"
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type="TSDF_INSERTER_2D"
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance=0.05
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space=true

-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability= 0.6
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability =0.4
-- TRAJECTORY_BUILDER_2D.submaps.high_resolution = 0.05
POSE_GRAPH.optimize_every_n_nodes =90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.global_constraint_search_after_n_seconds=10
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
-- fast localization
MAP_BUILDER.num_background_threads = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.global_sampling_ratio =0.001
POSE_GRAPH.constraint_builder.log_matches =true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth=8
POSE_GRAPH.constraint_builder.max_constraint_distance =10
POSE_GRAPH.optimization_problem.odometry_translation_weight=2
POSE_GRAPH.optimization_problem.odometry_rotation_weight=3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window=10
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)


-- MAP_BUILDER.use_trajectory_builder_2d = true
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- POSE_GRAPH.optimize_every_n_nodes = 90
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.99
-- POSE_GRAPH.constraint_builder.min_score= 0.99
return options
