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
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat  =false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data =2
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length= 1
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length=5
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 80

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 14
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 =0.01

-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 =5
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 =0.01
TRAJECTORY_BUILDER_3D.use_intensities=true   
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight=5
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight= 200
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 16
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.use_floor_costraint =true
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching  = false
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant=10
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.2

TRAJECTORY_BUILDER_3D.max_range=80
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based=true
-- TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.gravity_constant=-10
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_queue_duration=10
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.num_threads =16
-- TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight=200
<<<<<<< HEAD
<<<<<<< Updated upstream
-- TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight=20
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(180)
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.3
=======
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight=50
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(20)
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.8
>>>>>>> gtsamtest
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds=5
=======
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight=50
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(20)
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.8
-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds=5
>>>>>>> Stashed changes
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.2
TRAJECTORY_BUILDER_3D.submaps.low_resolution= 1

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100	
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability= 0.65
-- POSE_GRAPH.optimization_problem.fix_z_in_3d= true
POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes =90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.40
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score= 0.50
POSE_GRAPH.optimization_problem.acceleration_weight=0
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d =false
POSE_GRAPH.optimization_problem.fixed_frame_pose_use_tolerant_loss =true
POSE_GRAPH.optimization_problem.use_floor_costraint=true
POSE_GRAPH.constraint_builder.max_constraint_distance=50
POSE_GRAPH.global_sampling_ratio =0.1
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window=20
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score=0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window=60

return options