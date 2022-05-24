/*
 * Copyright 2016 The Cartographer Authors
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
#include "std_msgs/Float32.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include <vector>
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "submap_process.h"
#include "ros/ros.h"
#include <tuple>
#include "gflags/gflags.h"
#include <iostream>
#include <fstream>
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/merge_trajectory.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "msg_conversion.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/transform/transform.h"
#include "mutex"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "acml_location.h"
#include <condition_variable>
#include <tf/transform_listener.h>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PoseArray.h"
#include "cartographer_ros_msgs/Relocation.h"
#include "std_srvs/Empty.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include <future>
#include "cartographer/mapping/internal/2d/local_globel_pose_fusion.h"
// #include "cartographer/mapping/internal/3d/scan_matching/plane_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/grid_interface.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Vector3.h"
#include "cartographer_ros_msgs/score_client.h"


DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
DEFINE_int32(num_point_cloud,10,"num_point_cloud");


namespace cartographer_ros {
namespace {
using namespace cartographer::mapping;
using namespace cartographer;
using cartographer::mapping::Grid2D;
using cartographer::mapping::MapById;
using cartographer::mapping::NodeId;
using cartographer::mapping::PoseGraphInterface;
using cartographer::mapping::Submap2D;
using cartographer::mapping::SubmapId;
using cartographer::sensor::PointCloud;
using cartographer::transform::Rigid2d;
using namespace cartographer::mapping::scan_matching;

cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherTestOptions2D(
    const int branch_and_bound_depth) {
  auto parameter_dictionary = cartographer::common::MakeDictionary(
      R"text(
      return {
         linear_search_window = 3.,
         angular_search_window = math.rad(180.),
         branch_and_bound_depth = )text" +
      std::to_string(branch_and_bound_depth) + "}");
  return CreateFastCorrelativeScanMatcherOptions2D(parameter_dictionary.get());
}
cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherTestOptions2D(const int branch_and_bound_depth,
                                              int linear_search_window) {
  auto parameter_dictionary = cartographer::common::MakeDictionary(
      "return {"
      "branch_and_bound_depth  = " +
       std::to_string(branch_and_bound_depth)+
       ","
      "angular_search_window = math.rad(180.),"
      "linear_search_window   = "+
      std::to_string(linear_search_window)  +"}");
  return CreateFastCorrelativeScanMatcherOptions2D(parameter_dictionary.get());
}

cartographer::sensor::proto::AdaptiveVoxelFilterOptions
CreateVoxelFilterOptions() {
  auto voxel_filter_parameter_dictionary =
      cartographer::common::MakeDictionary(R"text(
          return {
            max_length = 5,
            min_num_points = 400,
            max_range = 100,
          })text");
  return cartographer::sensor::CreateAdaptiveVoxelFilterOptions(
      voxel_filter_parameter_dictionary.get());
}

cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
CreatCeresScanMatchingOption() {
  auto ceres_parameter_dictionary = cartographer::common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 10.,
          translation_weight = 5.,
          rotation_weight = 30.,
          ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 20,
            num_threads = 2,
          },
        })text");
  return cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(
      ceres_parameter_dictionary.get());
}
cartographer::sensor::PointCloud point_clouds;
bool ReciveFlag  = false;
std::mutex mutx;
bool ReciveFlagAcml =  false;
std::condition_variable condition;
std::mutex acml_mutex;
bool kill_thread = false;
cartographer::sensor::PointCloud point_clouds_acml;

transform::Rigid2d robot_pose;
std::condition_variable condition_pose;
std::mutex pose_mutex;
bool pose_recived = false;

void termin_out(int sig) {
  ros::shutdown();
  kill_thread = true;
  ReciveFlagAcml   =  true;
  condition.notify_all();
  condition_pose.notify_all();

}
//

cartographer::sensor::PointCloud match_pcl;
struct LaserFrame {
  std::vector<float> ranges;
  uint64_t system_time_stamp;
};

bool set_match_scan = false;
LaserFrame scan_to_be_matched;
 bool AreaMatchScanToNodes(
     const LaserFrame& laser,
     std::vector<::cartographer::transform::Rigid3d>& target_poses,
     ::cartographer::sensor::PointCloud& filtered_pcl);
//Test

uint8_t histogram_size = 32;
 cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                cartographer::mapping::TrajectoryNode>
     trajectory_nodes;
  struct ScanHistogram {
    std::vector<float> data;
    std::vector<int> histogram;
    ::cartographer::transform::Rigid3d pose;
  };

  std::ostream& operator<<(std::ostream& out, const ScanHistogram histogram) {
    out<<"---------------------------\n";
    for(auto size :histogram.histogram){
      std::string starts(size,'*');
      out<<starts<<"\n";
    }
    out<<"---------------------------\n";
    out<<std::endl;
    return out;
  }
  std::ostream& operator<<(
      std::ostream& out, const std::tuple<ScanHistogram, ScanHistogram,double>& hist) {
    auto [ls,rhs,score] = hist;
    // const ScanHistogram& ls = hist.;
    // const ScanHistogram& rhs = hist;
    out<<"-------------start--------------\n";
    out<<"----------score : "<<score<<"\n";
    for(int i = 0;i< ls.histogram.size();i++){

    // for(auto size :ls.histogram){
      std::string starts(ls.histogram[i],'*');
      out<<starts;
      std::string black(40-ls.histogram[i],' ');
      out<<black;
      out<<std::string(rhs.histogram[i],'*');
      out<<"\n";
    }
    
    out<<"---------------------------\n";
    out << std::endl;
    out << std::endl;
    out << std::endl;
    out << std::endl;
    out << std::endl;
    out << std::endl;

    return out;
  }

  struct ScorePose {
    double score;
    ::cartographer::transform::Rigid3d pose;
    ScorePose(double data, ::cartographer::transform::Rigid3d pos) : score(data), pose(pos) {}
  };

Eigen::Quaterniond ToEigenQ(const double theta) {
  auto q1 = sin(theta / 2) * sin(0) * sin(0) + cos(theta / 2) * cos(0) * cos(0);
  auto q2 =
      -sin(theta / 2) * sin(0) * cos(0) + sin(0) * cos(theta / 2) * cos(0);
  auto q3 = sin(theta / 2) * sin(0) * cos(0) + sin(0) * cos(theta / 2) * cos(0);
  auto q4 = sin(theta / 2) * cos(0) * cos(0) - sin(0) * sin(0) * cos(theta / 2);
  return Eigen::Quaterniond(q1, q2, q3, q4);
}

void GetHistogramOfNodes(std::vector<ScanHistogram>& nodes_histogram) {
  auto sensor_to_track = ::cartographer::transform::Rigid3d{
      {0.0255, 0, 0}, ToEigenQ(100 * M_PI / 180)};
  for (auto node : trajectory_nodes) {
  ScanHistogram scan;
  scan.histogram.resize(histogram_size);
  for (size_t i = 0; i < scan.histogram.size(); i++) {
    scan.histogram.at(i) = 0;
  }
  // LOG(WARNING) << "SCAN HISTOGRAM SIZE : "<<scan.histogram.size();
  auto pcl = node.data.constant_data->filtered_gravity_aligned_point_cloud;
  if (pcl.size() == 0) {
    continue;
  }
  scan.pose = node.data.global_pose;
  auto local_pose = node.data.constant_data->local_pose;
  Eigen::Vector3f origin_in_local = local_pose.cast<float>() * sensor_to_track.translation().cast<float>();

  // auto lidar_to_local = local_pose * sensor_to_track;
  // auto origin_in_local = lidar_to_local.translation();

  // auto points_cloud_in_local =
  //     cartographer::sensor::TransformPointCloud(pcl, lidar_to_local);


  for (auto point : pcl) {
    const Eigen::Vector3f delta = point.position;
    const float dis = delta.norm();  
    // LOG(WARNING)<<"dis: " <<dis;    
    int index = dis / 0.25;
    // LOG(WARNING)<<"GetHistogramOfNodes index : " <<index;
    // LOG(WARNING) << "-----HISTOGRAM SIZE : "<<scan.histogram.size();
    if (index < scan.histogram.size()) {
      // (scan.histogram.at(index))++;
      scan.histogram.at(index) = scan.histogram.at(index) + 1;
    }
    scan.data.push_back(dis);
  }
  nodes_histogram.push_back(scan);
  }
}

ScanHistogram ConvertLaserToScanHistogram(const LaserFrame& laser) {
  ScanHistogram scan_for_match;
  scan_for_match.histogram.reserve(histogram_size);
  for (size_t i = 0; i < histogram_size; i++) {
    scan_for_match.histogram.push_back(0);
  }
  LOG(WARNING)<<"scan_for_match.histogram SIZE: "<<scan_for_match.histogram.size();
  LOG(WARNING)<<"laser size:"<<laser.ranges.size();

  for (size_t k = 0; k < laser.ranges.size(); k++) {
    int index = laser.ranges.at(k) / 0.25;
    if (index < scan_for_match.histogram.size()) {
      (scan_for_match.histogram.at(index))++;
    }
  }
  return scan_for_match;
}

void FilterPoses(const std::vector<::cartographer::transform::Rigid3d>& poses,
                       ::cartographer::sensor::PointCloud& filtered_pcl) {
  ::cartographer::sensor::PointCloud pcl;
  for (auto pose : poses) {
    ::cartographer::sensor::RangefinderPoint result;
    result.position = pose.translation().cast<float>();
    pcl.push_back(result);
  }
  filtered_pcl = sensor::VoxelFilter(pcl, 0.1);
  for (auto pcl : filtered_pcl) {
    LOG(WARNING) <<"PCL : " <<pcl.position;
  }
}

bool GetTargetPoses(const std::vector<ScanHistogram>& laser_histogram,
                          const ScanHistogram& match_scan,
                          std::vector<::cartographer::transform::Rigid3d>& target_poses,
                          ::cartographer::sensor::PointCloud& filtered_pcl) {
  LOG(WARNING) <<"GetTargetPoses";
  const int threshold_num = 3;
  const float threshold_score = 0.5;
  std::vector<ScorePose> score_poses;
  LOG(WARNING) <<"laser_histogram size : "<<laser_histogram.size();
  score_poses.reserve(laser_histogram.size());
  for (auto histogram : laser_histogram) {
    // LOG(WARNING)<<"histogram.size() :" <<histogram.histogram.size()<<match_scan.histogram.size();
    if (histogram.histogram.size() != match_scan.histogram.size()) {
      return false;
    }
    double score = 0;
    // LOG(WARNING)<<" histogram.histogram.size()"<< histogram.histogram.size();

    int sum_histogram = 0;
    for (size_t i = 0; i < histogram.histogram.size(); i++) {
      sum_histogram += (histogram.histogram[i] + match_scan.histogram[i]);
      score +=abs((histogram.histogram.at(i) - match_scan.histogram.at(i)));
    }
    score = (1.0 - score / sum_histogram);
    // LOG(WARNING) <<"score: "<<score;
    score_poses.push_back(ScorePose(score, histogram.pose));
    // std::cout << std::tuple<ScanHistogram, ScanHistogram, float>(match_scan, histogram, score);

  }

  LOG(WARNING)<<"TEST";
  std::sort(score_poses.begin(), score_poses.end(),
            [](const ScorePose& lhs, const ScorePose& rhs) -> bool {
              return lhs.score > rhs.score;
            });
  for (size_t k = 0; k < score_poses.size(); k++) {
    // LOG(WARNING) <<"-------------score pose: "<<score_poses.at(k).score << "pose :"<<score_poses.at(k).pose;
    
    if ((k < threshold_num)/* && (score_poses.at(k).score > threshold_score)*/) {
      std::cout << std::tuple<ScanHistogram, ScanHistogram, double>(
          match_scan, laser_histogram[k], score_poses[k].score);

      target_poses.push_back(score_poses.at(k).pose);
      LOG(WARNING) <<"score pose : "<<score_poses.at(k).pose;
    }
  }
  FilterPoses(target_poses, filtered_pcl);
  return true;
}


bool AreaMatchScanToNodes(
    const LaserFrame& laser, std::vector<::cartographer::transform::Rigid3d>& target_poses,
    ::cartographer::sensor::PointCloud& filtered_pcl) {
  LOG(INFO) <<"AreaMatchScanToNodes";
  std::vector<ScanHistogram> nodes_histogram;
  GetHistogramOfNodes(nodes_histogram);
  ScanHistogram target_scan = ConvertLaserToScanHistogram(laser);
  std::cout<<target_scan;
  if (!GetTargetPoses(nodes_histogram, target_scan, target_poses, filtered_pcl)) {
    return false;
  }
  return true;
}

// void HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& msg) 
// {
//   cartographer::sensor::PointCloudWithIntensities point_cloud;
//   cartographer::common::Time time;
//   std::lock_guard<std::mutex> lock(acml_mutex);
//   point_clouds_acml = cartographer::sensor::PointCloud();
//   std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
//   for (auto point : point_cloud.points) {
//     point_clouds_acml.push_back({point.position});
//   }
//   // point_clouds_acml =  pocondition



void HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  cartographer::sensor::PointCloudWithIntensities point_cloud;
  cartographer::common::Time time;
  std::unique_lock<std::mutex> lock(mutx);
  point_clouds = cartographer::sensor::PointCloud();

  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);

  sensor::PointCloud  point_cloudes ;
  for (auto point : point_cloud.points) {
    point_cloudes.push_back({point.position});
  }

  point_cloudes = sensor::VoxelFilter(point_cloudes, 0.1);
  
  //pcl to laserscan
  if (!set_match_scan) {
    for (auto point : point_cloudes) {
      scan_to_be_matched.ranges.push_back(point.position.norm());
    }
    set_match_scan = true;
    LOG(WARNING) <<"scan_to_be_matched SIZE: " << scan_to_be_matched.ranges.size();
  }

  for (auto point : point_cloud.points) {
    point_clouds.push_back({point.position});
    // double dist = point.position.();
  }
  // LOG(INFO)<<"HandleLaserScanMessage"<<std::endl;
  ReciveFlag = true;

  // ReciveFlagAcml =  true; 
  // condition.notify_all();
  // point_cloud = TransformPointCloud(
  //     point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf{
  //                       1.745329222, Eigen::Vector3f{0, 0, 1}}));  



  // auto voxel_options = CreateVoxelFilterOptions();
  // point_clouds = cartographer::sensor::AdaptiveVoxelFilter(voxel_options)
  //                   .Filter(point_clouds);
}

/*----------------------------------------*/

void HandlePointCloudScanMessage(const sensor_msgs::PointCloud2::ConstPtr msg) 
{
  std::lock_guard<std::mutex> lock(acml_mutex);
  point_clouds = cartographer::sensor::PointCloud();
  cartographer::sensor::PointCloudWithIntensities point_cloud;
  cartographer::common::Time time;

  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  point_clouds_acml = cartographer::sensor::PointCloud();
  // std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  for (auto point : point_cloud.points) {
    point_clouds_acml.push_back({point.position});
  }

  ReciveFlagAcml = true;
  condition.notify_all();

  // static  int time_cout  = 0;
  //  static  int time_cout1  = 0;

  // if (++time_cout >= 4) {
  //   time_cout = 0;
  // std::unique_lock<std::mutex> lock(mutx);
  // cartographer::sensor::PointCloudWithIntensities point_cloud;
  // cartographer::common::Time time;
  // std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  // for (auto point : point_cloud.points) {
  //   point_clouds.push_back({point.position});
  // }
  // LOG(INFO)<<"HandlePointCloudScanMessage";

  // // if(time_cout1++ >= FLAGS_num_point_cloud){

  // ReciveFlag = true;
  // //   // point_clouds = cartographer::sensor::PointCloud();
  // //   time_cout1 = 0;  
  // // }


  // }

  // auto voxel_options = CreateVoxelFilterOptions();
  // point_clouds = cartographer::sensor::AdaptiveVoxelFilter(voxel_options)
  //                   .Filter(point_clouds);
}

void SubScribleBaseTf() {
  ros::NodeHandle ph("");
  tf2_ros::Buffer tf_buffer{::ros::Duration(1)};
  tf2_ros::TransformListener listener(tf_buffer, ph, true);
  geometry_msgs::TransformStamped transform;
  uint64_t oldtimestamp = 0;
  ros::Rate rate(100);
  while (!kill_thread) {
    try {
      transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
      uint64_t timestamp = transform.header.stamp.toNSec();
      if (oldtimestamp != timestamp) {
        oldtimestamp = timestamp;
        tf::Transform car_to_base;
        tf::transformMsgToTF(transform.transform, car_to_base);
        float x = car_to_base.getOrigin().x();
        float y = car_to_base.getOrigin().y();
        double roll, pitch, yaw;
        tf::Matrix3x3(car_to_base.getRotation()).getRPY(roll, pitch, yaw);
        std::lock_guard<std::mutex> lock(pose_mutex);
        robot_pose = transform::Rigid2d({x, y}, yaw);
        condition_pose.notify_all();
      }
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
}

bool odom_flag = false;
nav_msgs::Odometry odom_acml;
std::mutex odom_mutex;
void HandleOdomMessage(const nav_msgs::Odometry::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(odom_mutex);
  odom_acml = *msg;
  odom_flag  =true;
}
void AcmlProcess(AmclLocaton* acml) {
  ros::NodeHandle ph("");
  ros::Publisher partic_publish =
      ph.advertise<geometry_msgs::PoseArray>("partics", 1);
  LOG(INFO) << "start AcmlProcess";
  while (!kill_thread) {
 sensor::PointCloud point_clouds;
    {
      std::unique_lock<std::mutex> lock(acml_mutex);
      condition.wait(lock, []() { return ReciveFlagAcml; });
      ReciveFlagAcml = false;

      point_clouds = TransformPointCloud(
          point_clouds_acml, transform::Rigid3f::Rotation(Eigen::AngleAxisf{
                                 1.745329222, Eigen::Vector3f{0, 0, 1}}));
    }
    point_clouds_acml = sensor::VoxelFilter(point_clouds_acml, 0.1);
    {
      std::unique_lock<std::mutex> lock(pose_mutex);
      condition_pose.wait(lock);
    }
    auto particals = (*acml)(point_clouds, robot_pose);
    // transform::Rigid2d odom_pose;
    // while (!odom_flag) {
    // }
    // {
    //   std::unique_lock<std::mutex> lock(odom_mutex);
    //   odom_flag = false;
    //   tf::Quaternion q(
    //       odom_acml.pose.pose.orientation.x, odom_acml.pose.pose.orientation.y,
    //       odom_acml.pose.pose.orientation.z, odom_acml.pose.pose.orientation.w);
    //   double roll, pitch, yaw;
    //   tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //   odom_pose = transform::Rigid2d(
    //       {odom_acml.pose.pose.position.x, odom_acml.pose.pose.position.y},
    //       yaw);
    // }

    // auto particals = (*acml)(point_clouds, odom_pose);
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();
    for (const auto& partic : particals) {
      geometry_msgs::Pose pose;
      pose.position.x = partic.x;
      pose.position.y = partic.y;
      pose.position.z = 0;
      auto quaternion = tf::Quaternion({0, 0, 1}, partic.theta);
      geometry_msgs::Quaternion q;
      q.x = quaternion.x();
      q.y = quaternion.y();
      q.z = quaternion.z();
      q.w = quaternion.w();
      pose.orientation = q;
      pose_array.poses.push_back(pose);
    }
    partic_publish.publish(pose_array);
    static int sleep_time = 0;
    if (sleep_time++ > 2) {
      sleep(100);
    }
  }

}

void FastCorrlationScanMatch(const MapById<SubmapId, PoseGraphInterface::SubmapData>&submap_datas){
  CeresScanMatcher2D ceres_scan_matcher(CreatCeresScanMatchingOption());
  const auto options = CreateFastCorrelativeScanMatcherTestOptions2D(8);
  std::vector<std::pair<float, Rigid2d>> pose_estimates;
  cartographer::sensor::PointCloud point_cloud;
  {
    std::unique_lock<std::mutex> lock(mutx);
    point_cloud = point_clouds;
  }
  LOG(INFO)<<"FastCorrlationScanMatch";


  // auto voxel_options = CreateVoxelFilterOptions();
  //  // point_cloud = cartographer::sensor::AdaptiveVoxelFilter(point_cloud,voxel_options);
  // point_cloud =   sensor::VoxelFilter(point_cloud, 0.1);

  // point_cloud = TransformPointCloud(
  //     point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf{
  //                       1.745329222, Eigen::Vector3f{0, 0, 1}}));
  for (auto submap : submap_datas) {
    const Grid2D& grid =
        *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();

    Rigid2d pose_estimate;
    float score;
    constexpr float kMinScore = 0.2f;
    FastCorrelativeScanMatcher2D fast_correlative_scan_matcher(grid, options);
    fast_correlative_scan_matcher.MatchFullSubmap(point_cloud, kMinScore,
                                                  &score, &pose_estimate);

  

    pose_estimate = Rigid2d{
        pose_estimate.translation(),
        Rigid2d::Rotation2D(pose_estimate.rotation().angle())};

    ceres::Solver::Summary unused_summary;
    ceres_scan_matcher.Match(pose_estimate.translation(), pose_estimate,
                             point_cloud, grid, &pose_estimate,
                             &unused_summary);
    pose_estimates.push_back({score, pose_estimate});




    LOG(WARNING) << "trajectory id: " << submap.id.trajectory_id << "submap id "
              << "pose  "<<submap.id.submap_index << pose_estimate << " score : " << score;
  }
  std::sort(pose_estimates.begin(), pose_estimates.end(),
            [](const std::pair<float, Rigid2d> lhs,
               const std::pair<float, Rigid2d> rhs) {
              return lhs.first > rhs.first;
            });
   LOG(WARNING)<<"pose_estimates "<<pose_estimates.begin()->first;



  cartographer::mapping::AddPreLocationPose(
      cartographer::transform::Embed3D(pose_estimates.begin()->second));
}
 ros::Publisher mark_target_pub;
ros::Publisher pose_score_pub;
ros::ServiceClient pose_score_client;

 void PubTarget(const sensor::PointCloud& points)
 {
   visualization_msgs::Marker mark;
   mark.header.frame_id = "map";
   std::string namesring = "target";
   mark.ns = namesring.c_str();
   mark.header.stamp = ::ros::Time::now();
   mark.id = 0;
   mark.action = visualization_msgs::Marker::ADD;
   mark.type = visualization_msgs::Marker::POINTS;
   // mark.type = visualization_msgs::Marker::ARROW;

   mark.lifetime = ros::Duration(0);
   mark.scale.x = 0.3;
   mark.scale.y = 0.3;
   mark.scale.z = 0.3;

   mark.color.r = 1;  // 1.0;
   mark.color.a = 1;  // ran(e);
   mark.color.g = 0;  //(mark_id / sizeofils);
   mark.color.b = 0;  //(sizeofils- mark_id) / sizeofils;

   mark.pose.orientation.w = 1.0;

   for (auto p : points) {
     geometry_msgs::Point point;
     point.x = p.position.x();
     point.y = p.position.y();
     point.z = 0;
     mark.points.push_back(point);
   }
 // visualization_msgs::MarkerArray marks;
  //marks.markers.push_back(mark);
  mark_target_pub.publish(mark);

 }

// std::vector<Eigen::vector3f> FastCorrlationScanMatchLowResulution(
//      const MapById<SubmapId, PoseGraphInterface::SubmapData>& one_submap) {

bool start_relocation = false;
transform::Rigid2d init_pose;
void HandleInitPoseMessage(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  init_pose = (transform::Rigid2d::Translation(
    {msg->pose.position.x, msg->pose.position.y}
  ));

  start_relocation = true;
}

//  }
 transform::Rigid3d  FastCorrlationScanMatchWithOneSubmapAddMultySubmap(
     const MapById<SubmapId, PoseGraphInterface::SubmapData>& one_submap,
     const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_datas) {
   CeresScanMatcher2D ceres_scan_matcher(CreatCeresScanMatchingOption());
   std::vector<std::pair<float, Rigid2d>> pose_estimates;
    cartographer::sensor::PointCloud point_cloud;
  //  {
  //    std::unique_lock<std::mutex> lock(mutx);
  //    point_cloud = point_clouds;
  //  }
    LOG(INFO) << "FastCorrlationScanMatch";


  std::vector<sensor::PointCloud> accumlate_point_cloud;
  int num = 0;
  do {
    {
      std::unique_lock<std::mutex> lock(acml_mutex);
      condition.wait(lock, []() { return ReciveFlagAcml; });
      ReciveFlagAcml = false;
      // point_cloud = TransformPointCloud(
      //     point_clouds_acml, transform::Rigid3f::Rotation(Eigen::AngleAxisf{
      //                            1.745329222, Eigen::Vector3f{0, 0, 1}}));
      point_cloud = point_clouds_acml;
    }
    {
      std::unique_lock<std::mutex> lock(pose_mutex);
      condition_pose.wait(lock);
    }
    // point_cloud = TransformPointCloud(
    //     point_cloud, transform::Embed3D(robot_pose).cast<float>());
    accumlate_point_cloud.push_back(point_cloud);
    LOG(INFO) << "accumlate_point_cloud size" << accumlate_point_cloud.size();
  // } while (num++ < FLAGS_num_point_cloud);
  } while (num++ <20);
  //

  sensor::PointCloud point_clouds_acc;
  for (auto& point_cloud : accumlate_point_cloud) {
    for (auto point : point_cloud) {
      point_clouds_acc.push_back({point.position});
    }
  }

  //   transform::Rigid2d init_pose;
  //   {
  //     std::unique_lock<std::mutex> lock(pose_mutex);
  //     condition_pose.wait(lock);
  //     init_pose = robot_pose;
  //   }



  // auto voxel_options = CreateVoxelFilterOptions();
   // point_cloud =
   // cartographer::sensor::AdaptiveVoxelFilter(point_cloud,voxel_options);
    point_clouds_acc= sensor::VoxelFilter(point_clouds_acc, 0.1);

   // point_cloud = TransformPointCloud(
   //     point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf{
   //                       1.745329222, Eigen::Vector3f{0, 0, 1}}));

   // std::vector<std::pair<float, Rigid2d>> pose_point_cloud;
   cartographer::sensor::PointCloud pose_point_cloud;

   int i = 0;
   for (auto submap : submap_datas) {
     Eigen::Vector3f position;
     position.x() = submap.data.pose.translation().x();
     position.y() = submap.data.pose.translation().y();
     position.z() = 0;
     pose_point_cloud.push_back({position});
   }


  //  std::vector<::cartographer::transform::Rigid3d> target_poses;
  //   ::cartographer::sensor::PointCloud filtered_pcl;
  //   AreaMatchScanToNodes(scan_to_be_matched, target_poses, filtered_pcl);

   pose_point_cloud = sensor::VoxelFilter(pose_point_cloud, 40);
  //  for (auto target_pose : target_poses) {
  //    pose_point_cloud.push_back({Eigen::Vector3f{
  //        target_pose.translation().x(), target_pose.tra slation().y(), 0}});
  //   }

  //  pose_point_cloud = sensor::VoxelFilter(pose_point_cloud, 5);

   PubTarget(pose_point_cloud);
   const Grid2D& grid =
       *std::static_pointer_cast<const Submap2D>(one_submap.begin()->data.submap)
            ->grid();
   const auto options = CreateFastCorrelativeScanMatcherTestOptions2D(7, 10);

   FastCorrelativeScanMatcher2D fast_correlative_scan_matcher(grid, options);

  //  for (auto pose : pose_point_cloud) {
  //    Rigid2d pose_estimate;
  //    float score;

  //    constexpr float kMinScore = 0.2f;
  //     transform::Rigid2d init_pose =
  //         transform::Rigid2d::Translation(Eigen::Vector2d{pose.position.x(),pose.position.y()});

  //     fast_correlative_scan_matcher.Match(init_pose, point_cloud, kMinScore,
  //                                         &score, &pose_estimate);

  //     pose_estimate =
  //         Rigid2d{pose_estimate.translation(),
  //                 Rigid2d::Rotation2D(pose_estimate.rotation().angle())};

  //     // ceres::Solver::Summary unused_summary;
  //     // ceres_scan_matcher.Match(pose_estimate.translation(), pose_estimate,
  //     //                          point_cloud, grid, &pose_estimate,
  //     //                          &unused_summary);
  //     pose_estimates.push_back({score, pose_estimate});

  //     LOG(WARNING) << "index " << i
  //                  << "pose "<<pose.position
  //                  << " " 
  //         <<"pose_estimate :" << pose_estimate << " score : " << score;
  //     i++;
  //  }
  std::vector<std::pair<float, transform::Rigid2d>> init_pose_with_score;
  init_pose_with_score.push_back(
        {0.4, init_pose});
  // for (auto pose : pose_point_cloud) {
    // init_pose_with_score.push_back(
    //     {0.4, transform::Rigid2d({0,0}, 0.0)});
        // {0.4, transform::Rigid2d({pose.position.x(), pose.position.y()}, 0.0)});
  // }
  LOG(INFO) << init_pose_with_score.size();
  float score = 0;
  pose_estimates = fast_correlative_scan_matcher.MatchWithTargets(
      init_pose_with_score, point_clouds_acc, 10);

  std::sort(pose_estimates.begin(), pose_estimates.end(),
            [](const std::pair<float, Rigid2d> lhs,
               const std::pair<float, Rigid2d> rhs) {
              return lhs.first > rhs.first;
            });



  LOG(WARNING) << "pose_estimates " << pose_estimates.begin()->first;

  for (auto& pose_estimate : pose_estimates) {
    ceres::Solver::Summary unused_summary;
    ceres_scan_matcher.Match(pose_estimate.second.translation(),
                             pose_estimate.second, point_cloud, grid,
                             &pose_estimate.second, &unused_summary);
    // pose_estimates.push_back({score, pose_estimate})
  }
  std::sort(pose_estimates.begin(), pose_estimates.end(),
            [](const std::pair<float, Rigid2d> lhs,
               const std::pair<float, Rigid2d> rhs) {
              return lhs.first > rhs.first;
            });



  LOG(WARNING) << "pose_estimates " << pose_estimates.begin()->first<<pose_estimates.begin()->second;
  return cartographer::transform::Embed3D(pose_estimates.begin()->second);

  // cartographer::mapping::AddPreLocationPose(
  //     cartographer::transform::Embed3D(pose_estimates.begin()->second));



 }

std::vector<Eigen::AlignedBox2d> SplitAlignedBox(
    const Eigen::AlignedBox2d& bound_box, int num) {
  auto size = bound_box.sizes();
  double x_lenth = size.x() / num;
  double y_lenth = size.y() / num;
  std::vector<double> x_dim;
  for (double start = bound_box.min().x(); start <= bound_box.max().x();) {
    x_dim.push_back(start);
    start += x_lenth;
  }
  std::vector<double> y_dim;
  for (double start = bound_box.min().y(); start <= bound_box.max().y();) {
    y_dim.push_back(start);
    start += y_lenth;
  }
  std::vector<Eigen::AlignedBox2d> result;
  for (int i = 0; i < y_dim.size() - 1; i++) {
    for (int j = 0; j < x_dim.size() - 1; j++)
      result.push_back(
          Eigen::AlignedBox2d(Eigen::Vector2d{x_dim[j], y_dim[i]},
                              Eigen::Vector2d{x_dim[j + 1], y_dim[i + 1]}));
  }
  return result;
}


 void PubTarget(const std::vector<Eigen::AlignedBox2d>& tagets)
 {
   visualization_msgs::Marker mark;
   mark.header.frame_id = "map";
   std::string namesring ="target";
   mark.ns = namesring.c_str();
   mark.header.stamp = ::ros::Time::now();
   mark.id = 0;
   mark.action = visualization_msgs::Marker::ADD;
   mark.type = visualization_msgs::Marker::POINTS;
   // mark.type = visualization_msgs::Marker::ARROW;

   mark.lifetime = ros::Duration(0);
   mark.scale.x = 1;
   mark.scale.y = 1;
   mark.scale.z = 1;

   mark.color.r = 1;     // 1.0;
   mark.color.a = 1;       // ran(e);
   mark.color.g = 0;  //(mark_id / sizeofils);
   mark.color.b = 0;  //(sizeofils- mark_id) / sizeofils;

    mark.pose.orientation.w = 1.0;


   for (const auto& target : tagets) {
     geometry_msgs::Point point;
     point.x = target.center().x();
     point.y = target.center().y();
     point.z = 0;
     
     mark.points.push_back(point);
   }
 // visualization_msgs::MarkerArray marks;
  //marks.markers.push_back(mark);
  mark_target_pub.publish(mark);

 }

void  ComuteCoarPOse()
{



//  cartographer::mapping::AddPreLocationPose(
//       cartographer::transform::Embed3D(pose_estimates.begin()->second));

}


void FastCorrlationOneLocationMultTargetScanMatch(const MapById<SubmapId, PoseGraphInterface::SubmapData>&submap_datas){
  CeresScanMatcher2D ceres_scan_matcher(CreatCeresScanMatchingOption());
  
  std::vector<std::pair<float, Rigid2d>> pose_estimates;
  cartographer::sensor::PointCloud point_cloud;
  {
    std::unique_lock<std::mutex> lock(mutx);
    point_cloud = point_clouds;
  }
  LOG(INFO)<<"FastCorrlationScanMatch";


  auto voxel_options = CreateVoxelFilterOptions();
   // point_cloud = cartographer::sensor::AdaptiveVoxelFilter(point_cloud,voxel_options);
  point_cloud =   sensor::VoxelFilter(point_cloud, 0.1);

  // point_cloud = TransformPointCloud(
  //     point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf{
  //                       1.745329222, Eigen::Vector3f{0, 0, 1}}));

  const Grid2D& grid = *std::static_pointer_cast<const Submap2D>(
                            submap_datas.begin()->data.submap)
                            ->grid();

  LOG(INFO) << "grid.limits" << grid.limits().max();
  LOG(INFO) << "grid.num" << grid.limits().cell_limits().num_x_cells << " "
            << grid.limits().cell_limits().num_y_cells;

  Eigen::AlignedBox2d bounding_box(grid.limits().max());
  bounding_box.extend(
      grid.limits().max() -
      grid.limits().resolution() *
          Eigen::Vector2d(grid.limits().cell_limits().num_y_cells,
                          grid.limits().cell_limits().num_x_cells));



  std::vector<Eigen::AlignedBox2d>   little_bounding_boxs =  SplitAlignedBox(bounding_box,3);

int i = 0;

for (auto little_box : little_bounding_boxs) {
  PubTarget(little_bounding_boxs);
  Rigid2d pose_estimate;
  float score;
  const auto options = CreateFastCorrelativeScanMatcherTestOptions2D(
      7, std::max(little_box.sizes().x(), little_box.sizes().y()));

  constexpr float kMinScore = 0.2f;
  FastCorrelativeScanMatcher2D fast_correlative_scan_matcher(grid, options);
  transform::Rigid2d init_pose =
      transform::Rigid2d::Translation((little_box.center()));
  fast_correlative_scan_matcher.Match(init_pose, point_cloud, kMinScore, &score,
                                      &pose_estimate);

  pose_estimate =
      Rigid2d{pose_estimate.translation(),
              Rigid2d::Rotation2D(pose_estimate.rotation().angle())};
  // ceres::Solver::Summary unused_summary;
  // ceres_scan_matcher.Match(pose_estimate.translation(), pose_estimate,
  //                          point_cloud, grid, &pose_estimate,
  //                          &unused_summary);
  pose_estimates.push_back({score, pose_estimate});
  LOG(WARNING) << "index " << i << "little_bounding_boxs : " << little_box.min()
               << " " << little_box.max() << "pose_estimate :" << pose_estimate
               << " score : " << score;
  i++;
  }
  std::sort(pose_estimates.begin(), pose_estimates.end(),
            [](const std::pair<float, Rigid2d> lhs,
               const std::pair<float, Rigid2d> rhs) {
              return lhs.first > rhs.first;
            });
   LOG(WARNING)<<"pose_estimates "<<pose_estimates.begin()->first;



  cartographer::mapping::AddPreLocationPose(
      cartographer::transform::Embed3D(pose_estimates.begin()->second));
}

// bool start_relocation = false;
bool HandleRelocationQuery(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  start_relocation = true;
  return true;
}
#define LOCAITON
#define Debug

::ros::Publisher point_cloud_publisher_;
::ros::Publisher floor_point_cloud_publisher_;
::ros::Publisher floor_param_publisher_;
::ros::Publisher local_map_publisher_;
void RestSetMutablePose(::cartographer::transform::proto::Rigid3d* rigid3d) {
  rigid3d->mutable_translation()->set_x(0);
  rigid3d->mutable_translation()->set_y(0);
  rigid3d->mutable_translation()->set_y(0);

  rigid3d->mutable_rotation()->set_x(0);
  rigid3d->mutable_rotation()->set_y(0);
  rigid3d->mutable_rotation()->set_z(0);
  rigid3d->mutable_rotation()->set_w(1);
}

void GridToOccupan(
    std::shared_ptr<Grid2D> grid) {

  cartographer::mapping::ValueConversionTables conversion_tables;
  cartographer::mapping::proto::Submap2D proto_submap;
  RestSetMutablePose(proto_submap.mutable_local_pose());
  proto_submap.set_finished(1);
  *proto_submap.mutable_grid() = grid->ToProto();
  // proto_submap.mutable_grid()->has_probability_grid_2d
  cartographer::mapping::PoseGraphInterface::SubmapData one_submap;
  std::shared_ptr<const Submap2D> submap_ptr =
      std::make_shared<const Submap2D>(proto_submap, &conversion_tables);
  one_submap.submap = submap_ptr;
  one_submap.pose = transform::Rigid3d::Identity();

  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      slices;
  auto& slice = slices[cartographer::mapping::SubmapId{0, 0}];
  cartographer::io::FillSubmapSlice(one_submap.pose,
                                    one_submap.submap->ToProto(true), &slice,
                                    &conversion_tables);

  auto painted_slices = PaintSubmapSlices(slices, 0.05);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr =
      CreateOccupancyGridMsg(painted_slices, 0.05 , "map", ros::Time::now());
  local_map_publisher_.publish(*msg_ptr);

}

void LocalMapCallBack(std::shared_ptr<sensor::PointCloud> point_cloud,
                      std::shared_ptr<Grid2D> grid,double score) {
  ::cartographer::sensor::TimedPointCloud time_point_cloud;
  std_msgs::Float32 pose_score;
  pose_score.data = score;
  pose_score_pub.publish(pose_score);


  cartographer_ros_msgs::score_client score_data;
  score_data.request.score = score;

  if (pose_score_client.call(score_data)) {
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~ score = %f ", score);
  } else {
    ROS_ERROR("Failed to call service");
  }

  if(point_cloud!=nullptr){
  for (auto point : *point_cloud) {
    time_point_cloud.push_back({point.position, 0.0});
  }
  }
  auto msg = ToPointCloud2Message(ToUniversal(FromRos(ros::Time::now())), "map",
                                  time_point_cloud);
  point_cloud_publisher_.publish(msg);

  if(grid==nullptr){
    return;
  }


  ///
  GridToOccupan(grid);
}
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) {
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}
// #define  FLOOR_OPTION
#ifdef FLOOR_OPTION
void PlaneOptimazatgionCallBack(
const FloorData &floor_data) {
  //先发一个过来
  ::cartographer::sensor::TimedPointCloud time_point_cloud;
  for (auto const& point : floor_data.point_cloud) {
    time_point_cloud.push_back({point.position, 0.0});
  }
  auto msg = ToPointCloud2Message(ToUniversal(FromRos(ros::Time::now())),
                                  "map", time_point_cloud);
  floor_point_cloud_publisher_.publish(msg);

  //

  auto q = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0),
                                              floor_data.floor_param.head<3>());

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.header.stamp   = ros::Time::now();
  pose.pose.orientation.w  = q.w();
  pose.pose.orientation.x  = q.x();
  pose.pose.orientation.y  = q.y();
  pose.pose.orientation.z  = q.z();

  // geometry_msgs::Vector3 floor_vector;
  // floor_vector.x   = floor_data.floor_param.x();
  // floor_vector.y   = floor_data.floor_param.y();
  // floor_vector.z   = floor_data.floor_param.z();
  floor_param_publisher_.publish(pose);

}
#endif

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  signal(SIGINT, termin_out);
  signal(SIGTERM, termin_out);

  ros::NodeHandle nh("");
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  auto map_builder_ = map_builder.get();
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  LOG(INFO)<<"start main";
  //
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, true, trajectory_nodes);
    std::cout << "size of trajectory_nodes: " << trajectory_nodes.size()
              << std::endl;
  }
  //


#ifdef FLOOR_OPTION
  auto floor_optionzation = std::make_unique<FloorPlaneOptimazationWrapper>(
      &PlaneOptimazatgionCallBack);
  scan_matching::SetFloorPlaneOptimazationWrapper(
      std::move(floor_optionzation));
  floor_point_cloud_publisher_ =
      nh.advertise<sensor_msgs::PointCloud2>("/floor_point2", 10);
  floor_param_publisher_ =
      nh.advertise<geometry_msgs::PoseStamped>("/floor", 10);

#endif

#ifdef Debug
if (FLAGS_load_state_filename.empty()) {
  LOG(INFO) << "start StartTrajectoryWithDefaultTopics";
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  ///
  LOG(INFO) << "start spin";
  ::ros::spin();
  // acml_thread.join();
  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        false /* include_unfinished_submaps */);
  }
return;}
    const MapById<SubmapId, PoseGraphInterface::SubmapData> submap_data =
        GetSubmapDataFromPbstream(FLAGS_load_state_filename);
    std::cout << submap_data.size() << std::endl;
    LOG(INFO) << "start comutation";
    // PoseGraphInterface::SubmapData one_submap =
    // ToOneSubmap(FLAGS_load_state_filename);
    PoseGraphInterface::SubmapData one_submap = ToOneSubmap(map_builder_);
    MapById<SubmapId, PoseGraphInterface::SubmapData> one_submap_byid;
    const Grid2D& grid =
        *std::static_pointer_cast<const Submap2D>(one_submap.submap)->grid();

    LOG(INFO) << "grid.limits" << grid.limits().max();
    LOG(INFO) << "grid.num" << grid.limits().cell_limits().num_x_cells << " "
              << grid.limits().cell_limits().num_y_cells;

    one_submap_byid.Insert(SubmapId{0, 0}, one_submap);
    ros::NodeHandle private_node("~");

    ros::ServiceServer service =
        nh.advertiseService("relocation", HandleRelocationQuery);

    ros::Subscriber scan_subscribe =
        private_node.subscribe<sensor_msgs::LaserScan>("/scan", 1,
                                                       &HandleLaserScanMessage);

    ros::Subscriber inite_pose_subscriber =
        private_node.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,
                                                            &HandleInitPoseMessage);                                        

    ros::Subscriber point_cloud_subscribe =
        private_node.subscribe<sensor_msgs::PointCloud2>(
            "/scan_matched_points2", 1, &HandlePointCloudScanMessage);
    ros::Subscriber odom_subscribe = private_node.subscribe<nav_msgs::Odometry>(
        "/odom", 1, &HandleOdomMessage);
    point_cloud_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>("/local_map_point2", 10);
    local_map_publisher_ =
        nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 10);

    mark_target_pub =
        private_node.advertise<visualization_msgs::Marker>("target", 10);
    pose_score_pub =
        private_node.advertise<std_msgs::Float32>("pose_score", 10);
    // const Grid2D&grid =  *std::static_pointer_cast<const
    // Submap2D>(one_submap.submap)->grid();
  pose_score_client = nh.serviceClient<cartographer_ros_msgs::score_client>("pose_score_client");


   auto promis = std::async(std::launch::async, [=]() {
     while(!start_relocation)
     {
       if(kill_thread)
       {
         return;
       }

       sleep(1);
     }
      // std::thread fast_thread([=]() {

      // ComuteCoarPOse();
      //  FastCorrlationOneLocationMultTargetScanMatch(one_submap_byid);

      //  FastCorrlationScanMatch(submap_data);
      // while (!start_relocation) {
        // usleep(100000);
      // }
    LOG(INFO)<<"start relocation";
      while (!ReciveFlag) {
        if (kill_thread) {
          return;
        }
        sleep(1);
      }
      ReciveFlag = false;
      auto curr_pose = FastCorrlationScanMatchWithOneSubmapAddMultySubmap(
          one_submap_byid, submap_data);
      LOG(INFO) << "curr_pose" << curr_pose;
      //找到初始的nodeid
      auto pose_graph = static_cast<PoseGraph*>(map_builder_->pose_graph());
      MapById<NodeId, TrajectoryNodePose> nodes_pose =
          pose_graph->GetTrajectoryNodePoses();
      auto trajectories_state = pose_graph->GetTrajectoryStates();
      int active_id = 1;
      // if (trajectories_state.size() >= 1) {
      //   for (const auto& it : trajectories_state) {
      //     if (it.second == PoseGraphInterface::TrajectoryState::ACTIVE) {
      //       LOG(INFO) << "active id " << it.first;
      //       active_id = it.first;
      //     }
      //   }
      // }

      auto it = nodes_pose.BeginOfTrajectory(
          1);  // std::prev(nodes_pose.EndOfTrajectory(0));
      auto first_node_data = (it)->data;
      // fish submap

      // cartographer::mapping::RestActiveSubmap();

      // curr_pose = transform::Rigid3d::Identity();
      // auto first_node_data = it->data;
      std::vector<std::pair<NodeId, TrajectoryNodePose>> constraint_nodes;
      int i = 0;
      for (; it != nodes_pose.EndOfTrajectory(1); ++it) {
        if (++i > 2) break;
        constraint_nodes.push_back({it->id, it->data});
      }
      MapById<SubmapId, PoseGraphInterface::SubmapPose>
          all_submap_data;  //= pose_graph->GetAllSubmapPoses().trajectory(0);
      for (auto submap : pose_graph->GetAllSubmapPoses()) {
        if (submap.id.trajectory_id == 0) {
          all_submap_data.Insert(submap.id, submap.data);
        }
      }

      float distatnce = FLT_MAX;
      std::pair<SubmapId, PoseGraphInterface::SubmapPose> min_distance_submap(
          {0, 0}, {});
      for (auto submap_pose : all_submap_data) {
        float dis = (submap_pose.data.pose.translation() -
                     (curr_pose * first_node_data.global_pose).translation())
                        .norm();
        if (dis <= distatnce) {
          distatnce = dis;
          min_distance_submap.first = submap_pose.id;
          min_distance_submap.second = submap_pose.data;
        }
      }

      std::vector<PoseGraphInterface::Constraint> constraints;
      LOG(INFO) << "min_distance_submap " << min_distance_submap.first
                << "pose "
                << transform::Project2D(min_distance_submap.second.pose);

      for (auto constraint_node : constraint_nodes) {
        LOG(INFO) << "id " << constraint_node.first << "pose "
                  << transform::Project2D(curr_pose *
                                          constraint_node.second.global_pose);
        const transform::Rigid3d constraint_transform =
            min_distance_submap.second.pose.inverse() * curr_pose *
            constraint_node.second.global_pose;

        constraints.push_back(PoseGraphInterface::Constraint{
            min_distance_submap.first,
            constraint_node.first,
            {constraint_transform, 1e5, 1e5},
            PoseGraphInterface::Constraint::INTER_SUBMAP});
      }
      // constraints.push_back(PoseGraphInterface::Constraint{
      //       {0,0},
      //       {1,0},
      //       {transform::Rigid3d::Identity(), 1e10, 1e10},
      //       PoseGraphInterface::Constraint::INTER_SUBMAP});

      //
      pose_graph->AddSerializedConstraints(constraints);
      pose_graph->RunFinalOptimization();
#ifdef LOCAITON
      if (!FLAGS_load_state_filename.empty()) {
        auto pose_graph = static_cast<cartographer::mapping::PoseGraph2D*>(
            map_builder_->pose_graph());
        auto local_globle_fustion =
            std::make_unique<cartographer::mapping::LocalGloblePoseFusion>(
                cartographer::mapping::LocalGloblePoseFusionOption{},
                pose_graph, LocalMapCallBack);
        local_globle_fustion->SetLocalMap(
            std::make_shared<PoseGraphInterface::SubmapData>(one_submap));
        cartographer::mapping::SetLocalGlobleFusion(
            std::move(local_globle_fustion));
      }
#endif
  });
 
  std::thread sub_base_pose_thread([]() { SubScribleBaseTf(); });
  
  // std::thread acml_thread([&]() {
  //   while (promis.wait_for(std::chrono::seconds(0)) !=
  //          std::future_status::ready  ) {
  //     if(kill_thread)return ;
  //     std::this_thread::yield();
  //   }
  //   sleep(2);
  //   transform::Rigid2d init_pose;
  //   {
  //     std::unique_lock<std::mutex> lock(pose_mutex);
  //     condition_pose.wait(lock);
  //     init_pose = robot_pose;
  //   }

  //   AmclLocaton acml_location({{100, 2, 2 , 0.5}, {.002, 0.1, 100.5, 0.002}},
  //                             init_pose * transform::Rigid2d({2, 2}, 0.5),
  //                             &grid);
  //   //  AmclLocaton acml_location({1000, {0.1, 0.001, 0.5, 0.01}},
  //   //                            &grid);
  //   AcmlProcess(&acml_location);
  // });

#endif

  
LOG(INFO)<<"start StartTrajectoryWithDefaultTopics";
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }
///
LOG(INFO)<<"start spin";
  ::ros::spin();
  // acml_thread.join();
  sub_base_pose_thread.join();
  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        false/* include_unfinished_submaps */);
  }
  // if (!FLAGS_save_state_filename.empty()) {
  //   MergeTrajectory merge_trajector;
  //   merge_trajector.Merge(FLAGS_save_state_filename);
  // }

}


}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
