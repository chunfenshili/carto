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

#include "absl/strings/str_split.h"
#include "cartographer_ros/assets_writer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/common/time.h"
#include "cartographer_ros/assets_writer.h"
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read and use the transforms from the bag.");
DEFINE_string(output_file_prefix, "",
              "Will be prefixed to all output file names and can be used to "
              "define the output directory. If empty, the first bag filename "
              "will be used.");
DEFINE_string(pose_input_pose_file, "",
              "Will be prefixed to all output file names and can be used to "
              "define the output directory. If empty, the first bag filename "
              "will be used.");

using namespace cartographer;
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pose_input_pose_file.size() != 0) {
    std::ifstream myfile;
    myfile.open(FLAGS_pose_input_pose_file);
    std::string line;
    pcl::PointCloud<PosePointWithStamp>::Ptr pose_point_;
    pose_point_ = pcl::PointCloud<PosePointWithStamp>::Ptr(
        new pcl::PointCloud<PosePointWithStamp>());

    while (std::getline(myfile, line)) {
      std::istringstream iss(line);
      std::string name;
      int index;
      double x, y, z, qw, qx, qy, qz;
      int64_t tmp;
      if (!(iss >> name >> index >> x >> y >> z >> qw >> qx >> qy >> qz >>
            tmp)) {
        break;
      }  // error
      const common::Time time = common::FromUniversal(tmp);
      const cartographer::transform::Rigid3d pose{{x, y, z}, {qw, qx, qy, qz}};

      PosePointWithStamp pose_point;
      auto timestamp = time;
      pose_point.x = pose.translation().x();
      pose_point.y = pose.translation().y();
      pose_point.z = pose.translation().z();
      pose_point.rw = pose.rotation().w();
      pose_point.rx = pose.rotation().x();
      pose_point.ry = pose.rotation().y();
      pose_point.rz = pose.rotation().z();
      pose_point.time = cartographer::common::ToUniversal(timestamp);
      pose_point_->push_back(pose_point);
    }
    // pcl::io::savePCDFileASCII(FLAGS_pose_input_pose_file + ".pose.pcd",
    //                           *pose_point_);
    pcl::io::savePCDFileBinary(FLAGS_pose_input_pose_file + ".pose.pcd",
                              *pose_point_);
    return 0;
  }

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  CHECK(!FLAGS_pose_graph_filename.empty())
      << "-pose_graph_filename is missing.";

  ::cartographer_ros::AssetsWriter asset_writer(
      FLAGS_pose_graph_filename,
      absl::StrSplit(FLAGS_bag_filenames, ',', absl::SkipEmpty()),
      FLAGS_output_file_prefix);

  asset_writer.Run(FLAGS_configuration_directory, FLAGS_configuration_basename,
                   FLAGS_urdf_filename, FLAGS_use_bag_transforms);
}
