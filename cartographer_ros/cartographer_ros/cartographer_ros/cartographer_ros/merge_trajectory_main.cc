

#include "cartographer/mapping/merge_trajectory.h"
#include "ros/ros.h"
#include "gflags/gflags.h"



DEFINE_string(pbstream, "", "pbsream in merge");
int main(int argc, char** argv) 
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_alsologtostderr = true;
  CHECK(!FLAGS_pbstream.empty())
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  MergeTrajectory merge;
  merge.Merge(FLAGS_pbstream);
  ::ros::init(argc, argv, "pbsream_merge");
  ::ros::start();

}