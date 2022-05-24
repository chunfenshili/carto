#ifndef _MERGE_TRAJECTOR_
#define  _MERGE_TRAJECTOR_
#include <string>
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include <map>
class PbstreamData;
class MergeTrajectory {
 public:
  bool Merge(const std::string& state_filename);
 private:
  std::map<cartographer::mapping::NodeId,cartographer::mapping::SubmapId >
      nodes_;
  std::vector<cartographer::mapping::SubmapId> trim_submap_;

  std::map<cartographer::mapping::NodeId, int> ToNodeIndex(
      const std::vector<cartographer::mapping::proto::Trajectory>& trajectors);
  std::map<cartographer::mapping::SubmapId, int> ToSubmapndex(
      const std::vector<cartographer::mapping::proto::Trajectory>& trajectors);
  bool LoadState(cartographer::io::ProtoStreamReaderInterface* const reader);
  std::shared_ptr<PbstreamData> pb_data_;
 };

#endif