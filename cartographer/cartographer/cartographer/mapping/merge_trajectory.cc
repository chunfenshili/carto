
#include <algorithm>
#include <fstream>
#include <iostream>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

#include "cartographer/mapping/pose_graph_trimmer.h"
#include "absl/memory/memory.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/math.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform_interpolation_buffer.h"

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/merge_trajectory.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"
namespace {
::cartographer::mapping::proto::PoseGraph pose_graph_;
namespace carto = ::cartographer;
using namespace cartographer;
using carto::mapping::MapById;
using carto::mapping::NodeId;
using carto::mapping::SubmapId;
using carto::mapping::TrajectoryNode;
using carto::mapping::proto::SerializedData;
using carto::mapping::PoseGraphInterface;
// using cartographer::mapping::proto::PoseGraph;
using namespace cartographer::mapping;
static constexpr int kMappingStateSerializationFormatVersion = 2;
carto::mapping::proto::SerializationHeader CreateHeader() {
  carto::mapping::proto::SerializationHeader header;
  header.set_format_version(kMappingStateSerializationFormatVersion);
  return header;
}

}  // namespace

class PbstreamTrimmable : public cartographer::mapping::Trimmable {
 public:
  virtual ~PbstreamTrimmable() {}
  PbstreamTrimmable(carto::io::ProtoStreamReaderInterface* const reader) {
    LoadState(reader);
  }
  int num_submaps(int trajectory_id)const {
    return submap_data_.SizeOfTrajectoryOrZero(trajectory_id);
  }

  std::vector<SubmapId> GetSubmapIds(int trajectory_id) const {
    std::vector<SubmapId> submap_ids;
    for (const auto& it : submap_data_.trajectory(trajectory_id)) {
      if (std::find(trimmble_submaps_.begin(), trimmble_submaps_.end(),
                    it.id) != trimmble_submaps_.end())
        continue;
      submap_ids.push_back(it.id);
    }
    return submap_ids;
  }
  MapById<SubmapId, PoseGraphInterface::SubmapData> GetOptimizedSubmapData()const {
    MapById<SubmapId, PoseGraphInterface::SubmapData> submap_data;
    for (const auto& submap : submap_data_) {
      if (std::find(trimmble_submaps_.begin(), trimmble_submaps_.end(),
                    submap.id) != trimmble_submaps_.end())
        continue;
      submap_data.Insert(submap.id, submap.data);
    }
    return submap_data;
  }
  const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes()const {
    return trajectory_nodes_;
  }
  const std::vector<PoseGraphInterface::Constraint>& GetConstraints()const {
    return constraints_;
  }

  void TrimSubmap(const SubmapId& submap_id) {
    LOG(INFO) << "trimemble submap_trajectory " << submap_id.trajectory_id
              << "   with submap index  " << submap_id.submap_index;
    trimmble_submaps_.push_back(submap_id);
    //  trimmble_submaps_.push_back({0,15});
    //  trimmble_submaps_.push_back({0,10});
    //  trimmble_submaps_.push_back({0,11});
  }
  void GetTrimmbleInfo(std::map<NodeId, SubmapId>& nodes,
                       std::vector<SubmapId>& trim_submap) {
    nodes = nodes_;
    trim_submap = trimmble_submaps_;
  }

  bool IsFinished(int trajectory_id) const{ return true; }

  void SetTrajectoryState(int trajectory_id,
                          PoseGraphInterface::TrajectoryState state){};

  const std::map<int, PoseGraphInterface::TrajectoryState> GetTrajectoryStates()
      const {
    std::map<int, PoseGraphInterface::TrajectoryState> result;
    for (const auto trajectory : all_trajectories_) {
      result.emplace(trajectory.trajectory_id(),
                     PoseGraphInterface::TrajectoryState::FINISHED);
    }
    return result;
  }
  void TrimLongWay() {
    std::map<int, SubmapId> submaps_max_cell_num;
    for (const auto& submap : submap_data_) {
      if (submap_nodes_.count(submap.id) != 0) {
        if (submap_nodes_.at(submap.id).size() < 50) {
          if (all_trajectories_.rbegin()->trajectory_id() !=
              submap.id.trajectory_id)
            trimmble_submaps_.push_back((submap.id));
        }
      }

      // const Grid2D& grid =
      //     *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();
      // Eigen::Array2i offset;
      // CellLimits cell_limits;
      // grid.ComputeCroppedLimits(&offset, &cell_limits);
      // if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
      //   LOG(WARNING) << "Empty grid found in submap ID = " << submap.id;
      //   continue;
      // }
      // int max_num_cell = cell_limits.num_x_cells * cell_limits.num_y_cells;
      // LOG(INFO)<<"cell_num "<<cell_limits.num_x_cells<<"cell _num y"<<cell_limits.num_y_cells;
      // submaps_max_cell_num.insert(
      //     {max_num_cell,
      //      SubmapId{submap.id.trajectory_id, submap.id.submap_index}});
    }
    // for (const auto& submap : submaps_max_cell_num) {
    //   LOG(INFO) << "num cell :" << submap.first << "whith submap id "
    //             << submap.second.trajectory_id << " "
    //             << submap.second.submap_index;
    // }
    // auto it =  submaps_max_cell_num.rbegin();
    // for(int i =0;i<10;i++){
    //   trimmble_submaps_.push_back((it++)->second);
    // }
   
    // int average_cell =  
    // std::accumulate(submaps_max_cell_num.begin(),
    //                              submaps_max_cell_num.end(), 0,
    //                              [](int acc, std::pair<int,SubmapId> p) {
    //                                return (acc + p.first);
    //                              }) /
    //                  submaps_max_cell_num.size();
    // LOG(INFO)<<average_cell;

    // std::map<int,SubmapId> converance_cell_;

    // for (const auto& submap : submaps_max_cell_num) {
    //   int con =
    //       (abs(submap.first - average_cell) );
    //   LOG(INFO) << "num cell :" << con << "whith submap id "
    //             << submap.second.trajectory_id << " "
    //             << submap.second.submap_index;

    //   converance_cell_.insert({con, SubmapId{submap.second.trajectory_id,
    //                                          submap.second.submap_index}});
    // }

    // int converance =
    //     std::accumulate(converance_cell_.begin(), converance_cell_.end(), 0,
    //                     [](int acc, std::pair<int, SubmapId> p) {
    //                       return (acc + p.first);
    //                     }) /
    //     converance_cell_.size();
    // LOG(INFO) << "converance_cell_ " << converance;

    // for (const auto& submap : converance_cell_) {
    //   LOG(INFO) << "num cell coverence:"
    //             << sqrt(abs(submap.first -converance)) << "whith submap id  "
    //             << submap.second.trajectory_id << " "
    //             << submap.second.submap_index;
    // }
  }

  private:
   bool LoadState(carto::io::ProtoStreamReaderInterface* const reader);
   std::vector<cartographer::mapping::proto::Trajectory> all_trajectories_;
   std::vector<PoseGraphInterface::Constraint> constraints_;
   MapById<NodeId, TrajectoryNode> trajectory_nodes_;
   MapById<SubmapId, PoseGraphInterface::SubmapData> submap_data_;
   std::map<NodeId, SubmapId> nodes_;
   std::map<SubmapId,std::vector<NodeId>> submap_nodes_;
   std::vector<SubmapId> trimmble_submaps_;
   cartographer::mapping::ValueConversionTables conversion_tables_;
   cartographer::mapping::proto::PoseGraph pose_graph_proto;
   cartographer::mapping::proto::AllTrajectoryBuilderOptions
       all_builder_options_proto;
   MergeTrajectory* parent_;
};

bool  PbstreamTrimmable::LoadState(
    carto::io::ProtoStreamReaderInterface* const reader) {
  carto::io::ProtoStreamDeserializer deserializer(reader);
  cartographer::mapping::proto::PoseGraph pose_graph_proto =
      deserializer.pose_graph();
   all_trajectories_ =
      std::vector<proto::Trajectory>(pose_graph_proto.trajectory().begin(),
                                     pose_graph_proto.trajectory().end());
 
  pose_graph_proto = deserializer.pose_graph();
  all_builder_options_proto = deserializer.all_trajectory_builder_options();
  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }
  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }
  SerializedData proto;
  LOG(INFO)<<"load data";
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:

        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        if (!proto.submap().has_submap_2d()) {
          break;
        }
        proto.mutable_submap()->mutable_submap_2d()->set_finished(true);
        const transform::Rigid2d global_submap_pose_2d =
            transform::Project2D(submap_poses.at(submap_id));
        const std::shared_ptr<const Submap2D> submap_ptr =
            std::make_shared<const Submap2D>(proto.submap().submap_2d(),
                                             &conversion_tables_);
        submap_data_.Insert(submap_id, PoseGraphInterface::SubmapData());
        submap_data_.at(submap_id).submap = submap_ptr;
        submap_data_.at(submap_id).pose =
            transform::Embed3D(global_submap_pose_2d);
        break;
      }
      case SerializedData::kNode: {
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        std::shared_ptr<const TrajectoryNode::Data> constant_data =
            std::make_shared<const TrajectoryNode::Data>(
                FromProto(proto.node().node_data()));
        trajectory_nodes_.Insert(node_id,
                                 TrajectoryNode{constant_data, node_pose});

        break;
      }
      case SerializedData::kTrajectoryData: {
        break;
      }
      default:
        break;
    }
  }
  LOG(INFO)<<"constraint";
  for (const auto& constraint : FromProto(pose_graph_proto.constraint())) {
    switch (constraint.tag) {
      case PoseGraphInterface::Constraint::Tag::INTRA_SUBMAP:
       nodes_.emplace(std::pair<NodeId, SubmapId>(constraint.node_id,
                                                  constraint.submap_id));
       submap_nodes_[constraint.submap_id].push_back(constraint.node_id);
       break;
      case PoseGraphInterface::Constraint::Tag::INTER_SUBMAP:
        break;
    }

    const PoseGraphInterface::Constraint::Pose pose = {
        constraint.pose.zbar_ij *
            transform::Rigid3d::Rotation(
                trajectory_nodes_.at(constraint.node_id)
                    .constant_data->gravity_alignment.inverse()),
        constraint.pose.translation_weight, constraint.pose.rotation_weight};
    constraints_.push_back(PoseGraphInterface::Constraint{
        constraint.submap_id, constraint.node_id, pose, constraint.tag});
  }
  return true;
}

std::map<NodeId, int> MergeTrajectory::ToNodeIndex(
    const std::vector<proto::Trajectory>& trajectors) {
  std::map<NodeId, int> result;
  int32_t start_trajectory_nodeid = 0;
  int32_t trajectory_node_id = 0;
  for (const auto &trajectory: trajectors) {
    for (const auto& trajectory_node : trajectory.node()) {
      //   start_trajectory_nodeid + trajectory_node.node_index();
      auto node_id =
          NodeId{trajectory.trajectory_id(), trajectory_node.node_index()};
      if (nodes_.find(node_id) != nodes_.end()) {
        auto it = std::find(trim_submap_.begin(), trim_submap_.end(),
                            nodes_.at(node_id));
        if (it == trim_submap_.end()) {
          result.emplace(node_id, trajectory_node_id);
          trajectory_node_id++;
        }
      }
    }
    start_trajectory_nodeid += trajectors[trajectory.trajectory_id()].node_size();
  }
  return result;
}
std::map<SubmapId, int> MergeTrajectory::ToSubmapndex(
    const std::vector<proto::Trajectory>& trajectors) {
  std::map<SubmapId, int> result;
  int32_t start_submap_nodeid = 0;
  int32_t trajectory_submap_id = 0;
  for (const auto & trajectory:trajectors) {
    for (const auto& trajectory_submap : trajectory.submap()) {
      // int32_t trajectory_submap_id  =start_submap_nodeid +
      //     trajectory_submap.submap_index();
      auto submap_id =SubmapId{trajectory.trajectory_id(), trajectory_submap.submap_index()};
      auto it = std::find(trim_submap_.begin(), trim_submap_.end(), submap_id);
      if (it == trim_submap_.end()) {
        result.emplace(submap_id, trajectory_submap_id);
        trajectory_submap_id++;
      }
    }
    // start_submap_nodeid += trajectors[trajector_id].submap_size();
  }
  return result;
}

struct PbstreamData {
  cartographer::mapping::proto::PoseGraph pose_graph_proto;
  mapping::proto::AllTrajectoryBuilderOptions all_builder_options_proto;
  std::vector<proto::Trajectory> all_trajectories;
  std::vector<SerializedData> node_data_proto;
  std::vector<SerializedData> submap_data_proto;
  std::vector<SerializedData> serialized_data;
  proto::PoseGraph_Constraint constraints;
};

bool  MergeTrajectory::Merge(const std::string&state_filename)
{

  LOG(INFO)<<"start merge";
  //read
  ::io::ProtoStreamReader reader(state_filename);
  ::io::ProtoStreamReader reader1(state_filename);

  LOG(INFO) << "start_trim";
  OverlappingSubmapsTrimmer2D ovelapping_trim(2, 3, 6,true);
  PbstreamTrimmable* trimmer = new PbstreamTrimmable(&reader1);
  trimmer->TrimLongWay();
  ovelapping_trim.Trim(trimmer);
  trimmer->GetTrimmbleInfo(nodes_, trim_submap_);
  delete trimmer;
  LOG(INFO)<< "start deserializer";
  ::io::ProtoStreamDeserializer deserializer(&reader);
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();

  std::vector<proto::Trajectory> all_trajectories =
      std::vector<proto::Trajectory>(pose_graph_proto.trajectory().begin(),
                              pose_graph_proto.trajectory().end());
  // if (all_trajectories.size() < 2) {
  //   LOG(INFO) << "trajectories size  smaller 2,need not to merg";
  //   return false;
  // }
  LOG(INFO)<<"start index ";
  std::map<SubmapId, int> submap_id_index = ToSubmapndex(all_trajectories); 
  std::map<NodeId, int> node_id_index =  ToNodeIndex(all_trajectories);
  
  LOG(INFO) <<submap_id_index.size();
  LOG(INFO) <<node_id_index.size();
  proto::Trajectory  merge_trajectory;
  merge_trajectory.set_trajectory_id(0);
  for (const proto::Trajectory& trajectory : all_trajectories) {
    for (const auto& trajectory_submap : trajectory.submap()) {
      
      proto::Trajectory_Submap merge_trajectory_submap = trajectory_submap;
      auto submap_id = SubmapId{trajectory.trajectory_id(),
                                trajectory_submap.submap_index()};
      if (submap_id_index.count(submap_id) != 0) {
        merge_trajectory_submap.set_submap_index(submap_id_index[submap_id]);
        *merge_trajectory.add_submap() = merge_trajectory_submap;
      }
      // LOG(INFO) << submap_id_index[SubmapId{trajectory.trajectory_id(),
      //                                       trajectory_submap.submap_index()}];
    }
  }
  for (const proto::Trajectory& trajectory : all_trajectories) {
    for (const auto& trajectory_node : trajectory.node()) {
      proto::Trajectory_Node merge_trajectory_node = trajectory_node;
      auto node_id =
          NodeId{trajectory.trajectory_id(), trajectory_node.node_index()};
      if (node_id_index.count(node_id) != 0) {
        merge_trajectory_node.set_node_index(node_id_index[node_id]);
        *merge_trajectory.add_node() = merge_trajectory_node;
      }
    }
  }
  
  proto::PoseGraph merge_pose_graph = pose_graph_proto;
  merge_pose_graph.clear_constraint();
  merge_pose_graph.clear_trajectory();
  *merge_pose_graph.add_trajectory() = merge_trajectory;
  for (const auto& constraint : pose_graph_proto.constraint()) {
    proto::PoseGraph_Constraint merge_constraint = constraint;
    auto node_id = NodeId{merge_constraint.node_id().trajectory_id(),
                          merge_constraint.node_id().node_index()};
    if (node_id_index.count(node_id) == 0) continue;
    auto submap_id = SubmapId{merge_constraint.submap_id().trajectory_id(),
                              merge_constraint.submap_id().submap_index()};
    if (submap_id_index.count(submap_id) == 0) continue;
    int32_t submap_index  =  submap_id_index[submap_id];
    merge_constraint.mutable_submap_id()->set_submap_index(submap_index);
    merge_constraint.mutable_submap_id()->set_trajectory_id(0);
    int32_t node_index = node_id_index[node_id];
    merge_constraint.mutable_node_id()->set_node_index(node_index);
    merge_constraint.mutable_node_id()->set_trajectory_id(0);
    *merge_pose_graph.add_constraint() = merge_constraint;
  }

  carto::io::ProtoStreamWriter writer(
      state_filename.substr(0, state_filename.find(".pbstream")) +
      ".merge.pbstream");
  writer.WriteProto(CreateHeader());
  LOG(INFO)<<"start merge pose_graph";
  proto::SerializedData data;
  *data.mutable_pose_graph() = merge_pose_graph;
  writer.WriteProto(data);
  data.Clear();
  const auto& options_with_sensor_ids_proto =
      all_builder_options_proto.options_with_sensor_ids(0);
  *data.mutable_all_trajectory_builder_options()
       ->add_options_with_sensor_ids() = options_with_sensor_ids_proto;
  writer.WriteProto(data);
  data.Clear();
  LOG(INFO)<<"start merge data";
  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto::Submap merge_submap = proto.submap();
        auto submap_id = SubmapId{proto.submap().submap_id().trajectory_id(),
                                  proto.submap().submap_id().submap_index()};
        if (submap_id_index.count(submap_id) == 0) continue;
        int32_t submap_index = submap_id_index[submap_id];
        merge_submap.mutable_submap_id()->set_trajectory_id(0);
        merge_submap.mutable_submap_id()->set_submap_index(submap_index);
        *data.mutable_submap() = merge_submap;
        break;
      }
      case SerializedData::kNode: {
        proto::Node merge_node;
        merge_node = proto.node();
        auto node_id = NodeId{proto.node().node_id().trajectory_id(),
                              proto.node().node_id().node_index()};
        if (node_id_index.count(node_id) == 0) continue;

        int32_t node_index = node_id_index[node_id];
        merge_node.mutable_node_id()->set_trajectory_id(0);
        merge_node.mutable_node_id()->set_node_index(node_index);
        *data.mutable_node()  =  merge_node;
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto::TrajectoryData merge_trajector_data;
        merge_trajector_data = proto.trajectory_data();
        merge_trajector_data.set_trajectory_id(0);
        *data.mutable_trajectory_data() =  merge_trajector_data;
        break;
      }
     default:break;
        // LOG(WARNING) << "Skipping unknown message type in stream: "
        //              << proto.GetTypeName();
    }
    writer.WriteProto(data);
    data.Clear();
  }
  CHECK(reader.eof());
  return (writer.Close());
}

