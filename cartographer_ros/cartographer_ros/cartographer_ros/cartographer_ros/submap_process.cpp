#include "submap_process.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_trimmer.h"

// #include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
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
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/common/lua_parameter_dictionary.h"



class DummyFileResolverTest : public cartographer::common::FileResolver {
 public:
  DummyFileResolverTest() {}

  DummyFileResolverTest(const DummyFileResolverTest&) = delete;
  DummyFileResolverTest& operator=(const DummyFileResolverTest&) = delete;

  ~DummyFileResolverTest()  {}

  std::string GetFileContentOrDie(const std::string& unused_basename)  {
    LOG(FATAL) << "Not implemented";
    return {};
  }

  std::string GetFullPathOrDie(const std::string& unused_basename)  {
    LOG(FATAL) << "Not implemented";
    return {};
  }
};

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

cartographer::mapping::ValueConversionTables conversion_tables;

cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterTestOptions2D() {
    std::string code = 
        "return { "
        "insert_free_space =false, "
        "hit_probability = 0.85, "
        "miss_probability = 0.20, "
        "}";
    auto parameter_dictionary = absl::make_unique<common::LuaParameterDictionary>(
        code, absl::make_unique<DummyFileResolverTest>());
 
    return CreateProbabilityGridRangeDataInserterOptions2D(
        parameter_dictionary.get());
}
mapping::ProbabilityGrid CreateProbabilityGrid(
    const double resolution,
    mapping::ValueConversionTables* conversion_tables) {
  constexpr int kInitialProbabilityGridSize = 100;
  Eigen::Vector2d max =
      0.5 * kInitialProbabilityGridSize * resolution * Eigen::Vector2d::Ones();
  return mapping::ProbabilityGrid(
      mapping::MapLimits(resolution, max,
                         mapping::CellLimits(kInitialProbabilityGridSize,
                                             kInitialProbabilityGridSize)),
      conversion_tables);
}
 void RestSetMutablePose(::cartographer::transform::proto::Rigid3d* rigid3d) {
   rigid3d->mutable_translation()->set_x(0);
   rigid3d->mutable_translation()->set_y(0);
   rigid3d->mutable_translation()->set_y(0);

   rigid3d->mutable_rotation()->set_x(0);
   rigid3d->mutable_rotation()->set_y(0);
   rigid3d->mutable_rotation()->set_z(0);
   rigid3d->mutable_rotation()->set_w(1);
 }





}  // namespace

cartographer::mapping::MapById<
    SubmapId,
    PoseGraphInterface::SubmapData>
GetSubmapDataFromPbstream(const std::string& file) {


  MapById<SubmapId, PoseGraphInterface::SubmapData> submap_data_;
  cartographer::mapping::proto::AllTrajectoryBuilderOptions
      all_builder_options_proto;

  ::io::ProtoStreamReader reader(file);
  ::io::ProtoStreamDeserializer deserializer(&reader);
  cartographer::mapping::proto::PoseGraph pose_graph_proto =
      deserializer.pose_graph();

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
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        if (!proto.submap().has_submap_2d()) {
          break;
        }
        // proto.mutable_submap()->mutable_submap_2d()->set_finished(true);
        const transform::Rigid2d global_submap_pose_2d =
            transform::Project2D(submap_poses.at(submap_id));
         std::shared_ptr<const Submap2D> submap_ptr =
            std::make_shared<const Submap2D>(proto.submap().submap_2d(),
                                             &conversion_tables);
        submap_data_.Insert(submap_id, PoseGraphInterface::SubmapData());
        submap_data_.at(submap_id).submap = submap_ptr;
        submap_data_.at(submap_id).pose =
            transform::Embed3D(global_submap_pose_2d);
        break;
      }
      case SerializedData::kNode: {
        break;
      }
      case SerializedData::kTrajectoryData: {
        break;
      }
      default:
        break;
    }
  }
  return submap_data_;
  
}
cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
    const cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapData>& submaps)
 {
  proto::Grid2D proto;
  const MapLimits limits(0.05, {2., 2.}, CellLimits(2., 1.));
  *proto.mutable_limits() = ToProto(limits);
  proto.mutable_cells()->Add(static_cast<uint16>(0));
  proto.mutable_known_cells_box()->set_max_x(0);
  proto.mutable_known_cells_box()->set_max_y(0);
  proto.mutable_known_cells_box()->set_min_x(1);
  proto.mutable_known_cells_box()->set_min_y(1);
  proto.mutable_probability_grid_2d();
  std::unique_ptr<ProbabilityGrid> probability_grid =
      std::make_unique<ProbabilityGrid>(proto, &conversion_tables);
  probability_grid->FinishUpdate();
  auto computer_loocup_table =
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.6));
  for (const auto& submap : submaps) {
    LOG(INFO) << "start merge submap :" << submap.id.trajectory_id << " "
              << submap.id.submap_index;
    Eigen::Array2i offset;
    CellLimits cell_limits;
    const Grid2D& grid =
        *std::static_pointer_cast<const Submap2D>(submap.data.submap)->grid();
    grid.ComputeCroppedLimits(&offset, &cell_limits);

    LOG(INFO) << grid.limits().cell_limits().num_x_cells;
    LOG(INFO) << grid.limits().cell_limits().num_y_cells;
    LOG(INFO) << grid.limits().max();

    const transform::Rigid3d& global_frame_from_submap_frame = submap.data.pose;
    const transform::Rigid3d submap_frame_from_local_frame =
        submap.data.submap->local_pose().inverse();

    // const Eigen::Vector3d grid_limit_from_globle =
    //     global_frame_from_submap_frame * submap_frame_from_local_frame *
    //     Eigen::Vector3d{grid.limits().max().x(), grid.limits().max().y(), 0.0};

     constexpr float kPadding = 1e-6f;
    // probability_grid->GrowLimits(
    //     global_frame_from_submap_frame.translation().cast<float>().head<2>() -
    //     kPadding * Eigen::Vector2f::Ones());

    // probability_grid->GrowLimits((grid_limit_from_globle).cast<float>().head<2>() +
    //                              kPadding * Eigen::Vector2f::Ones());
    
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

      probability_grid->GrowLimits(
          center_of_cell_in_global_frame.translation().cast<float>() -
          kPadding * Eigen::Vector2f::Ones());
      probability_grid->GrowLimits(
          center_of_cell_in_global_frame.translation().cast<float>() +
          kPadding * Eigen::Vector2f::Ones());

      Eigen::Array2i cell_index_in_globle =
          probability_grid->limits().GetCellIndex(
              center_of_cell_in_global_frame.translation().cast<float>().head<2>());

      float problity = dynamic_cast<const ProbabilityGrid*>(
              std::static_pointer_cast<const Submap2D>(submap.data.submap)
                  ->grid())
              ->GetProbability(xy_index);

      if (probability_grid->IsKnown(cell_index_in_globle)) {
         float odd = Odds(problity);
         float oddl1 = Odds(probability_grid->GetProbability(cell_index_in_globle));

         problity = ProbabilityFromOdds(odd * oddl1);

       // float oddl1 =(probability_grid->GetProbability(cell_index_in_globle));
        // problity = std::max(problity,oddl1);
        // if (problity >= probability_grid->GetMaxCorrespondenceCost()) {
        //   problity = probability_grid->GetMaxCorrespondenceCost();
        // }
       }

       probability_grid->SetProbability(cell_index_in_globle, problity);
        probability_grid->ApplyLookupTable(
            cell_index_in_globle,
            computer_loocup_table);
        probability_grid->FinishUpdate();
    }
    LOG(INFO)<<"FinishUpdate";
  
  }
  LOG(INFO)<<"start proto submap";
  
  LOG(INFO)<<probability_grid->limits().cell_limits().num_x_cells;
  LOG(INFO)<<probability_grid->limits().cell_limits().num_y_cells;
  proto::Submap2D  proto_submap;
  RestSetMutablePose(proto_submap.mutable_local_pose());
  proto_submap.set_finished(1);
  *proto_submap.mutable_grid()= probability_grid->ToProto();
  // proto_submap.mutable_grid()->has_probability_grid_2d

   cartographer::mapping::PoseGraphInterface::SubmapData one_submap ;

  LOG(INFO)<<"trance to submap";
   std::shared_ptr<const Submap2D> submap_ptr =
       std::make_shared<const Submap2D>(proto_submap, &conversion_tables);

   one_submap.submap = submap_ptr;
   one_submap.pose = transform::Rigid3d::Identity();

   return one_submap;
 }



 cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     const std::string& file,bool save) {

   mapping::ProbabilityGridRangeDataInserter2D range_data_inserter(
       CreateProbabilityGridRangeDataInserterTestOptions2D());
   mapping::ProbabilityGrid probability_grid =
       CreateProbabilityGrid(0.05, &conversion_tables);

   cartographer::mapping::proto::AllTrajectoryBuilderOptions
       all_builder_options_proto;

   ::io::ProtoStreamReader reader(file);
   ::io::ProtoStreamDeserializer deserializer(&reader);
   cartographer::mapping::proto::PoseGraph pose_graph_proto =
       deserializer.pose_graph();
   all_builder_options_proto = deserializer.all_trajectory_builder_options();
 
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


   while (deserializer.ReadNextSerializedData(&proto)) {
     switch (proto.data_case()) {
       case SerializedData::kPoseGraph:

         LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                       "stream likely corrupt!.";
         break;
       case SerializedData::kAllTrajectoryBuilderOptions:
         LOG(ERROR)
             << "Found multiple serialized "
                "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                "corrupt!.";
         break;
       case SerializedData::kSubmap: {
         break;
       }
       case SerializedData::kNode: {

        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
         transform::Rigid3d node_pose = node_poses.at(node_id);
        

         std::shared_ptr<const TrajectoryNode::Data> constant_data =
             std::make_shared<const TrajectoryNode::Data>(
                 FromProto(proto.node().node_data()));

          transform::Rigid3d pose_imu =
              node_pose * transform::Rigid3d{transform::Rigid3d::Rotation(
                              constant_data->gravity_alignment).inverse()};

          Eigen::Vector3f original{
              static_cast<float>(node_pose.translation().x()),
              static_cast<float>(node_pose.translation().y()), 0.0};


          LOG(INFO)<<constant_data->filtered_gravity_aligned_point_cloud.size();
          if(constant_data->filtered_gravity_aligned_point_cloud.size()==0)continue;
          auto point_cloud = TransformPointCloud(
              constant_data->filtered_gravity_aligned_point_cloud,
              pose_imu.cast<float>());
          //  sensor::RangeData range_data_in_local =
          //       TransformRangeData(constant_data->filtered_gravity_aligned_point_cloud,
          //                   node_pose.cast<float>());

          // transform::Rigid3d pose_imu =  node_pose* transform::Rigid3d{
          //   transform::Rigid3d::Rotation(constant_data->gravity_alignment) };
             
          // Eigen::Vector3f original{pose_imu.translation().x(),pose_imu.translation().y(),0};
        // LOG(INFO)<<original;
         range_data_inserter.Insert(
             {{original},
              point_cloud,
              {}},
             &probability_grid);

         break;
       }
       case SerializedData::kTrajectoryData: {
         break;
       }
       default:
         break;
     }
   }

   proto::Submap2D proto_submap;
   RestSetMutablePose(proto_submap.mutable_local_pose());
   proto_submap.set_finished(1);
   *proto_submap.mutable_grid() = probability_grid.ToProto();
   // proto_submap.mutable_grid()->has_probability_grid_2d

   cartographer::mapping::PoseGraphInterface::SubmapData one_submap ;

   if (save) {
     std::ofstream pro_io(file + ".submap");
     proto_submap.SerializePartialToOstream(&pro_io);
   }

  LOG(INFO)<<"trance to submap";
   std::shared_ptr<const Submap2D> submap_ptr =
       std::make_shared<const Submap2D>(proto_submap, &conversion_tables);

   one_submap.submap = submap_ptr;
   one_submap.pose = transform::Rigid3d::Identity();

   return one_submap;
 }
 //
 cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     cartographer::mapping::MapBuilderInterface* map_builder_) {
   mapping::ProbabilityGridRangeDataInserter2D range_data_inserter(
       CreateProbabilityGridRangeDataInserterTestOptions2D());
   mapping::ProbabilityGrid probability_grid =
       CreateProbabilityGrid(0.05, &conversion_tables);
  
  if(map_builder_==nullptr){
    LOG(INFO)<<"map_bulder empty";
    return {};
  }

  auto trajectories_state = map_builder_->pose_graph()->GetTrajectoryStates();
  std::set<int> frozen_trajectories;
  LOG(INFO) << "trajectories_state.size  = " << trajectories_state.size();
  if (trajectories_state.size() >= 1) {
    for (const auto& it : trajectories_state) {
      if (it.second == PoseGraphInterface::TrajectoryState::FROZEN||
      it.second == PoseGraphInterface::TrajectoryState::FINISHED) {
        frozen_trajectories.insert(it.first);
        LOG(INFO) << "frozen_trajectory is " << it.first;
      }
    }
  }
  auto node_datas = map_builder_->pose_graph()->GetTrajectoryNodes();
  for (const auto& node_data : node_datas) {
    //
    if (frozen_trajectories.count(node_data.id.trajectory_id) == 0) continue;
    //
    transform::Rigid3d pose_imu =
        node_data.data.global_pose *
        transform::Rigid3d{transform::Rigid3d::Rotation(
                               node_data.data.constant_data->gravity_alignment)
                               .inverse()};

    if (node_data.data.constant_data->filtered_gravity_aligned_point_cloud
            .size() == 0) {
      continue;
    }

    auto point_cloud = TransformPointCloud(
        node_data.data.constant_data->filtered_gravity_aligned_point_cloud,
        pose_imu.cast<float>());
    Eigen::Vector3f original{
        static_cast<float>(node_data.data.global_pose.translation().x()),
        static_cast<float>(node_data.data.global_pose.translation().y()), 0.0};

    range_data_inserter.Insert({{original}, point_cloud, {}},
                               &probability_grid);
   }

   proto::Submap2D proto_submap;

   RestSetMutablePose(proto_submap.mutable_local_pose());
   proto_submap.set_finished(1);
   *proto_submap.mutable_grid() = probability_grid.ToProto();
   // proto_submap.mutable_grid()->has_probability_grid_2d

   cartographer::mapping::PoseGraphInterface::SubmapData one_submap;

   LOG(INFO) << "trance to submap";
   std::shared_ptr<const Submap2D> submap_ptr =
       std::make_shared<const Submap2D>(proto_submap, &conversion_tables);

   one_submap.submap = submap_ptr;
   one_submap.pose = transform::Rigid3d::Identity();

   return one_submap;
 }

cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     std::ifstream* in_infile) {
   
   proto::Submap2D proto_submap;
  if(!proto_submap.ParsePartialFromIstream(in_infile))return {};
   cartographer::mapping::PoseGraphInterface::SubmapData one_submap;
   LOG(INFO) << "trance to submap";
   std::shared_ptr<const Submap2D> submap_ptr =
       std::make_shared<const Submap2D>(proto_submap, &conversion_tables);
   one_submap.submap = submap_ptr;
   one_submap.pose = transform::Rigid3d::Identity();

   return one_submap;
 }
 cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     cartographer::mapping::PoseGraphInterface::SubmapData* submap,
     const std::vector<cartographer::mapping::TrajectoryNode>& trajector_data) {
   mapping::ProbabilityGridRangeDataInserter2D range_data_inserter(
       CreateProbabilityGridRangeDataInserterTestOptions2D());

   LOG(INFO)<<"to one submap  from trajector_id";

   Grid2D* probability_grid = const_cast<Grid2D*>(
       static_cast<Submap2D*>(const_cast<Submap*>(submap->submap.get()))
           ->grid());

   for (const auto& node_data : trajector_data) {
     transform::Rigid3d pose_imu =
         node_data.global_pose *
         transform::Rigid3d{transform::Rigid3d::Rotation(
                                node_data.constant_data->gravity_alignment)
                                .inverse()};
    //  LOG(INFO) << " point cloud size"
    //            << node_data.constant_data->filtered_gravity_aligned_point_cloud
    //                   .size();
     if (node_data.constant_data->filtered_gravity_aligned_point_cloud.size() ==
         0) {
       continue;
     }

     auto point_cloud = TransformPointCloud(
         node_data.constant_data->filtered_gravity_aligned_point_cloud,
         pose_imu.cast<float>());
     Eigen::Vector3f original{
         static_cast<float>(node_data.global_pose.translation().x()),
         static_cast<float>(node_data.global_pose.translation().y()), 0.0};

     range_data_inserter.Insert({{original}, point_cloud, {}},
                                probability_grid);
   }

   proto::Submap2D proto_submap;
   RestSetMutablePose(proto_submap.mutable_local_pose());
   proto_submap.set_finished(1);
   *proto_submap.mutable_grid() = probability_grid->ToProto();
   // proto_submap.mutable_grid()->has_probability_grid_2d
   cartographer::mapping::PoseGraphInterface::SubmapData one_submap;
   LOG(INFO) << "trance to submap";
   std::shared_ptr<const Submap2D> submap_ptr =
       std::make_shared<const Submap2D>(proto_submap, &conversion_tables);

   one_submap.submap = submap_ptr;
   one_submap.pose = transform::Rigid3d::Identity();
   return one_submap;
 }
 bool OneSubmapToFile(
     cartographer::mapping::PoseGraphInterface::SubmapData* submap,
     const std::string& file) {
   proto::Submap2D proto_submap;
   RestSetMutablePose(proto_submap.mutable_local_pose());
   proto_submap.set_finished(1);

   auto  probability_grid =
       std::static_pointer_cast<const Submap2D>(submap->submap);
   *proto_submap.mutable_grid() = probability_grid->grid()->ToProto();
   std::ofstream pro_io(file);
 return   proto_submap.SerializePartialToOstream(&pro_io);
 }
