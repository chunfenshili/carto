#ifndef _SUBMAP_PROCESS_H
#define  _SUBMAP_PROCESS_H
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder_interface.h"
 cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     const std::string& file,bool save = false);

     
cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
    const cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapData>& submaps);

cartographer::mapping::MapById<
    cartographer::mapping::SubmapId,
    cartographer::mapping::PoseGraphInterface::SubmapData>
GetSubmapDataFromPbstream(const std::string& file);

 cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     cartographer::mapping::MapBuilderInterface* map_builder_) ;

cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
     std::ifstream* in_infile);

cartographer::mapping::PoseGraphInterface::SubmapData ToOneSubmap(
    cartographer::mapping::PoseGraphInterface::SubmapData* submap,
    const std::vector<cartographer::mapping::TrajectoryNode>&
        trajector_data);
bool OneSubmapToFile(
    cartographer::mapping::PoseGraphInterface::SubmapData* submap,
    const std::string& file
        );

#endif